#include <tellopp/drone1.h>
#include <iostream>
#include <boost/array.hpp>
#include <glog/logging.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/mem.h>
#include <libswscale/swscale.h>
}

using namespace std::chrono_literals;

using boost::asio::ip::udp;

namespace tellopp {
  sdk1_drone::sdk1_drone()
      : _connected(false)
      , _run(true)
      , _work(new boost::asio::io_service::work(_ios))
      , _thread([this] { _ios.run(); })
      //, _state_socket(_ios, udp::endpoint(udp::v4(), 8890))
      , _video_socket(_ios, udp::endpoint(udp::v4(), 6038))
      , _command_socket(_ios)
      , _keep_alive_timer(_ios)
      , _video_thread(&sdk1_drone::receive_video_thread, this)
      , _has_frame(false) {
    //boost::asio::socket_base::receive_buffer_size option(100000);
    //_video_socket.set_option(option);
    _command_socket.open(udp::v4());
    udp::resolver resolver(_ios);
    udp::resolver::query query(udp::v4(), "192.168.10.1", "8889");
    _drone_endpoint = *resolver.resolve(query);

    auto now = std::chrono::system_clock::now().time_since_epoch();
    _time_zero = std::chrono::duration_cast<std::chrono::seconds>(now).count();
  }

  sdk1_drone::~sdk1_drone() {
    land();
    //
    _work.reset();
    _run = false;
    _keep_alive_timer.cancel();
    _command_socket.close();
    //_state_socket.close();
    _video_socket.close();
    _thread.join();
    _video_thread.join();
  }

  static std::string connection_string(int16_t video_port) {
    boost::endian::little_int16_t port_le = boost::endian::native_to_little(video_port);
    char buf[12];
    sprintf(buf, "conn_req:__");
    memcpy(&buf[9], &port_le, sizeof(boost::endian::little_int16_t));
    return buf;
  }

  bool sdk1_drone::connect() {
    start_receive_response();
    _connected = true;
    send_command(connection_string(6038));
    SendDateTime();
    StartVideo();
    SetVideoEncoderRate(VideoBitRate4M);
    //send_command("streamon");
    //start_receive_state();
    start_keep_alive();
    return true;
  }

  void sdk1_drone::start_receive_response()
  {
    static udp::endpoint remote_endpoint;
    static boost::array<char, 2048> recv_buffer;

    _command_socket.async_receive_from(
        boost::asio::buffer(recv_buffer), remote_endpoint,
        [this](const boost::system::error_code& ec,  std::size_t bytes_transferred) {
          if (ec) {
            if (ec != boost::system::errc::operation_canceled)
              LOG(INFO) << "receive response failed: ec" << ec.message();
            return;
          }
          //std::string s(&recv_buffer[0], bytes_transferred);
          //LOG(INFO) << "got response: " << s;
          start_receive_response();
        });
  }

  static void avframeToMat1(const AVFrame * frame, cv::Mat& image)
  {
    int width = frame->width;
    int height = frame->height;

    // Allocate the opencv mat and store its stride in a 1-element array
    if (image.rows != height || image.cols != width || image.type() != CV_8UC3) image = cv::Mat(height, width, CV_8UC3);
    int cvLinesizes[1];
    cvLinesizes[0] = image.step1();

    // Convert the colour format and write directly to the opencv matrix
    SwsContext* conversion = sws_getContext(width, height, (AVPixelFormat) frame->format, width, height, AV_PIX_FMT_BGR24, SWS_FAST_BILINEAR, NULL, NULL, NULL);
    sws_scale(conversion, frame->data, frame->linesize, 0, height, &image.data, cvLinesizes);
    sws_freeContext(conversion);
  }

  static void avframeToMat2(const AVFrame * frame, cv::Mat& image) {
    cv::Mat mat(frame->height, frame->width, CV_8UC3, frame->data[0], frame->linesize[0]);
    mat.copyTo(image);
  }

  void sdk1_drone::receive_video_thread() {
    udp::endpoint remote_endpoint;
    boost::array<unsigned char, 2048> recv_buffer;
    boost::system::error_code error;
    std::vector<unsigned char> raw_frame_buffer;
    while (_run) {
      auto bytes_transferred = _video_socket.receive_from(boost::asio::buffer(recv_buffer), remote_endpoint, 0, error);
      if (error && error != boost::asio::error::message_size)
        throw boost::system::system_error(error);

      raw_frame_buffer.insert(std::end(raw_frame_buffer), &recv_buffer[2], &recv_buffer[bytes_transferred]);

      if (bytes_transferred != 1460) {

        try {
          _frame_decoder.parse(&raw_frame_buffer[0], raw_frame_buffer.size());
          if (_frame_decoder.is_frame_available()) {
            AVFrame frame = _frame_decoder.decode_frame();
            tellopp::spinlock::scoped_lock sl(_lock);
            avframeToMat1(&frame, _last_frame);
            _has_frame = true;
            _video_stable = true;
          }
        } catch (...)
        {
          _video_stable = false;
        }
        raw_frame_buffer.clear();
      }

    }
  }

  bool sdk1_drone::read_frame(cv::Mat* frame){
    if (!_has_frame)
      return false;

    tellopp::spinlock::scoped_lock sl(_lock);
    _has_frame = false;
    _last_frame.copyTo(*frame);
    return true;
  }

  void sdk1_drone::start_keep_alive() {
    _keep_alive_timer.expires_from_now(boost::posix_time::milliseconds(100));
    _keep_alive_timer.async_wait([this](const boost::system::error_code& ec){
      if (ec)
        return;
      StartVideo();
      SendStickCommand2();
      start_keep_alive();
    });
  }

  void sdk1_drone::send_command(std::string command)
  {
    LOG(INFO) << "tx: " << command;
    auto x = _command_socket.send_to(boost::asio::buffer(command.data(), command.size()), _drone_endpoint);
  }

  // TakeOff tells drones to liftoff and start flying.

  void sdk1_drone::take_off() {
    tellopp::spinlock::scoped_lock sl(_lock);
    _command_packet.init(takeoffCommand, 0x68, 0);
    seq++;
    _command_packet.append_le(seq);
    _command_packet.finalize();
    auto x = _command_socket.send_to(boost::asio::buffer(_command_packet.data(), _command_packet.size()), _drone_endpoint);
  }

  void sdk1_drone::throw_take_off() {
    tellopp::spinlock::scoped_lock sl(_lock);
    _command_packet.init(throwtakeoffCommand, 0x48, 0);
    seq++;
    _command_packet.append_le(seq);
    _command_packet.finalize();
    auto x = _command_socket.send_to(boost::asio::buffer(_command_packet.data(), _command_packet.size()), _drone_endpoint);
  }


  void sdk1_drone::land() {
    tellopp::spinlock::scoped_lock sl(_lock);
    _command_packet.init(landCommand, 0x68, 1);
    seq++;
    _command_packet.append_le(seq);
    _command_packet.append(0x00);
    _command_packet.finalize();
    auto x = _command_socket.send_to(boost::asio::buffer(_command_packet.data(), _command_packet.size()), _drone_endpoint);
  }

  void sdk1_drone::palm_land() {
    tellopp::spinlock::scoped_lock sl(_lock);
    _command_packet.init(palmLandCommand, 0x68, 1);
    seq++;
    _command_packet.append_le(seq);
    _command_packet.append(0x00);
    _command_packet.finalize();
    auto x = _command_socket.send_to(boost::asio::buffer(_command_packet.data(), _command_packet.size()), _drone_endpoint);
  }

  void sdk1_drone::flip(fliptype_t t){
    tellopp::spinlock::scoped_lock sl(_lock);
    _command_packet.init(flipCommand, 0x70, 1);
    seq++;
    _command_packet.append_le(seq);
    _command_packet.append(t);
    _command_packet.finalize();
    auto x = _command_socket.send_to(boost::asio::buffer(_command_packet.data(), _command_packet.size()), _drone_endpoint);
  }

  void sdk1_drone::SetExposureLevel(exposure_level_t level){
    tellopp::spinlock::scoped_lock sl(_lock);
    _command_packet.init(exposureCommand, 0x48, 1);
    seq++;
    _command_packet.append_le(seq);
    _command_packet.append(level);
    _command_packet.finalize();
    auto x = _command_socket.send_to(boost::asio::buffer(_command_packet.data(), _command_packet.size()), _drone_endpoint);
  }

  void sdk1_drone::StartVideo()
  {
    tellopp::spinlock::scoped_lock sl(_lock);
    _command_packet.init(videoStartCommand, 0x60, 0);
    _command_packet.append_le(0x0000); // seq = 0
    _command_packet.finalize();
    auto x = _command_socket.send_to(boost::asio::buffer(_command_packet.data(), _command_packet.size()), _drone_endpoint);
  }

  void sdk1_drone::SetVideoEncoderRate(videobitrate_t rate) {
    tellopp::spinlock::scoped_lock sl(_lock);
    _command_packet.init(videoEncoderRateCommand, 0x68, 1);
    seq++;
    _command_packet.append_le(seq);
    _command_packet.append(rate);
    _command_packet.finalize();
    auto x = _command_socket.send_to(boost::asio::buffer(_command_packet.data(), _command_packet.size()), _drone_endpoint);
  }

  void sdk1_drone::SendDateTime() {
    tellopp::spinlock::scoped_lock sl(_lock);
    _command_packet.init(timeCommand, 0x50, 5);
    seq++;
    _command_packet.append_le(seq);

    auto now = std::chrono::system_clock::now().time_since_epoch();
    auto s = std::chrono::duration_cast<std::chrono::seconds>(now);
    now -= s;
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now);
    int seconds = s.count() - _time_zero;
    int minutes = seconds / 60;
    seconds = seconds % 60;
    int milliseconds = ms.count();

    _command_packet.append(0x00); // hour
    _command_packet.append(minutes); // minute
    _command_packet.append(seconds); // second
    _command_packet.append(milliseconds & 0xff); // millisecond
    _command_packet.append(milliseconds >> 8); // millisecond

    _command_packet.finalize();
    auto x = _command_socket.send_to(boost::asio::buffer(_command_packet.data(), _command_packet.size()), _drone_endpoint);
  }

  void sdk1_drone::SendStickCommand2() {
    tellopp::spinlock::scoped_lock sl(_lock);
    _stick_packet.init(stickCommand, 0x60, 11);
    _stick_packet.append_le(0x0000); // seq = 0

    // RightX center=1024 left =364 right =-364
    int16_t axis1 = int16_t(660.0 * _stick_state.rx + 1024.0);

    // RightY down =364 up =-364
    int16_t axis2 = int16_t(660.0 * _stick_state.ry + 1024.0);

    // LeftY down =364 up =-364
    int16_t axis3 = int16_t(660.0 * _stick_state.ly + 1024.0);

    // LeftX left =364 right =-364
    int16_t axis4 = int16_t(660.0 * _stick_state.lx + 1024.0);


    char buf[128];
    sprintf(buf, "stick command: yaw=%4d thr=%4d pit=%4d rol=%4d", axis4, axis3, axis2, axis1);
    //LOG(INFO) << buf;

    //log.debug("stick command: yaw=%04x thr=%04x pit=%04x rol=%04x" %
    //          (axis4, axis3, axis2, axis1))

    // speed control
    //int16_t axis5 = int16_t(_stick_state.throttle);

    //int64_t packedAxis = int64_t(axis1) & 0x7FF | int64_t(axis2 & 0x7FF) << 11 | 0x7FF & int64_t(axis3) << 22 |
    //                     0x7FF & int64_t(axis4) << 33 | int64_t(axis5) << 44;
    //_stick_packet.append(0xFF & packedAxis);
    //_stick_packet.append(packedAxis >> 8 & 0xFF);
    //_stick_packet.append(packedAxis >> 16 & 0xFF);
    //_stick_packet.append(packedAxis >> 24 & 0xFF);
    //_stick_packet.append(packedAxis >> 32 & 0xFF);
    //_stick_packet.append(packedAxis >> 40 & 0xFF);

    _stick_packet.append(((axis2 << 11 | axis1) >> 0) & 0xff);
    _stick_packet.append(((axis2 << 11 | axis1) >> 8) & 0xff);
    _stick_packet.append(((axis3 << 11 | axis2) >> 5) & 0xff);
    _stick_packet.append(((axis4 << 11 | axis3) >> 2) & 0xff);
    _stick_packet.append(((axis4 << 11 | axis3) >> 10) & 0xff);
    _stick_packet.append(((axis4 << 11 | axis3) >> 18) & 0xff);

    auto now = std::chrono::system_clock::now().time_since_epoch();
    auto s = std::chrono::duration_cast<std::chrono::seconds>(now);
    now -= s;
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now);
    int seconds = s.count() - _time_zero;
    int minutes = seconds / 60;
    seconds = seconds % 60;
    int milliseconds = ms.count();

    _stick_packet.append(0x00); // hour
    _stick_packet.append(minutes); // minute
    _stick_packet.append(seconds); // second
    _stick_packet.append(milliseconds & 0xff); // millisecond
    _stick_packet.append(milliseconds >> 8); // millisecond

    _stick_packet.finalize();
    auto x = _command_socket.send_to(boost::asio::buffer(_stick_packet.data(), _stick_packet.size()), _drone_endpoint);
  }


}