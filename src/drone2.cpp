#include <tellopp/drone2.h>
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
  sdk2_drone::sdk2_drone()
      : _connected(false)
      , _run(true)
      , _work(new boost::asio::io_service::work(_ios))
      , _thread([this] { _ios.run(); })
      , _state_socket(_ios, udp::endpoint(udp::v4(), 8890))
      , _video_socket(_ios, udp::endpoint(udp::v4(), 11111))
      , _command_socket(_ios)
      , _keep_alive_timer(_ios)
      , _video_thread(&sdk2_drone::receive_video_thread, this)
      , _has_frame(false) {
    //boost::asio::socket_base::receive_buffer_size option(100000);
    //_video_socket.set_option(option);
    _command_socket.open(udp::v4());
    udp::resolver resolver(_ios);
    udp::resolver::query query(udp::v4(), "192.168.10.1", "8889");
    _drone_endpoint = *resolver.resolve(query);
  }

  sdk2_drone::~sdk2_drone() {
    _work.reset();
    _run = false;
    _keep_alive_timer.cancel();
    _command_socket.close();
    _state_socket.close();
    _video_socket.close();
    _thread.join();
    _video_thread.join();
  }

  bool sdk2_drone::connect() {
    start_receive_response();
    _connected = true;
    send_command("command");
    send_command("streamon");
    start_receive_state();
    start_keep_alive();

    return true;
  }

  void sdk2_drone::start_receive_response()
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
          std::string s(&recv_buffer[0], bytes_transferred);
          LOG(INFO) << "got response: " << s;
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

  void sdk2_drone::receive_video_thread() {
    udp::endpoint remote_endpoint;
    boost::array<unsigned char, 2048> recv_buffer;
    boost::system::error_code error;
    std::vector<unsigned char> raw_frame_buffer;
    while (_run) {
      auto bytes_transferred = _video_socket.receive_from(boost::asio::buffer(recv_buffer), remote_endpoint, 0, error);
      if (error && error != boost::asio::error::message_size)
        throw boost::system::system_error(error);

      raw_frame_buffer.insert(std::end(raw_frame_buffer), &recv_buffer[0], &recv_buffer[bytes_transferred]);

      if (bytes_transferred != 1460) {

        try {
          _frame_decoder.parse(&raw_frame_buffer[0], raw_frame_buffer.size());
          if (_frame_decoder.is_frame_available()) {
            AVFrame frame = _frame_decoder.decode_frame();
            tellopp::spinlock::scoped_lock sl(_lock);
            avframeToMat1(&frame, _last_frame);
            _has_frame = true;
          }
        } catch (...)
        {
        }
        raw_frame_buffer.clear();
      }

    }
  }

  void sdk2_drone::start_receive_state()
  {
    static udp::endpoint remote_endpoint;
    static boost::array<char, 2048> recv_buffer;
    _state_socket.async_receive_from(
        boost::asio::buffer(recv_buffer), remote_endpoint,
        [this](const boost::system::error_code& ec,  std::size_t bytes_transferred) {
          if (ec)
            return;
          std::string s(&recv_buffer[0], bytes_transferred);
          //LOG(INFO) << "got state: " << s;
          start_receive_state();
        });
  }

  void sdk2_drone::start_keep_alive() {
    _keep_alive_timer.expires_from_now(boost::posix_time::seconds(5));
    _keep_alive_timer.async_wait([this](const boost::system::error_code& ec){
      if (ec)
        return;
      send_command("command"); // must be send every 5s
      start_keep_alive();
    });
  }

  void sdk2_drone::send_command(std::string command)
  {
    LOG(INFO) << "tx: " << command;
    auto x = _command_socket.send_to(boost::asio::buffer(command.data(), command.size()), _drone_endpoint);
  }

  // TakeOff tells drones to liftoff and start flying.

  void sdk2_drone::take_off() {
    send_command("takeoff");
  }

  void sdk2_drone::land() {
    send_command("land");
  }

  void sdk2_drone::flip(fliptype_t t){
    switch (t){
      case flip_front:
        send_command("flip f");
        break;
      case flip_left:
        send_command("flip l");
        break;
      case flip_back:
        send_command("flip b");
        break;
      case flip_right:
        send_command("flip r");
        break;
    }
  }

  bool sdk2_drone::read_frame(cv::Mat* frame){
    if (!_has_frame)
      return false;

    tellopp::spinlock::scoped_lock sl(_lock);
    _has_frame = false;
    _last_frame.copyTo(*frame);
    return true;
  }

}