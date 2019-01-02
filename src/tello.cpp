#include <tellopp/tello.h>

//https://github.com/hybridgroup/gobot/blob/master/platforms/dji/tello/driver.go

#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/endian/arithmetic.hpp>
#include <thread>

using namespace std::chrono_literals;

using boost::asio::ip::udp;

namespace tellopp {


  class le_stream_parser {
  public:
    le_stream_parser(const char* buf, size_t sz) {
      _buf = buf;
      _cursor=buf;
      _remaining = sz;
    }

    // TODO chech CRC8
    inline uint16_t get_le16() {
      uint16_t d = (uint16_t(_cursor[1]) << 8) | uint16_t(_cursor[0]);
      _cursor += 2;
      _remaining -= 2;
      return d;
    }

    inline uint8_t get_byte(){
      uint8_t d = *_cursor;
      ++_cursor;
      --_remaining;
      return d;
    }
  private:
    const char* _buf; // not needed
    const char* _cursor;
    size_t _remaining;
  };

  bool parse(const char* buf, size_t sz, drone::FlightData* data){

    if (sz<24) {
      std::cerr << "Invalid buffer length for flight data packet" << std::endl;
      return false;
    }

    memset(data, 0, sizeof(drone::FlightData));

    le_stream_parser parser(buf, sz);
    data->Height = parser.get_le16();
    data->NorthSpeed = parser.get_le16();
    data->EastSpeed = parser.get_le16();
    data->GroundSpeed = parser.get_le16();
    data->FlyTime = parser.get_le16();

    uint8_t b1 = parser.get_byte();


    data->ImuState = (b1 >> 0 & 0x1) == 1;
    data->PressureState = (b1 >> 1 & 0x1) == 1;
    data->DownVisualState = (b1 >> 2 & 0x1) == 1;
    data->PowerState = (b1 >> 3 & 0x1) == 1;
    data->BatteryState = (b1 >> 4 & 0x1) == 1;
    data->GravityState = (b1 >> 5 & 0x1) == 1;
    data->WindState = (b1 >> 7 & 0x1) == 1;

    data->ImuCalibrationState = parser.get_byte();
    data->BatteryPercentage = parser.get_byte();
    data->DroneFlyTimeLeft = parser.get_le16();
    data->DroneBatteryLeft = parser.get_le16();

    uint8_t f2 = parser.get_byte();

    data->EmSky = (f2 >> 0 & 0x1) == 1;
    data->EmGround = (f2 >> 1 & 0x1) == 1;
    data->EmOpen = (f2 >> 2 & 0x1) == 1;
    data->DroneHover = (f2 >> 3 & 0x1) == 1;
    data->OutageRecording = (f2 >> 4 & 0x1) == 1;
    data->BatteryLow = (f2 >> 5 & 0x1) == 1;
    data->BatteryLower = (f2 >> 6 & 0x1) == 1;
    data->FactoryMode = (f2 >> 7 & 0x1) == 1;

    data->FlyMode = parser.get_byte();
    data->ThrowFlyTimer = parser.get_byte();
    data->CameraState = parser.get_byte();

    data->ElectricalMachineryState = parser.get_byte(); // this seems to intentional in the protocol but why is the state 16bits??

    // sometimes there is a bigger state...
    //TODO check remaing
    uint8_t f3 = parser.get_byte();

    data->FrontIn = (f3 >> 0 & 0x1) == 1;
    data->FrontOut = (f3 >> 1 & 0x1) == 1;
    data->FrontLSC = (f3 >> 2 & 0x1) == 1;

    uint8_t f4 = parser.get_byte();

    data->TemperatureHeight = (f4 >> 0 & 0x1) == 1;

    std::cout << "BatteryPercentage :" << (int) data->BatteryPercentage << std::endl;
  }


  drone::drone()
      : _connected(false), _run(true), _video_socket(_io_service, udp::endpoint(udp::v4(), 6038))
      , _command_socket(_io_service)
      , _data_thread(&drone::process_data, this)
      , _video_thread(&drone::process_video, this)
      , _stick_thread(&drone::process_stick, this) {
    _command_socket.open(udp::v4());

    udp::resolver resolver(_io_service);
    udp::resolver::query query(udp::v4(), "192.168.10.1", "8889");
    //udp::resolver::query query(udp::v4(), "localhost", "8889");
    _drone_endpoint = *resolver.resolve(query);

    auto now = std::chrono::system_clock::now().time_since_epoch();
    _time_zero = std::chrono::duration_cast<std::chrono::seconds>(now).count();
  }

  drone::~drone() {
    _run = false;
    _video_socket.close();
    _data_thread.join();
    _video_thread.join();
    _stick_thread.join();
  }

  bool drone::connect() {
    std::string str = connection_string(6038);
    std::cout << "writing " << str << std::endl;
    auto x = _command_socket.send_to(boost::asio::buffer(str.data(), str.size()), _drone_endpoint);

    boost::array<char, 128> recv_buf;
    udp::endpoint sender_endpoint;

    size_t len = _command_socket.receive_from(boost::asio::buffer(recv_buf), sender_endpoint);
    //std::cout << "got " << len << " bytes" << std::endl;
    //std::cout.write(recv_buf.data(), len);
    _connected = true;
    return true;
  }

  // TakeOff tells drones to liftoff and start flying.
  int drone::takeOff() {
    tellopp::spinlock::scoped_lock sl(_lock);
    _command_packet.init(takeoffCommand, 0x68, 0);
    seq++;
    _command_packet.append_le(seq);
    _command_packet.finalize();
    auto x = _command_socket.send_to(boost::asio::buffer(_command_packet.data(), _command_packet.size()), _drone_endpoint);
    return 0;
  }

  int drone::land() {
    tellopp::spinlock::scoped_lock sl(_lock);
    _command_packet.init(landCommand, 0x68, 1);
    seq++;
    _command_packet.append_le(seq);
    _command_packet.append(0x00);
    _command_packet.finalize();
    auto x = _command_socket.send_to(boost::asio::buffer(_command_packet.data(), _command_packet.size()), _drone_endpoint);
    return 0;
  }

  int  drone::flip(fliptype_t t){
    tellopp::spinlock::scoped_lock sl(_lock);
    _command_packet.init(flipCommand, 0x70, 1);
    seq++;
    _command_packet.append_le(seq);
    _command_packet.append(t);
    _command_packet.finalize();
    auto x = _command_socket.send_to(boost::asio::buffer(_command_packet.data(), _command_packet.size()), _drone_endpoint);
  }

  bool drone::read_frame(cv::Mat* frame){
    if (!_frame_decoder.has_frame())
      return false;

    tellopp::spinlock::scoped_lock sl(_lock);
    _frame_decoder.get_frame(frame);
    return true;
  }

  int drone::SendDateTime() {
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

  int drone::SendStickCommand() {
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

    // speed control
    int16_t axis5 = int16_t(_stick_state.throttle);

    int64_t packedAxis = int64_t(axis1) & 0x7FF | int64_t(axis2 & 0x7FF) << 11 | 0x7FF & int64_t(axis3) << 22 |
                         0x7FF & int64_t(axis4) << 33 | int64_t(axis5) << 44;
    _stick_packet.append(0xFF & packedAxis);
    _stick_packet.append(packedAxis >> 8 & 0xFF);
    _stick_packet.append(packedAxis >> 16 & 0xFF);
    _stick_packet.append(packedAxis >> 24 & 0xFF);
    _stick_packet.append(packedAxis >> 32 & 0xFF);
    _stick_packet.append(packedAxis >> 40 & 0xFF);


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
    return 0;
  }

  int drone::SendStickCommand2() {
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
    std::cerr << buf << std::endl;

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
    return 0;
  }


  std::string drone::connection_string(int16_t video_port) {
    boost::endian::little_int16_t port_le = boost::endian::native_to_little(video_port);
    char buf[12];
    sprintf(buf, "conn_req:__");
    memcpy(&buf[9], &port_le, sizeof(boost::endian::little_int16_t));
    return buf;
  }

  void drone::process_data() {
    while (!_connected && _run) {
      std::this_thread::sleep_for(20ms);
    }

    std::cerr << "starting process data thread" << std::endl;
    try {
      while (_run) {
        boost::array<char, 2048> recv_buf;
        udp::endpoint remote_endpoint;
        boost::system::error_code error;
        auto sz = _command_socket.receive_from(boost::asio::buffer(recv_buf), remote_endpoint, 0, error);
        if (error && error != boost::asio::error::message_size)
          throw boost::system::system_error(error);

        if (((uint8_t) recv_buf[0]) !=MESSAGE_START)
          continue;

        uint16_t msgType = (uint16_t(recv_buf[6]) << 8) | uint16_t(recv_buf[5]);
        switch (msgType){
          case wifiMessage:
            break;
          case lightMessage:
            break;
          case logMessage:
            //std::cout << "logmessage: " << std::string(&recv_buf[9], sz-9) << std::endl;
            break;
          case timeCommand:{
            char buf[128];
            sprintf(buf, "recv: time data");
            // TODO self.__publish(event=self.EVENT_TIME, data=data[7:9])
            std::cout << buf << std::endl;
        }

        break;
          case bounceCommand:
            break;

          //VIDEO_START_CMD, VIDEO_ENCODER_RATE_CMD, PALM_LAND_CMD
          case exposureCommand:
          case videoEncoderRateCommand:
          case takeoffCommand:
          case landCommand:{
            char buf[128];
            uint16_t seq = uint16_t((recv_buf[8]) << 8) | uint16_t(recv_buf[7]);
            sprintf(buf, "recv: ack: cmd=0x%02x seq=0x%04x", msgType, seq);
            std::cout << buf << std::endl;
          }
          break;
          case flipCommand:
            break;
          case flightMessage:
            //const char* buf = &recv_buf[9];
            //size_t payload_size = sz - 11;
            //std::cout << "flight message" << std::endl;
            parse(&recv_buf[9], sz - 11, &_flightData);
            break;
          default:
            std::cerr << "unknown message" <<  msgType << std::endl;
        };

        //std::cout << "got video: " << sz << " bytes" << std::endl;
      }
    }
    catch (std::exception &e) {
      std::cerr << e.what() << std::endl;
    }
    std::cerr << "exiting process data thread" << std::endl;
  }

  void drone::process_video() {
    while (!_connected && _run) {
      std::this_thread::sleep_for(20ms);
    }
    std::cerr << "starting process video thread" << std::endl;
    try {
      while (_run) {
        boost::array<char, 2048> recv_buf;
        udp::endpoint remote_endpoint;
        boost::system::error_code error;
        auto sz = _video_socket.receive_from(boost::asio::buffer(recv_buf), remote_endpoint, 0, error);
        if (error && error != boost::asio::error::message_size)
          throw boost::system::system_error(error);

        {
          tellopp::spinlock::scoped_lock sl(_lock);
          _frame_decoder.insert(&recv_buf[0], sz);
        }

        //std::cout << "got video: " << sz << " bytes" << std::endl;
      }
    }
    catch (std::exception &e) {
      std::cerr << e.what() << std::endl;
    }
    std::cerr << "exiting process video thread" << std::endl;
  }

  void drone::process_stick() {
    {
      while (!_connected && _run) {
        std::this_thread::sleep_for(20ms);
      }

      std::cerr << "process_stick: setting clock" << std::endl;

      SendDateTime();
      std::this_thread::sleep_for(50ms);

      std::cerr << "process_stick: starting stick thread" << std::endl;
      while (_run) {
        SendStickCommand2();
        std::this_thread::sleep_for(200ms);
      }
      std::cerr << "exiting process_stick thread" << std::endl;
    }
  }
}