//https://github.com/hybridgroup/gobot/blob/master/platforms/dji/tello/driver.go

#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/endian/arithmetic.hpp>
#include <thread>
#include <tellopp/crc.h>
#include <tellopp/spinlock.h>
#include <tellopp/frame_decoder.h>

#pragma once

using namespace std::chrono_literals;

using boost::asio::ip::udp;

namespace tellopp {
  class drone {
  public:
    enum {
      MESSAGE_START = 204
    };

    // the 16-bit messages and commands stored in bytes 6 & 5 of the packet
    enum message_t {
      wifiMessage = 0x001a, // 26
      videoRateQuery = 0x0028, // 40
      lightMessage = 0x0035, // 53
      flightMessage = 0x0056, // 86
      logMessage = 0x1050, // 4176

      videoEncoderRateCommand = 0x0020, // 32
      videoStartCommand = 0x0025, // 37
      exposureCommand = 0x0034, // 52
      timeCommand = 0x0046, // 70
      stickCommand = 0x0050, // 80
      takeoffCommand = 0x0054, // 84
      landCommand = 0x0055, // 85
      flipCommand = 0x005c, // 92
      throwtakeoffCommand = 0x005d, // 93
      palmLandCommand = 0x005e, // 94
      bounceCommand = 0x1053, // 4179
    };

    enum fliptype_t {
      flip_front = 0,
      flip_left = 1,
      flip_back = 2,
      flip_right = 3,
      flip_forward_left = 4,
      flip_back_left = 5,
      flip_back_right = 6,
      flip_forward_right = 7
    };

    enum videobitrate_t {
      // VideoBitRateAuto sets the bitrate for streaming video to auto-adjust.
          VideoBitRateAuto = 0,

      // VideoBitRate1M sets the bitrate for streaming video to 1 Mb/s.
          VideoBitRate1M = 1,

      // VideoBitRate15M sets the bitrate for streaming video to 1.5 Mb/s
          VideoBitRate15M = 2,

      // VideoBitRate2M sets the bitrate for streaming video to 2 Mb/s.
          VideoBitRate2M = 3,

      // VideoBitRate3M sets the bitrate for streaming video to 3 Mb/s.
          VideoBitRate3M = 4,

      // VideoBitRate4M sets the bitrate for streaming video to 4 Mb/s.
          VideoBitRate4M = 5,
    };

    // FlightData packet returned by the Tello
    struct FlightData {
      bool BatteryLow;
      bool BatteryLower;
      int8_t BatteryPercentage;
      bool BatteryState;
      int8_t CameraState;
      bool DownVisualState;
      int16_t DroneBatteryLeft;
      int16_t DroneFlyTimeLeft;
      bool DroneHover;
      bool EmOpen;
      bool EmSky;
      bool EmGround;
      int16_t EastSpeed;
      int16_t ElectricalMachineryState;
      bool FactoryMode;
      int8_t FlyMode;
      int16_t FlySpeed;
      int16_t FlyTime;
      bool FrontIn;
      bool FrontLSC;
      bool FrontOut;
      bool GravityState;
      int16_t GroundSpeed;
      int16_t Height;
      int8_t ImuCalibrationState;
      bool ImuState;
      int8_t LightStrength;
      int16_t NorthSpeed;
      bool OutageRecording;
      bool PowerState;
      bool PressureState;
      int16_t SmartVideoExitMode;
      bool TemperatureHeight;
      int8_t ThrowFlyTimer;
      int8_t WifiDisturb;
      int8_t WifiStrength;
      bool WindState;
    };

// WifiData packet returned by the Tello
    struct WifiData {
      int8_t Disturb = 0;
      int8_t Strength = 0;
    };

    struct stick_state {
      float rx = 0;
      float ry = 0;
      float lx = 0;
      float ly = 0;
      int8_t throttle = 0;
      bool bouncing = false;
    };

    class rx_packet_parser {
      bool init(const char* buf, size_t sz) {
        _buf = buf;
        _remaining = sz;
        if (get_byte()!=MESSAGE_START)
          return false;
        uint16_t xsz = get_le16();
        uint8_t crc8 = get_byte();
      }
        // TODO chech CRC8
      uint16_t get_le16();
      uint8_t  get_byte();

      const char* _buf;
      size_t _remaining;
    };

    class tx_packet {
    public:
      void init(message_t msg, uint8_t pktType, size_t payload_size) {
        _size=0;
        _allocated_size = payload_size + 11;
        uint16_t packet_size = payload_size + 11;
        append(MESSAGE_START);
        append_le(packet_size << 3);
        append(crc8(_buf, 3));
        append(pktType);
        append_le(msg);
      }

      inline void append(uint8_t data) {
        assert(_size + 1 <= _allocated_size);
        _buf[_size] = data;
        ++_size;
      }

      inline void append_le(uint16_t data) {
        assert(_size + 2 <= _allocated_size);
        boost::endian::little_int16_t le = boost::endian::native_to_little(data);
        memcpy(&_buf[_size], &le, sizeof(boost::endian::little_int16_t));
        _size += sizeof(boost::endian::little_int16_t);
      }

      int finalize() {
        assert(_size + 2 == _allocated_size);
        auto crc = crc16(_buf, _size);
        append_le(crc);
      }

      inline const char *data() const {
        return _buf;
      }

      inline size_t size() const {
        return _size;
      }

    private:

      enum {
        MAX_SZ = 128
      };

      char _buf[MAX_SZ];
      size_t _size = 0;
      size_t _allocated_size = 0;
    };

    drone();

    ~drone();

    bool connect();

    int takeOff();

    int land();

    int flip(fliptype_t t);

    bool read_frame(cv::Mat* frame);

  private:
    int SendDateTime();

    int SendStickCommand();
    int SendStickCommand2();

    std::string connection_string(int16_t video_port);

    void process_stick();

    void process_data();

    void process_video();

    bool _run;
    bool _connected;
    tellopp::spinlock _lock;
    boost::asio::io_service _io_service;
    udp::socket _command_socket;
    udp::socket _video_socket;
    udp::endpoint _drone_endpoint;
    tx_packet _command_packet;
    tx_packet _stick_packet;
    stick_state _stick_state;
    uint16_t seq = 0;
    int32_t _time_zero;
    FlightData _flightData;
    frame_decoder _frame_decoder;

    std::thread _data_thread;
    std::thread _video_thread;
    std::thread _stick_thread;
  };
} // namespace