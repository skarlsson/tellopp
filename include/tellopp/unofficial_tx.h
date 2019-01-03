#include <boost/endian/arithmetic.hpp>
#include "crc.h"
#pragma once

namespace tellopp {

  // should go in API_V1

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
}
