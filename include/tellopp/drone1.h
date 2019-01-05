//https://github.com/hybridgroup/gobot/blob/master/platforms/dji/tello/driver.go

#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <thread>
#include <tellopp/spinlock.h>
#include <tellopp/h264decoder.h>
#include <dlib/opencv.h>
#include "unofficial_tx.h"
#pragma once

using namespace std::chrono_literals;

using boost::asio::ip::udp;

namespace tellopp {

  // move to common
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


  class sdk1_drone {
  public:
      struct stick_state {
        /*inline void fast_mode(){
          throttle = 1;
        }

        inline void slow_mode(){
          throttle = 0;
        }
        */

        /**
        * set_throttle controls the vertical up and down motion of the drone.
        * Pass in an int from -100 - 100. (positive value means upward)
        */
        inline void set_throttle(int val) {
          ly = (val / 100.0);
        }


        /**
         * set_pitch controls the forward and backward tilt of the drone.
         * Pass in an int from -100 - 100. (positive value will make the drone move forward)
         */
        inline void set_pitch(int val) {
          ry = (val / 100.0);
        }


        /**
         *  set_roll controls the the side to side tilt of the drone.
         * Pass in an int from -100 - 100. (positive value will make the drone move to the right)
         */
        inline void set_roll(int val) {
          rx = (val / 100.0);
        }

        /**
         * set_yaw controls the left and right rotation of the drone.
         * Pass in an int from -100 - 100. (positive value will make the drone turn to the right)
         */
        inline void set_yaw(int val) {
          lx = (val / -100.0);
        }

        inline  void hover() {
          rx = 0.0;
          ry = 0.0;
          lx = 0.0;
          ly = 0.0;
        }

      float rx = 0.0;
      float ry = 0.0;
      float lx = 0.0;
      float ly = 0.0;
      //int8_t throttle = 0;
      //bool bouncing = false;
    };

    sdk1_drone();

    ~sdk1_drone();

    bool connect();

    void take_off();

    void throw_take_off();

    void land();

    void palm_land();

    void flip(fliptype_t t);

    bool read_frame(cv::Mat* frame);

    void send_command(std::string);

    void SetVideoEncoderRate(videobitrate_t rate);
    void SetExposureLevel(exposure_level_t level);

    inline const stick_state& stick() const {
      return _stick_state;
    }

    inline stick_state& stick() {
      return _stick_state;
    }

    bool start_video();
    bool stop_video();

    inline bool stable_video() const {
      return _video_stable;
    }


  private:

    void StartVideo();

    void SendDateTime();
    void SendStickCommand2();
    void start_receive_response();
    //void start_receive_state();
    void start_keep_alive();
    void receive_video_thread();

    bool _run;
    bool _connected = false;
    bool _video_stable = false;

    uint16_t seq = 0;
    int32_t _time_zero = 0;
    tx_packet _command_packet;

    stick_state _stick_state;
    tx_packet _stick_packet;

    tellopp::spinlock _lock;
    boost::asio::io_service _ios;
    std::unique_ptr<boost::asio::io_service::work> _work;
    udp::socket _command_socket;
    //udp::socket _state_socket;
    udp::socket _video_socket;
    udp::endpoint _drone_endpoint;
    boost::asio::deadline_timer _keep_alive_timer;
    H264Decoder _frame_decoder;
    cv::Mat     _last_frame;
    bool        _has_frame;
    std::thread _thread;
    std::thread _video_thread;
  };
} // namespace