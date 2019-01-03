//https://github.com/hybridgroup/gobot/blob/master/platforms/dji/tello/driver.go

#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <thread>
#include <tellopp/spinlock.h>
#include <tellopp/h264decoder.h>
#include <dlib/opencv.h>
//#include "unofficial_tx.h"
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

  class sdk2_drone {
  public:
    sdk2_drone();

    ~sdk2_drone();

    bool connect();

    void take_off();

    void land();

    void flip(fliptype_t t);

    bool read_frame(cv::Mat* frame);

    void send_command(std::string);

  private:

    void start_receive_response();
    void start_receive_state();
    void start_keep_alive();
    void receive_video_thread();

    bool _run;
    bool _connected;
    tellopp::spinlock _lock;
    boost::asio::io_service _ios;
    std::unique_ptr<boost::asio::io_service::work> _work;
    udp::socket _command_socket;
    udp::socket _state_socket;
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