#include <dlib/opencv.h>
#pragma once

class frame_decoder{
public:
  frame_decoder()
  : _current_seq(0)
  , _next_sub_seq(0)
  , _has_frame(false) {

  }

  void insert(const char* data, size_t sz);

  inline bool has_frame() const { return _has_frame; }

  inline bool get_frame(cv::Mat* p) {
    *p = _frame;
    _has_frame = false;
  }

private:
  // move this to a frame decoder
  std::vector<uint8_t> _vectordata;
  cv::Mat _frame;
  uint8_t _current_seq;
  uint8_t _next_sub_seq;
  bool    _has_frame;
};

