#include <tellopp/frame_decoder.h>
#include <opencv2/opencv.hpp>

void frame_decoder::insert(const char* data, size_t sz) {
  uint8_t frame_seq = (uint8_t) data[0];
  uint8_t frame_sub_seq = (uint8_t) data[1];

  if (frame_seq != _current_seq){
    _vectordata.clear();
    _current_seq = frame_seq;
    std::cerr << "starting frame " << (int) _current_seq << std::endl;
    _next_sub_seq = 0;
  }

  // last one??
  if (sz == 1460) {
    if (frame_sub_seq == _next_sub_seq) {
      _vectordata.insert(std::end(_vectordata), &data[2], &data[sz]);
      _next_sub_seq++;
      std::cerr << "got subframe " << (int) frame_sub_seq << std::endl;
    } else {
      std::cerr << "missed sub frame expected " << (int) _next_sub_seq << ", actual " << (int) frame_sub_seq << std::endl;
      _next_sub_seq = 0;
      //missed frames
    }
  } else {
    _vectordata.insert(std::end(_vectordata), &data[2], &data[sz]);

    try {
      //cv::Mat data_mat(720, 1280, CV_8UC2, _vectordata.data(), _vectordata.size());
      cv::Mat data_mat(_vectordata, false);
      cv::imdecode(data_mat, 1, &_frame); //put 0 if you want greyscale
      if (_frame.data != nullptr) {
        _has_frame = true;
        std::cerr << "decoded ok" << std::endl;
      } else {
        _has_frame = false;
        std::cerr << "failed to decod" << std::endl;
      }

    } catch(...)
    {

    }
  }

}
