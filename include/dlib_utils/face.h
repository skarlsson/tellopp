#include <string>
#include <dlib/matrix.h>
#include <dlib/pixel.h>
#pragma once

class face_recognition {
	public:
	face_recognition();
  void load(std::string model_path);

  std::vector<dlib::matrix<float,0,1>> get_face_descriptors(const std::vector<dlib::matrix<dlib::rgb_pixel>>& faces);
};


