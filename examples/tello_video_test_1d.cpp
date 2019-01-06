#include <dlib/opencv.h>
#include <opencv2/highgui/highgui.hpp>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>

#include <glog/logging.h>
#include <tellopp/drone1.h>
#include <dlib_utils/face.h>
#include <tellopp/stick_handler.h>

#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <GL/glx.h>

using namespace dlib;
using namespace std;


const char* windowName = "tello basic sample";
const int width = 800;
const int height = 600;

static bool should_exit = false;

inline int64_t milliseconds_since_epoch() {
  return std::chrono::duration_cast<std::chrono::milliseconds>
    (std::chrono::system_clock::now().time_since_epoch()).count();
}

enum Button
{
  PITCH,
  ROLL,
  YAW,
  THROTTLE,
//  PITCH_FWD,
//  PITCH_BACK,
//  ROLL_LEFT,
//  ROLL_RIGHT,
//  YAW_LEFT,
//  YAW_RIGHT,
//  THROTTLE_UP,
//  THROTTLE_DOWN,
    TAKE_OFF,
  THROW_TAKE_OFF,
  LAND,
  PALM_LAND,
  FLIP
  //MouseX,
  //MouseY
};



int main()
{


  //if (SDL_Init( SDL_INIT_VIDEO | SDL_INIT_JOYSTICK| SDL_INIT_KEYBOARD ) < 0)
  if (SDL_Init(SDL_INIT_EVERYTHING))
  {
    fprintf(stderr, "Couldn't initialize SDL: %s\n", SDL_GetError());
    exit(1);
  }

  SDL_Window * window = SDL_CreateWindow("SDL2 Keyboard/Mouse events", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 640, 480, 0);
  SDL_Renderer * renderer = SDL_CreateRenderer(window, -1, 0);
  SDL_Surface * image = SDL_LoadBMP("spaceship.bmp");
  SDL_Texture * texture = SDL_CreateTextureFromSurface(renderer, image);
  SDL_FreeSurface(image);
  SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);


  // Load face detection and pose estimation models.
  frontal_face_detector detector = get_frontal_face_detector();
  shape_predictor pose_model;
  try {
    deserialize("../shape_predictor_68_face_landmarks.dat") >> pose_model;
  }
  catch(serialization_error& e)
  {
    cout << "You need dlib's default face landmarking model file to run this example." << endl;
    cout << "You can get it from the following URL: " << endl;
    cout << "   http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2" << endl;
    cout << endl << e.what() << endl;
    return -1;
  }

  shape_predictor sp;
  try {
    deserialize("../shape_predictor_5_face_landmarks.dat") >> sp;
  }
  catch(serialization_error& e)
  {
    cout << "You need dlib's default face landmarking model file to run this example." << endl;
    cout << "You can get it from the following URL: " << endl;
    cout << "   http://dlib.net/files/shape_predictor_5_face_landmarks.dat.bz2" << endl;
    cout << endl << e.what() << endl;
    return -1;
  }

  face_recognition recognition_net;

  try {
    recognition_net.load("../dlib_face_recognition_resnet_model_v1.dat");
  }
  catch(serialization_error& e) {
    cout << "You need dlib's default recognition model file to run this example." << endl;
    cout << "You can get it from the following URL: " << endl;
    cout << " \"http://dlib.net/files/dlib_face_recognition_resnet_model_v1.dat.bz2" << endl;
    cout << endl << e.what() << endl;
    return -1;
  }

  tellopp::sdk1_drone d;

  volatile bool landing = false;
  volatile bool landed = false;
  volatile bool search_mode = false;

  stick_handler s_handler;
  // keyboard controll thread

  s_handler.set_button_handler([&](stick_handler::logical_button_t b, stick_handler::button_event_t ev){
    if (ev==stick_handler::BUTTON_DOWN) {
      switch (b){
        case stick_handler::TAKE_OFF:
          LOG(INFO) << "TAKE_OFF";
          d.take_off();
          break;
        case stick_handler::THROW_TAKE_OFF:
          LOG(INFO) << "THROW_TAKE_OFF";
          d.throw_take_off();
          break;
        case stick_handler::LAND:
          LOG(INFO) << "LAND";
          d.land();
          break;
        case stick_handler::PALM_LAND:
          LOG(INFO) << "PALM_LAND";
          d.palm_land();
          break;
        default:
          LOG(INFO) << "UNHANDLED EVENT:" << b;
      }
    }
  });

  std::thread t2([&](){
    //int64_t hover_timeout = 0;

    while(!should_exit)
    {
      SDL_Event event;
      //While there's events to handle
      while( SDL_PollEvent( &event ) )
      {
        if (event.type==SDL_QUIT){
          should_exit = true;
          break;
        }
        s_handler.handle_event(event);
      }

      //auto  x = s_handler.get_raw_state();
      /*for (auto s : x)
        LOG(INFO)  << "state " << s.first << ":" << s.second;
      */

      auto state = s_handler.get_state();
      //LOG(INFO) << "[" << state.get_roll() << ", " << state.get_pitch() << ", " << state.get_yaw() << ", " << state.get_throttle() << "]";
      d.stick() = state;
      std::this_thread::sleep_for(10ms);
    }
  });

  d.connect();
  while (!d.stable_video() && !should_exit)
    std::this_thread::sleep_for(100ms);


  LOG(INFO) << "stable video - starting run";


  if (!should_exit) {
    try {
      bool searching = true;

      image_window win;
      // Grab and process frames until the main window is closed by the user.
      while (!win.is_closed() && !landed && !should_exit) {
        // Grab a frame
        cv::Mat temp;

        if (d.read_frame(&temp)) {
          // Turn OpenCV's Mat into something dlib can deal with.  Note that this just
          // wraps the Mat object, it doesn't copy anything.  So cimg is only valid as
          // long as temp is valid.  Also don't do anything to temp that would cause it
          // to reallocate the memory which stores the image as that will make cimg
          // contain dangling pointers.  This basically means you shouldn't modify temp
          // while using cimg.

          cv_image<bgr_pixel> cimg(temp);

          // Detect faces
          std::vector<rectangle> faces;

          if (search_mode)
            faces = detector(cimg);

          // Find the pose of each face.
          std::vector<full_object_detection> shapes;

          //for (unsigned long i = 0; i < faces.size(); ++i)
          //  shapes.push_back(pose_model(cimg, faces[i]));

          // This call asks the DNN to convert each face image in faces into a 128D vector.
          // In this 128D vector space, images from the same person will be close to each other
          // but vectors from different people will be far apart.  So we can use these vectors to
          // identify if a pair of images are from the same person or from different people.


          std::vector<matrix<rgb_pixel>> faces2;
          for (auto face : faces) {
            auto shape = sp(cimg, face);
            matrix<rgb_pixel> face_chip;
            extract_image_chip(cimg, get_face_chip_details(shape, 150, 0.25), face_chip);
            faces2.push_back(move(face_chip));
          }

          std::vector<matrix<float, 0, 1>> face_descriptors = recognition_net.get_face_descriptors(faces2);

          // add stick control
          // search mode
          // track face - try to get midpoint of shape to center of image
          // try to occupy face at 100 pixels - 60cm??

          if (!landing && search_mode) {
            if (faces.size()) {
              if (searching) {
                LOG(INFO) << "face found - stopped searching";
                searching = false;
              }
              d.stick().hover();
            } else {
              if (!searching) {
                LOG(INFO) << "face lost - staring searching";
                searching = true;
              }
              d.stick().set_yaw(100);
            }
          }


          // Display it all on the screen
          win.clear_overlay();
          win.set_image(cimg);

          // put some boxes on the faces so we can see that the detector is finding
          // them.
          win.add_overlay(faces);

          // this gives
          win.add_overlay(render_face_detections(shapes));
        } else {
          std::this_thread::sleep_for(10ms);
          continue;
        }
      }
    }
    catch (exception &e) {
      cout << e.what() << endl;
    }
  }

  should_exit = true;

  //t1.join();
  t2.join();


  t2.join();
  SDL_DestroyTexture(texture);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;

}


