#include <dlib/opencv.h>
#include <opencv2/highgui/highgui.hpp>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>

#include <glog/logging.h>
#include <tellopp/drone1.h>
#include <dlib_utils/face.h>
#include <gainput/gainput.h>

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
  PITCH_FWD,
  PITCH_BACK,
  ROLL_LEFT,
  ROLL_RIGHT,
  YAW_LEFT,
  YAW_RIGHT,
  THROTTLE_UP,
  THROTTLE_DOWN,
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
  static int attributeListDbl[] = {      GLX_RGBA,      GLX_DOUBLEBUFFER, /*In case single buffering is not supported*/      GLX_RED_SIZE,   1,      GLX_GREEN_SIZE, 1,      GLX_BLUE_SIZE,  1,
                                         None };

  Display* xDisplay = XOpenDisplay(0);
  if (xDisplay == 0)
  {
    std::cerr << "Cannot connect to X server." << std::endl;
    return -1;
  }

  Window root = DefaultRootWindow(xDisplay);

  XVisualInfo* vi = glXChooseVisual(xDisplay, DefaultScreen(xDisplay), attributeListDbl);
  assert(vi);

  GLXContext context = glXCreateContext(xDisplay, vi, 0, GL_TRUE);

  Colormap cmap = XCreateColormap(xDisplay, root,                   vi->visual, AllocNone);

  XSetWindowAttributes swa;
  swa.colormap = cmap;
  swa.event_mask = ExposureMask
                   | KeyPressMask | KeyReleaseMask
                   | PointerMotionMask | ButtonPressMask | ButtonReleaseMask;

  Window xWindow = XCreateWindow(
      xDisplay, root,
      0, 0, width, height, 0,
      CopyFromParent, InputOutput,
      CopyFromParent, CWEventMask,
      &swa
  );

  glXMakeCurrent(xDisplay, xWindow, context);

  XSetWindowAttributes xattr;
  xattr.override_redirect = False;
  XChangeWindowAttributes(xDisplay, xWindow, CWOverrideRedirect, &xattr);

  XMapWindow(xDisplay, xWindow);
  XStoreName(xDisplay, xWindow, windowName);



  // Setup Gainput
  gainput::InputManager manager;
  const gainput::DeviceId mouseId = manager.CreateDevice<gainput::InputDeviceMouse>();
  const gainput::DeviceId keyboardId = manager.CreateDevice<gainput::InputDeviceKeyboard>();
  const gainput::DeviceId padId = manager.CreateDevice<gainput::InputDevicePad>();

  gainput::InputMap map(manager);
  map.MapBool(PITCH_FWD, keyboardId, gainput::KeyW);
  map.MapBool(PITCH_BACK, keyboardId, gainput::KeyS);
  map.MapBool(ROLL_LEFT, keyboardId, gainput::KeyA);
  map.MapBool(ROLL_RIGHT, keyboardId, gainput::KeyD);

  map.MapBool(YAW_LEFT, keyboardId, gainput::KeyLeft);
  map.MapBool(YAW_RIGHT, keyboardId, gainput::KeyRight);
  map.MapBool(THROTTLE_UP, keyboardId, gainput::KeyUp);
  map.MapBool(THROTTLE_DOWN, keyboardId, gainput::KeyDown);

  map.MapBool(TAKE_OFF, keyboardId, gainput::KeyT);
  map.MapBool(THROW_TAKE_OFF, keyboardId, gainput::KeyY);

  map.MapBool(LAND, keyboardId, gainput::KeyEscape);
  map.MapBool(PALM_LAND, keyboardId, gainput::KeyL);

  map.MapBool(FLIP, keyboardId, gainput::KeyF1);

  //map.MapBool(YAW_LEFT, mouseId, gainput::MouseButtonLeft);
  //map.MapBool(YAW_RIGHT, mouseId, gainput::MouseButtonRight);

  //map.MapFloat(MouseX, mouseId, gainput::MouseAxisX);
  //map.MapFloat(MouseY, mouseId, gainput::MouseAxisY);

  manager.SetDisplaySize(width, height);

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


  // flight controll thread
  /*std::thread t1([&](){
    std::this_thread::sleep_for(10s);

    LOG(INFO) << "throw_take_off";
    //d.throw_take_off();
    //wait for liftoff
    std::this_thread::sleep_for(30s);
    landing = true;
    d.stick().hover();
    LOG(INFO) << "landing";
    d.palm_land();
    std::this_thread::sleep_for(2s);
    LOG(INFO) << "landed";
    landed=true;
  });
  */


  // keyboard controll thread

  std::thread t2([&](){
    int64_t hover_timeout = 0;
    while(!should_exit)
    {
      XEvent event;
      while (XPending(xDisplay))
      {
        XNextEvent(xDisplay, &event);
        manager.HandleEvent(event);
      }

      manager.Update();

      tellopp::sdk1_drone::stick_state stick;

      gainput::DeviceButtonSpec anyButton[32];
      const size_t anyCount = manager.GetAnyButtonDown(anyButton, 32);
      for (size_t i = 0; i < anyCount; ++i) {
        // Filter the returned buttons as needed.
        const gainput::InputDevice *device = manager.GetDevice(anyButton[i].deviceId);
        if (device->GetButtonType(anyButton[i].buttonId) == gainput::BT_BOOL) {
          auto mapped_id = map.GetUserButtonId(anyButton[i].deviceId, anyButton[i].buttonId);
          if (mapped_id == gainput::InvalidDeviceButtonId)
            continue;
          switch (mapped_id) {
            case PITCH_FWD:
              stick.set_pitch(100);
              break;
            case PITCH_BACK:
              stick.set_pitch(-40);
              break;
            case ROLL_LEFT:
              stick.set_roll(-40);
              break;
            case ROLL_RIGHT:
              stick.set_roll(40);
              break;
            case YAW_LEFT:
              stick.set_yaw(100);
              break;
            case YAW_RIGHT:
              stick.set_yaw(-100);
              break;
            case THROTTLE_UP:
              stick.set_throttle(20);
              break;
            case THROTTLE_DOWN:
              stick.set_throttle(-20);
              break;

            case TAKE_OFF:
              d.take_off();
            case THROW_TAKE_OFF:
              d.throw_take_off();
              break;
            case LAND:
              d.land();
              break;
            case PALM_LAND:
              d.palm_land();
              break;
            case FLIP:
              d.flip(tellopp::flip_forward_left);
              break;
          }
        }
      }

      d.stick() = stick;

      /*
       * if (map.GetFloatDelta(MouseX) != 0.0f || map.GetFloatDelta(MouseY) != 0.0f)
      {
        //LOG(INFO) << "Mouse: " << map.GetFloat(MouseX) << ", " << map.GetFloat(MouseY);
      }
       */

      std::this_thread::sleep_for(10ms);

    }
  });

  d.connect();
  while (!d.stable_video())
    std::this_thread::sleep_for(100ms);

  LOG(INFO) << "stable video - starting run";


  try
  {
    bool searching = true;

    image_window win;
    // Grab and process frames until the main window is closed by the user.
    while(!win.is_closed() && !landed)
    {
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
        for (auto face : faces)
        {
          auto shape = sp(cimg, face);
          matrix<rgb_pixel> face_chip;
          extract_image_chip(cimg, get_face_chip_details(shape,150,0.25), face_chip);
          faces2.push_back(move(face_chip));
        }

        std::vector<matrix<float,0,1>> face_descriptors = recognition_net.get_face_descriptors(faces2);

        // add stick control
        // search mode
        // track face - try to get midpoint of shape to center of image
        // try to occupy face at 100 pixels - 60cm??

        if (!landing && search_mode){
          if (faces.size()) {
            if (searching){
              LOG(INFO) << "face found - stopped searching";
              searching = false;
            }
            d.stick().hover();
          } else {
            if (!searching){
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
  catch(exception& e)
  {
    cout << e.what() << endl;
  }

  should_exit = true;

  //t1.join();
  t2.join();

  XDestroyWindow(xDisplay, xWindow);
  XCloseDisplay(xDisplay);

}


