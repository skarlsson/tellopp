#include <tellopp/stick_handler.h>
#include <chrono>
#include <glog/logging.h>

using namespace std::chrono_literals;

enum logical_axis_t {
  PITCH,
  ROLL,
  YAW,
  THROTTLE
};


struct joysting_axis_mapping_item {
  int device;
  int device_axis;
  logical_axis_t logical_axis;
};

struct joysting_button_mapping_item {
  int device;
  int device_button;
  stick_handler::logical_button_t logical_button;
};

struct keyboard_axis_mapping_item {
  int device_key;
  logical_axis_t logical_axis;
  int value;
};

joysting_axis_mapping_item joystick_axis_mapping[] = {
  { 0, 0, ROLL },
  { 0, 1, PITCH },
  { 0, 3, YAW }
//  { 0, 2, THROTTLE }
};

joysting_button_mapping_item joystick_button_events[] = {
  { 0, 5, stick_handler::TAKE_OFF },
  { 0, 6, stick_handler::THROW_TAKE_OFF },
  { 0, 7, stick_handler::LAND },
  { 0, 8, stick_handler::PALM_LAND },
  { 0, 1, stick_handler::FRONT_FLIP }
};

keyboard_axis_mapping_item keyboard_axis_mapping[] = {
  {SDLK_w, PITCH, 50},
  {SDLK_s, PITCH, -50},
  {SDLK_a, ROLL, 50},
  {SDLK_d, ROLL, -50},
  {SDLK_LEFT, YAW, -100},
  {SDLK_RIGHT, YAW, 100},
  {SDLK_UP, THROTTLE, 50},
  {SDLK_DOWN, THROTTLE, -50}
};

stick_handler::stick_handler(){
//Check if there's any joysticks
  if( SDL_NumJoysticks() < 1 ) {
    LOG(ERROR) << "cannot find joystick";
  }

  //Open the joystick
  _joystick = SDL_JoystickOpen( 0 );

  //If there's a problem opening the joystick
  if( _joystick == NULL ) {
    LOG(ERROR) << "cannot open joystick 0";
  }
}

stick_handler::~stick_handler(){
  close();

  if (_joystick)
    SDL_JoystickClose(_joystick);

  _joystick = nullptr;
}

void stick_handler::close(){
//  _exit = true;
}

void stick_handler::handle_event(const SDL_Event& event){
  //If a axis was changed
  switch (event.type) {
    case SDL_JOYAXISMOTION: {
      //If joystick 0 has moved
      if (event.jaxis.which == 0) {
        raw_mapping[event.jaxis.axis] = event.jaxis.value;

        for (int i = 0; i != sizeof(joystick_axis_mapping) / sizeof(joystick_axis_mapping[0]); ++i) {
          if ((event.jaxis.which == joystick_axis_mapping[i].device) &&
              (event.jaxis.axis == joystick_axis_mapping[i].device_axis)) {
            switch (joystick_axis_mapping[i].logical_axis) {
              case ROLL:
                _stick_state.set_roll((event.jaxis.value * 100) / 32678);
                break;
              case PITCH:
                _stick_state.set_pitch((event.jaxis.value * 100) / 32678);
                break;
              case YAW:
                _stick_state.set_yaw((event.jaxis.value * 100) / 32678);
                break;
              case THROTTLE:
                _stick_state.set_throttle((event.jaxis.value * 100) / 32678);
                break;
            }
          }
        }
      }
    }
    break;

    case SDL_JOYBUTTONDOWN:  {
      LOG(INFO) << "SDL_JOYBUTTONDOWN device_button: " << (int) event.jbutton.button;
      for (int i = 0; i != sizeof(joystick_button_events) / sizeof(joystick_button_events[0]); ++i) {
        if (event.jbutton.button == joystick_button_events[i].device_button) {
          if (_button_handler)
            _button_handler(joystick_button_events[i].logical_button, BUTTON_DOWN);
            break;
          }
        }
      }
      break;

    case SDL_KEYDOWN:{
      LOG(INFO) << "KEYDOWN";
      for (int i = 0; i != sizeof(keyboard_axis_mapping) / sizeof(keyboard_axis_mapping[0]); ++i) {
        if (event.key.keysym.sym == keyboard_axis_mapping[i].device_key){
          switch (keyboard_axis_mapping[i].logical_axis){
            case ROLL:
              _stick_state.set_roll(keyboard_axis_mapping[i].value); break;
              break;
            case PITCH:
              _stick_state.set_pitch(keyboard_axis_mapping[i].value); break;
              break;
            case YAW:
              _stick_state.set_yaw(keyboard_axis_mapping[i].value); break;
              break;
            case THROTTLE:
              _stick_state.set_throttle(keyboard_axis_mapping[i].value); break;
              break;
          }
        }
      }

    }
    break;

    case SDL_KEYUP:{
      for (int i = 0; i != sizeof(keyboard_axis_mapping) / sizeof(keyboard_axis_mapping[0]); ++i) {
        if (event.key.keysym.sym == keyboard_axis_mapping[i].device_key){
          switch (keyboard_axis_mapping[i].logical_axis){
            case ROLL:
              if (_stick_state.get_roll() == keyboard_axis_mapping[i].value)
                _stick_state.set_roll(0);
              break;
            case PITCH:
              if (_stick_state.get_pitch() == keyboard_axis_mapping[i].value)
              _stick_state.set_pitch(0);
              break;
            case YAW:
              if (_stick_state.get_yaw() == keyboard_axis_mapping[i].value)
                _stick_state.set_yaw(0);
              break;
            case THROTTLE:
              if (_stick_state.get_throttle() == keyboard_axis_mapping[i].value)
                _stick_state.set_throttle(0);;
              break;
          }
        }
      }
    }

      break;
  }
}





