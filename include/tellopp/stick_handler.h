#include <map>
#include <thread>
#include <SDL2/SDL.h>
#include "stick_state.h"
#include <functional>
#pragma once

class stick_handler{
public:
  enum logical_button_t {
    TAKE_OFF,
    THROW_TAKE_OFF,
    LAND,
    PALM_LAND,
    FRONT_FLIP
  };
  enum button_event_t {
    BUTTON_DOWN,
    BUTTON_UP
  };

  //std::function<void (logical_button_t, button_event_t)>;

  stick_handler();
  ~stick_handler();
  void close();

  void set_button_handler(std::function<void (logical_button_t, button_event_t)> h) { _button_handler = h; }

  void handle_event(const SDL_Event& event);

  stick_state get_state() { return _stick_state; }

  std::map<int, int> get_raw_state() const  { return raw_mapping; }

private:

  SDL_Joystick* _joystick = nullptr;

  std::map<int, int> raw_mapping;

  stick_state _stick_state;
  std::function<void (logical_button_t, button_event_t)> _button_handler;

  //bool _exit=false;
};

