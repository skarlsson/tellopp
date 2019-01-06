#pragma once

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


  inline int get_throttle() const {
    return ly * 100.0;
  }

  /**
         * set_pitch controls the forward and backward tilt of the drone.
         * Pass in an int from -100 - 100. (positive value will make the drone move forward)
         */
  inline void set_pitch(int val) {
    ry = (val / 100.0);
  }

  inline int get_pitch() const {
    return ry * 100.0;
  }


  /**
   *  set_roll controls the the side to side tilt of the drone.
   * Pass in an int from -100 - 100. (positive value will make the drone move to the right)
   */
  inline void set_roll(int val) {
    rx = (val / 100.0);
  }

  inline int get_roll() const {
    return rx * 100.0;
  }

  /**
         * set_yaw controls the left and right rotation of the drone.
         * Pass in an int from -100 - 100. (positive value will make the drone turn to the right)
         */
  inline void set_yaw(int val) {
    lx = (val / -100.0);
  }

  inline int get_yaw() const {
    return lx * 100.0;
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
