#include <tellopp/drone1.h>

int main(int argc, char* argv[])
{
  {
    tellopp::sdk1_drone d;
    d.connect();
    while (!d.stable_video())
      std::this_thread::sleep_for(100ms);
    d.throw_take_off();
    std::this_thread::sleep_for(5s);
    d.stick().set_yaw(50);
    std::this_thread::sleep_for(10s);
    d.stick().set_yaw(0);
    std::this_thread::sleep_for(2s);
    d.flip(tellopp::flip_front);
    std::this_thread::sleep_for(4s);
    d.palm_land();
    std::this_thread::sleep_for(2s);
  }

  return 0;
}
