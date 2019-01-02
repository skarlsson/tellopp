#include <tellopp/tello.h>

int main(int argc, char* argv[])
{
  {
    tellopp::drone d;
    d.connect();
    //std::this_thread::sleep_for(2s);
    d.takeOff();
    //std::this_thread::sleep_for(5s);
    //d.flip(tellopp::drone::flip_forward_left);
    std::this_thread::sleep_for(10s);
    d.land();
    std::this_thread::sleep_for(10s);
  }

  return 0;
}
