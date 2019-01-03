#include <tellopp/tello.h>

int main(int argc, char* argv[])
{
  {
    tellopp::sdk2_drone d;
    d.connect();
    std::this_thread::sleep_for(2s);
    d.send_command("battery?");
    std::this_thread::sleep_for(1s);
    d.takeOff();
    std::this_thread::sleep_for(1s);
    d.flip(tellopp::sdk2_drone::flip_front);
    std::this_thread::sleep_for(1s);
    d.send_command("stop");
    std::this_thread::sleep_for(1s);
    d.send_command("down 20");
    std::this_thread::sleep_for(3s);
    d.land();
    std::this_thread::sleep_for(2s);
  }

  return 0;
}
