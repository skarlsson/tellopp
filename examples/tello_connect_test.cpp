#include <tellopp/drone1.h>

int main(int argc, char* argv[])
{
  {
    tellopp::sdk1_drone d;
    d.connect();
    std::this_thread::sleep_for(10s);
    //d.send_command("battery?");
    std::this_thread::sleep_for(1s);
    d.throw_take_off();
    std::this_thread::sleep_for(10s);
    //d.send_command("up 100");
    std::this_thread::sleep_for(2s);
    //d.send_command("cw 90");
    std::this_thread::sleep_for(2s);
    d.flip(tellopp::flip_front);
    std::this_thread::sleep_for(1s);
    //d.send_command("stop");
    std::this_thread::sleep_for(1s);
    //d.send_command("down 20");
    std::this_thread::sleep_for(3s);
    d.land();
    std::this_thread::sleep_for(2s);
  }

  return 0;
}
