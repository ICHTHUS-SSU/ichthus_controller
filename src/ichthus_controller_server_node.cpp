#include "ichthus_controller/ichthus_controller.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ichthus_controller");
  ichthus_controller::IchthusController controller;
  controller.doLoop();
  return 0;
}
