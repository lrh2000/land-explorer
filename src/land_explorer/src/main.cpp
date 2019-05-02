#include <ros/init.h>
#include "land_explorer/node.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "land_explorer");

  LandExplorer node;
  return node.exec();
}
