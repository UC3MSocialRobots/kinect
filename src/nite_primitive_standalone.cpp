#include "nite_primitive.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "nite_primitive_standalone");
  NitePrimitiveClass primitive;
  primitive.init_nite();
  primitive.run();
  return 0;
}

