#ifndef WHEEL_HPP
#define WHEEL_HPP

#include <string>

class Wheel
{
public:
  std::string name = "";
  int enc_counts_per_rev = 0;
  double pos = 0.0;
  double vel = 0.0;
  double cmd = 0.0;

  void setup(const std::string& wheel_name, int counts_per_rev)
  {
    name = wheel_name;
    enc_counts_per_rev = counts_per_rev;
  }
};

#endif
