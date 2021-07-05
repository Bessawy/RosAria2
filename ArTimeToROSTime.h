#ifndef ARTIMETOROSTIMESTAMP_H
#define ARTIMETOROSTIMESTAMP_H

#include "rclcpp/rclcpp.hpp"

#ifdef ADEPT_PKG
#include "ariaUtil.h"
#else
#include "Aria/ariaUtil.h"
#endif

double convertArTimeToROS(const ArTime& t, rclcpp::Node* node)
{
  // ARIA/ARNL times are in reference to an arbitrary starting time, not OS
  // clock, so find the time elapsed between now and t
  // to adjust the time stamp in ROS time vs. now accordingly.
  ArTime arianow;
  const double dtsec = (double) t.mSecSince(arianow) * 1e6;
  //printf("was %f seconds ago\n", dtsec);
  return node->now().nanoseconds() - dtsec;
}

#endif

