#include "ros_uav_command/testPositioning.h"

geographic_msgs::GeoPoint home_;

void latlon_to_XY(double lat, double lon, double *X, double *Y) {
  // Convert lat/lon to NE
  static constexpr double LOCATION_SCALING_FACTOR = 111318.84502145034;  // deg to m
  static constexpr double DEG_TO_RAD = M_PI/180.0;

  *Y = (lat-home_.latitude) * LOCATION_SCALING_FACTOR;
  *X = (lon-home_.longitude) * LOCATION_SCALING_FACTOR * std::max(cos(home_.latitude*DEG_TO_RAD),0.01);  // not sure about this one, let's see how it actually goes...

  return;

}

void XY_to_latlon(double X, double Y, double *lat, double *lon) {
    // Convert NE to lat/lon
    static constexpr double LOCATION_SCALING_FACTOR_INV = 0.000008983204953368922;  // m to deg
    static constexpr double DEG_TO_RAD = M_PI/180.0;

    *lat = home_.latitude + Y*LOCATION_SCALING_FACTOR_INV;
    *lon = home_.longitude + X*LOCATION_SCALING_FACTOR_INV/std::max(cos(home_.latitude*DEG_TO_RAD),0.01);

    return;

}
