// Lunar orientation and gravity, matching the models inside Luminary099.
#ifndef MOON_H
#define MOON_H

#include "vmath.h"

#define MOON_MU 4.9027780e12     // m^3/s^2 (MUM in CONTROLLED_CONSTANTS)
#define MOON_RADIUS 1738000.0    // nominal sphere radius used for scenery, m

// Port of the AGC's MOONMX: rows of M are the moon-fixed axes in basic
// reference (inertial) coordinates. R_inertial = M^T R_moonfixed.
// t_ephem_s = AGC time + TEPHEM, in seconds.
M3 moon_matrix(double t_ephem_s);

// Moon angular velocity (inertial frame), rad/s.
V3 moon_omega(double t_ephem_s);

V3 moon_fixed_to_inertial(V3 rp, double t_ephem_s);
V3 moon_inertial_to_fixed(V3 r, double t_ephem_s);

// Moon-fixed position from selenographic latitude/longitude (rad) and radius.
V3 moon_latlon_to_fixed(double lat, double lon, double radius);
void moon_fixed_to_latlon(V3 rp, double *lat, double *lon, double *radius);

#endif
