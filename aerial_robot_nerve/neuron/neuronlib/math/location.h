#ifndef __LOCATION_H_
#define __LOCATION_H_

#pragma once

#include <inttypes.h>

/*#include <AP_Common/AP_Common.h>*/
/*#include <AP_HAL/AP_HAL.h> */

#include "vector2.h"
#include "vector3.h"



////////////////////////////////////////////////////////////////////////////////
/// @name	Types
///
/// Data structures and types used throughout the libraries and applications. 0 = default
/// bit 0: Altitude is stored               0: Absolute,	1: Relative
/// bit 1: Chnage Alt between WP            0: Gradually,	1: ASAP
/// bit 2: Direction of loiter command      0: Clockwise	1: Counter-Clockwise
/// bit 3: Req.to hit WP.alt to continue    0: No,          1: Yes
/// bit 4: Relative to Home					0: No,          1: Yes
/// bit 5:
/// bit 6:
/// bit 7: Move to next Command             0: YES,         1: Loiter until commanded

//@{

#define PACKED __attribute__((__packed__))

namespace ap
{
struct PACKED Location_Option_Flags {
  uint8_t relative_alt : 1;           // 1 if altitude is relateive to home
  uint8_t unused1      : 1;           // unused flag (defined so that loiter_ccw uses the correct bit)
  uint8_t loiter_ccw   : 1;           // 0 if clockwise, 1 if counter clockwise
  uint8_t terrain_alt  : 1;           // this altitude is above terrain
};

struct PACKED Location {
  union PACKED{
    Location_Option_Flags flags;                    ///< options bitmask (1<<0 = relative altitude)
    uint8_t options;                                /// allows writing all flags to eeprom as one byte
  };
  // by making alt 24 bit we can make p1 in a command 16 bit,
  // allowing an accurate angle in centi-degrees. This keeps the
  // storage cost per mission item at 15 bytes, and allows mission
  // altitudes of up to +/- 83km
  int32_t alt:24;                                     ///< param 2 - Altitude in centimeters (meters * 100)
  int32_t lat;                                        ///< param 3 - Lattitude * 10**7
  int32_t lng;                                        ///< param 4 - Longitude * 10**7
};



/*
 * LOCATION
 */
// longitude_scale - returns the scaler to compensate for shrinking longitude as you move north or south from the equator
// Note: this does not include the scaling to convert longitude/latitude points to meters or centimeters
float        longitude_scale(const struct Location &loc);

// return distance in meters between two locations
float        get_distance(const struct Location &loc1, const struct Location &loc2);

// return distance in centimeters between two locations
uint32_t     get_distance_cm(const struct Location &loc1, const struct Location &loc2);

// return bearing in centi-degrees between two locations
int32_t      get_bearing_cd(const struct Location &loc1, const struct Location &loc2);

// see if location is past a line perpendicular to
// the line between point1 and point2. If point1 is
// our previous waypoint and point2 is our target waypoint
// then this function returns true if we have flown past
// the target waypoint
bool        location_passed_point(const struct Location & location,
                                  const struct Location & point1,
                                  const struct Location & point2);

/*
  return the proportion we are along the path from point1 to
  point2. This will be less than >1 if we have passed point2
*/
float       location_path_proportion(const struct Location &location,
                                     const struct Location &point1,
                                     const struct Location &point2);

//  extrapolate latitude/longitude given bearing and distance
void        location_update(struct Location &loc, float bearing, float distance);

// extrapolate latitude/longitude given distances north and east
void        location_offset(struct Location &loc, float ofs_north, float ofs_east);

/*
  return the distance in meters in North/East plane as a N/E vector
  from loc1 to loc2
*/
Vector2f    location_diff(const struct Location &loc1, const struct Location &loc2);

/*
 * check if lat and lng match. Ignore altitude and options
 */
bool        locations_are_same(const struct Location &loc1, const struct Location &loc2);

/*
 * convert invalid waypoint with useful data. return true if location changed
 */
bool        location_sanitize(const struct Location &defaultLoc, struct Location &loc);

/*
  print a int32_t lat/long in decimal degrees
*/
/*void        print_latlon(AP_HAL::BetterStream *s, int32_t lat_or_lon); */

// Converts from WGS84 geodetic coordinates (lat, lon, height)
// into WGS84 Earth Centered, Earth Fixed (ECEF) coordinates
// (X, Y, Z)
void        wgsllh2ecef(const Vector3d &llh, Vector3d &ecef);

// Converts from WGS84 Earth Centered, Earth Fixed (ECEF)
// coordinates (X, Y, Z), into WHS84 geodetic
// coordinates (lat, lon, height)
void        wgsecef2llh(const Vector3d &ecef, Vector3d &llh);
}; // namespace ap
#endif
