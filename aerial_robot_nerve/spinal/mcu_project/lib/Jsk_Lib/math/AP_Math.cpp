#include "AP_Math.h"
#include <float.h>

namespace ap
{

// a varient of asin() that checks the input ranges and ensures a
// valid angle as output. If nan is given as input then zero is
// returned.
float safe_asin(float v)
{
  if (isnan(v)) {
    return 0.0f;
  }
  if (v >= 1.0f) {
    return M_PI/2;
  }
  if (v <= -1.0f) {
    return -M_PI/2;
  }
  return asinf(v);
}

// a varient of sqrt() that checks the input ranges and ensures a
// valid value as output. If a negative number is given then 0 is
// returned. The reasoning is that a negative number for sqrt() in our
// code is usually caused by small numerical rounding errors, so the
// real input should have been zero
float safe_sqrt(float v)
{
  float ret = sqrtf(v);
  if (isnan(ret)) {
    return 0;
  }
  return ret;
}

/* fast inv sqrt(added by bakui) */
float inv_sqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;

  int32_t i =  *reinterpret_cast<int32_t*>(&y);
  //long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *reinterpret_cast<float*>(&i);
  //y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}
}; // namespace ap
