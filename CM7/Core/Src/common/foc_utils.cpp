#include "foc_utils.hpp"
#include <stdint.h>
#include <math.h>

// function approximating the sine calculation by using fixed size array
// uses a 65 element lookup table and interpolation
// thanks to @dekutree for his work on optimizing this
__attribute__((weak)) float _sin(float a)
{
  // 16bit integer array for sine lookup. interpolation is used for better precision
  // 16 bit precision on sine value, 8 bit fractional value for interpolation, 6bit LUT size
  // resulting precision compared to stdlib sine is 0.00006480 (RMS difference in range -PI,PI for 3217 steps)
  static uint16_t sine_array[65] = {0,804,1608,2411,3212,4011,4808,5602,6393,7180,7962,8740,9512,10279,11039,11793,12540,13279,14010,14733,15447,16151,16846,17531,18205,18868,19520,20160,20788,21403,22006,22595,23170,23732,24279,24812,25330,25833,26320,26791,27246,27684,28106,28511,28899,29269,29622,29957,30274,30572,30853,31114,31357,31581,31786,31972,32138,32286,32413,32522,32610,32679,32729,32758,32768};
  unsigned int i = (unsigned int)(a * (64*4*256.0 /_2PI));
  int t1, t2, frac = i & 0xff;
  i = (i >> 8) & 0xff;
  if (i < 64) 
  {
    t1 = sine_array[i]; t2 = sine_array[i+1];
  }
  else if(i < 128) 
  {
    t1 = sine_array[128 - i]; t2 = sine_array[127 - i];
  }
  else if(i < 192) 
  {
    t1 = -sine_array[-128 + i]; t2 = -sine_array[-127 + i];
  }
  else 
  {
    t1 = -sine_array[256 - i]; t2 = -sine_array[255 - i];
  }
  return (1.0f/32768.0f) * (t1 + (((t2 - t1) * frac) >> 8));
}

// function approximating cosine calculation by using fixed size array
// ~55us (float array)
// ~56us (int array)
// precision +-0.005
// it has to receive an angle in between 0 and 2PI
__attribute__((weak)) float _cos(float a){
  // Ensure input is in [0, 2π) range to prevent boundary issues
  // Normalize if needed (defensive check for floating point errors)
  if(a < 0.0f || a >= _2PI)
  {
      // Use fmodf directly (safe_fmodf is defined later, but fmodf is standard)
      float normalized = fmodf(a, _2PI);
      a = normalized >= 0.0f ? normalized : (normalized + _2PI);
  }
  
  // CRITICAL FIX: cos(a) = sin(a + π/2), but we need to ensure proper wrapping
  // The _sin lookup table expects [0, 2π), so we must wrap correctly
  // cos(0) = 1 = sin(π/2)
  // cos(π/2) = 0 = sin(π)
  // cos(π) = -1 = sin(3π/2)
  // cos(3π/2) = 0 = sin(2π) = sin(0)
  float a_sin = a + _PI_2;
  
  // Wrap to [0, 2π) range - this is critical for _sin to work correctly
  // Since a is in [0, 2π), a_sin will be in [π/2, 2π + π/2)
  // We need to wrap values >= 2π back to [0, 2π)
  if(a_sin >= _2PI)
  {
      a_sin = a_sin - _2PI;
  }
  // Defensive check for negative (shouldn't happen, but be safe)
  if(a_sin < 0.0f)
  {
      a_sin = a_sin + _2PI;
  }
  
  // Now a_sin is guaranteed to be in [0, 2π), so _sin should work correctly
  return _sin(a_sin);
}


__attribute__((weak)) void _sincos(float a, float* s, float* c){
  *s = _sin(a);
  *c = _cos(a);
}

float safe_fmodf(float x, float y) {
    if (y == 0.0f || isnan(x) || isnan(y)) {
        return NAN;
    }
    return x - y * floorf(x / y);
}

// normalizing radian angle to [0,2PI]
__attribute__((weak)) float _normalizeAngle(float angle)
{
  float a = safe_fmodf(angle, _2PI);
  return a >= 0 ? a : (a + _2PI);
}

// Electrical angle calculation
float _electricalAngle(float shaft_angle, int pole_pairs) 
{
  return (shaft_angle * pole_pairs);
}

// square root approximation function using
// https://reprap.org/forum/read.php?147,219210
// https://en.wikipedia.org/wiki/Fast_inverse_square_root
__attribute__((weak)) float _sqrtApprox(float number) {//low in fat
  // float x;
  // const float f = 1.5F; // better precision

  // x = number * 0.5F;
  float y = number;
  long i = * ( long * ) &y;
  i = 0x5f375a86 - ( i >> 1 );
  y = * ( float * ) &i;
  // y = y * ( f - ( x * y * y ) ); // better precision
  return number * y;
}
