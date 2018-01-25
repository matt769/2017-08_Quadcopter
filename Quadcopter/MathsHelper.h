// mainly using info from here: http://www.microchip.com/forums/m817546.aspx
// and progmem stuff from here: http://forum.arduino.cc/index.php?topic=75126.0

// atan2Lookup
// atan2LookupWithInterpolation

#include <avr/pgmspace.h>

const PROGMEM  uint16_t pglkp[257] = {
  0,
  28, 57, 85, 114, 143, 171, 200, 229, 257, 286,
  314, 343, 372, 400, 429, 457, 486, 514, 543, 571,
  600, 628, 657, 685, 713, 742, 770, 798, 827, 855,
  883, 912, 940, 968, 996, 1024, 1052, 1080, 1108, 1136,
  1164, 1192, 1220, 1248, 1276, 1303, 1331, 1359, 1386, 1414,
  1442, 1469, 1497, 1524, 1552, 1579, 1606, 1633, 1661, 1688,
  1715, 1742, 1769, 1796, 1823, 1850, 1877, 1904, 1930, 1957,
  1984, 2010, 2037, 2063, 2090, 2116, 2142, 2169, 2195, 2221,
  2247, 2273, 2299, 2325, 2351, 2376, 2402, 2428, 2453, 2479,
  2504, 2530, 2555, 2580, 2606, 2631, 2656, 2681, 2706, 2731,
  2755, 2780, 2805, 2830, 2854, 2879, 2903, 2927, 2952, 2976,
  3000, 3024, 3048, 3072, 3096, 3120, 3143, 3167, 3191, 3214,
  3238, 3261, 3284, 3308, 3331, 3354, 3377, 3400, 3423, 3446,
  3468, 3491, 3514, 3536, 3558, 3581, 3603, 3625, 3648, 3670,
  3692, 3714, 3735, 3757, 3779, 3801, 3822, 3844, 3865, 3887,
  3908, 3929, 3950, 3971, 3992, 4013, 4034, 4055, 4076, 4096,
  4117, 4137, 4158, 4178, 4198, 4218, 4239, 4259, 4279, 4299,
  4318, 4338, 4358, 4378, 4397, 4417, 4436, 4455, 4475, 4494,
  4513, 4532, 4551, 4570, 4589, 4608, 4626, 4645, 4664, 4682,
  4700, 4719, 4737, 4755, 4774, 4792, 4810, 4828, 4846, 4863,
  4881, 4899, 4916, 4934, 4951, 4969, 4986, 5004, 5021, 5038,
  5055, 5072, 5089, 5106, 5123, 5139, 5156, 5173, 5189, 5206,
  5222, 5239, 5255, 5271, 5287, 5304, 5320, 5336, 5352, 5368,
  5383, 5399, 5415, 5430, 5446, 5462, 5477, 5492, 5508, 5523,
  5538, 5553, 5568, 5584, 5599, 5613, 5628, 5643, 5658, 5673,
  5687, 5702, 5716, 5731, 5745, 5760
};

const int noOfSegments = 256;  // note that this gets multiplied by 180 later go can't go much higher
const int scalingFactor = 128;
const float intToFloat = 1.0f / (float)scalingFactor;
const int yThreshold = 32767 / noOfSegments;

float atan2Lookup(int y, int x) {
  bool xneg = (x < 0);
  if (xneg) x = -x;
  bool yneg = (y < 0);
  if (yneg) y = -y;

  bool swap = (x < y);
  if (swap) {
    int tmp = x;
    x = y;
    y = tmp;
  }
  // x is now guaranteed to be larger than (or equal) y
  if (x == 0) return 0.0; // both values are zero
  while (y > yThreshold) {
    y = y >> 1;  // reduce y so there's no overflow in the next line
    x = x >> 1; // reduce x to maintain ratio
  }
  int idx = (y * noOfSegments) / x;
  int lkpDegValue = pgm_read_word_near(pglkp + idx);

  if (swap) {
    lkpDegValue = (90 * scalingFactor) - lkpDegValue;
  }
  if (xneg) {
    lkpDegValue = (180 * scalingFactor) - lkpDegValue;
  }
  if (yneg) {
    lkpDegValue = -lkpDegValue;
  }

  return (float)lkpDegValue * intToFloat;
}

// with interpolation between points
float atan2LookupWithInterpolation(int y, int x) {

  bool xneg = (x < 0);
  if (xneg) x = -x; // could use ~x for slight increase
  bool yneg = (y < 0);
  if (yneg) y = -y; // could use ~y for slight increase
  bool swap = (x < y);
  if (swap) {
    int tmp = x;
    x = y;
    y = tmp;
  }
  // x is now guaranteed to be larger than (or equal) y
  if (x == 0) return 0.0; // both values are zero
  while (y > yThreshold) {
    y = y >> 1;  // reduce y so there's no overflow in the next line
    x = x >> 1; // reduce x to maintain ratio
  }
  int idx = (y * noOfSegments) / x;
  float partial = ((y * noOfSegments) / (float)x ) - idx; // how much was truncated in integer division?
  int lkpDegValue = pgm_read_word_near(pglkp + idx);

  int adj = 0;
  if (idx != noOfSegments) { // for all except the final index
    int lkpDegNextValue = pgm_read_word_near(pglkp + idx + 1);
    adj = (int)((float)(lkpDegNextValue - lkpDegValue) * partial);
  }
  lkpDegValue += adj;

  // assign to the correct quadrant based on the inputs
  if (swap) {
    lkpDegValue = (90 * scalingFactor) - lkpDegValue;
  }
  if (xneg) {
    lkpDegValue = (180 * scalingFactor) - lkpDegValue;
  }
  if (yneg) {
    lkpDegValue = -lkpDegValue;
  }

  return (float)lkpDegValue * intToFloat;
}
