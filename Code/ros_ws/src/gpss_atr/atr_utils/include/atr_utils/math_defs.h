#ifndef ATR_MATH_DEFS_H
#define ATR_MATH_DEFS_H

#include <vector>
#include <string>
#include <math.h>

namespace atr
{
#define DEG2RAD(DEG) ((DEG) * ((M_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * ((180.0) / (M_PI)))
namespace math
{
typedef double real;

typedef std::vector<double> VDouble;
typedef std::vector<int> VInt;
typedef std::vector<std::string> VString;
typedef std::vector<bool> VBool;

}  // namespace math
}  // namespace atr

#endif  // ATR_MATH_DEFS_H