#ifdef _MSC_VER
#include <cmath>

inline double round(double val)
{    
	return std::floor(val + 0.5);
}

namespace std
{
inline long long atoll(const char* str)
{
	return (long long) _atoi64(str);
}
}

#endif