#include <dfv/utils.h>

namespace dfv
{

    long double DegToRad(long double deg)
    {
        return deg * pi / 180.0L;
    }

    long double RadToDeg(long double rad)
    {
        return rad * 180.0L / pi;
    }

}
