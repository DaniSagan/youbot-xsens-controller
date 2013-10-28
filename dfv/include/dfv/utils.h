#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <string>
#include <sstream>

namespace dfv
{
    const long double pi = 3.141592653589793238462643383279502884197L;
    
    long double DegToRad(long double deg);
    long double RadToDeg(long double rad);
    
    double NormalizeAngle(double angle);
    
    std::vector<std::string> StrTokenize(const std::string& str, char delimiter);
}

#endif
