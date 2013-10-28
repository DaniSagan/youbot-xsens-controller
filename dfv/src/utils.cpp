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
    
    double NormalizeAngle(double angle)
    {
        while(angle < -dfv::pi)
        {
            angle += 2*dfv::pi;
        }
        while(angle >= dfv::pi)
        {
            angle -= 2*dfv::pi;
        }
        return angle;
    }
    
    std::vector<std::string> StrTokenize(const std::string& str, char delimiter)
    {
        std::stringstream ss(str);
        std::string s;
        std::vector<std::string> res;
        
        while (getline(ss, s, delimiter)) 
        {
            res.push_back(s);
        }
        return res;
    }

}
