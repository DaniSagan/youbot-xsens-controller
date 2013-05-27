#include <dfv/vector3.h>

namespace dfv
{ 
    Vector3::Vector3():
        x(0), y(0), z(0)
    {

    }

    Vector3::Vector3(double x_, double y_, double z_):
        x(x_), y(y_), z(z_)
    {

    }

    Vector3::Vector3(const Quaternion& q):
        x(q.x), y(q.y), z(q.z)
    {

    }

    Vector3::~Vector3()
    {
        //dtor
    }
    
    Vector3& Vector3::operator=(const Vector3& v)
    {
        if(this != &v)
        {
            this->x = v.x;
            this->y = v.y;
            this->z = v.z;
        }
        
        return *this;
    }
    
    Vector3& Vector3::operator+=(const Vector3& v)
    {
        this->x += v.x;
        this->y += v.y;
        this->z += v.z;
        
        return *this;
    }
    
    Vector3& Vector3::operator-=(const Vector3& v)
    {
        this->x -= v.x;
        this->y -= v.y;
        this->z -= v.z;
        
        return *this;
    }
    
    Vector3& Vector3::operator*=(const double k)
    {
        this->x *= k;
        this->y *= k;
        this->z *= k;
        
        return *this;
    }
    
    const Vector3 Vector3::operator+(const Vector3& v) const
    {
        return Vector3(*this) += v;
    }
    
    const Vector3 Vector3::operator-(const Vector3& v) const
    {
        return Vector3(*this) -= v;
    }
    
    const Vector3 operator*(double k, Vector3& v)
    {
        return Vector3(v) *= k;
    }
    
    const Vector3 Vector3::operator*(double k) const
    {
        return Vector3(*this) *= k;
    }
    
    double Vector3::operator*(const Vector3& v) const
    {
        return this->x * v.x + this->y * v.y + this->z * v.z;
    }
    
    const Vector3 Vector3::operator^(const Vector3& v) const
    {
        return Vector3(this->y*v.z - this->z*v.y,
                       this->z*v.x - this->x*v.z,
                       this->x*v.y - this->y*v.x);
    }
    
    bool Vector3::operator==(const Vector3& v) const
    {
        return (this->x == v.x) && (this->y == v.y) && (this->z == v.z);
    }
    
    bool Vector3::operator!=(const Vector3& v) const
    {
        return !(*this == v);
    }

    double Vector3::GetMagnitude() const
    {
        return sqrt(pow(this->x, 2.0) + pow(this->y, 2.0) + pow(this->z, 2.0));
    }

    void Vector3::Normalize()
    {
        double mag = this->GetMagnitude();
        if(mag > 0)
        {
            this->x /= mag;
            this->y /= mag;
            this->z /= mag;
        }
        else
        {
            this->x = 0.0;
            this->y = 0.0;
            this->z = 0.0;
        }

    }

    const Vector3 Vector3::GetNormalized() const
    {
        double mag = this->GetMagnitude();

        if(mag > 0)
        {
            Vector3 result;
            result.x = this->x / mag;
            result.y = this->y / mag;
            result.z = this->z / mag;
            return result;
        }
        else
        {
            return Vector3(0.0, 0.0, 0.0);
        }
    }

    const Vector3 Vector3::GetScalated(double k) const
    {
        return Vector3(this->x*k, this->y*k, this->z*k);
    }
    
    Vector3& Vector3::Rotate(const Quaternion q)
    {
        *this = this->GetRotated(q);
        return *this;
    }
    
    const Vector3 Vector3::GetRotated(const Quaternion& q) const
    {
        Quaternion v(*this);
        return Vector3(q*v*(q.GetConjugate()));
    }
    
    std::string Vector3::ToString() const
    {
        std::stringstream ss;
        ss << "[" << this->x << ", " << this->y << ", " << this->z << "]";
        return ss.str();
    }
    
    Vector3 Vector3::i = Vector3(1, 0, 0);
    Vector3 Vector3::j = Vector3(0, 1, 0);
    Vector3 Vector3::k = Vector3(0, 0, 1);
    
    std::ostream& operator<<(std::ostream& os, const Vector3& v)
    {
        return os << v.ToString();
    }
}
