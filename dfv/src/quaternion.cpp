#include "dfv/quaternion.h"

namespace dfv
{

    Quaternion::Quaternion(): w(0), x(0), y(0), z(0)
    {
        //ctor
    }

    Quaternion::Quaternion(double w_, double x_, double y_, double z_): w(w_), x(x_), y(y_), z(z_)
    {

    }

    Quaternion::Quaternion(const Vector3& v):
        w(0.0), x(v.x), y(v.y), z(v.z)
    {

    }

    Quaternion::~Quaternion()
    {
        //dtor
    }
    
    Quaternion& Quaternion::operator=(const Quaternion& q)
    {
        if(this != &q)
        {
            this->w = q.w;
            this->x = q.x;
            this->y = q.y;
            this->z = q.z;
        }
        
        return *this;
    }
    
    Quaternion& Quaternion::operator+=(const Quaternion& q)
    {
        this->w += q.w;
        this->x += q.x;
        this->y += q.y;
        this->z += q.z;
        
        return *this;
    }
    
    Quaternion& Quaternion::operator-=(const Quaternion& q)
    {
        this->w -= q.w;
        this->x -= q.x;
        this->y -= q.y;
        this->z -= q.z;
        
        return *this;
    }
    
    Quaternion& Quaternion::operator*=(const double k)
    {
        this->w *= k;
        this->x *= k;
        this->y *= k;
        this->z *= k;
        
        return *this;
    }
    
    const Quaternion Quaternion::operator+(const Quaternion& q) const
    {
        return Quaternion(*this) += q;
    }
    
    const Quaternion Quaternion::operator-(const Quaternion& q) const
    {
        return Quaternion(*this) -= q;
    }
    
    const Quaternion operator*(double k, Quaternion& q)
    {
        return Quaternion(q) *= k;
    }
    
    const Quaternion Quaternion::operator*(double k) const
    {
        return Quaternion(*this) *= k;
    }
    
    const Quaternion Quaternion::operator*(const Quaternion& q) const
    {
        return Quaternion(this->w*q.w - this->x*q.x - this->y*q.y - this->z*q.z,
                          this->x*q.w + this->w*q.x - this->z*q.y + this->y*q.z,
                          this->y*q.w + this->z*q.x + this->w*q.y - this->x*q.z,
                          this->z*q.w - this->y*q.x + this->x*q.y + this->w*q.z);
    }
    
    bool Quaternion::operator==(const Quaternion& q) const
    {
        return (this->w == q.w) && (this->x == q.x) && (this->y == q.y) && (this->z == q.z);
    }
    
    bool Quaternion::operator!=(const Quaternion& q) const
    {
        return !(*this == q);
    }

    std::string Quaternion::ToString() const
    {
        std::stringstream ss;
        ss << this->w << " + " << this->x << "*i + " << this->y << "*j + " << this->z << "*k";
        return ss.str();
    }

    double Quaternion::GetModulus() const
    {
        return sqrt(pow(this->w, 2) + pow(this->x, 2) + pow(this->y, 2) + pow(this->z, 2));
    }

    void Quaternion::Normalize()
    {
        double l = this->GetModulus();
        if(l == 0.f)
        {
            this->w = 0.f;
            this->x = 0.f;
            this->y = 0.f;
            this->z = 0.f;
        }
        else
        {
            this->w /= l;
            this->x /= l;
            this->y /= l;
            this->z /= l;
        }
    }

    const Quaternion Quaternion::GetConjugate() const
    {
        return Quaternion(this->w, -this->x, -this->y, -this->z);
    }

    const Quaternion Quaternion::GetRotationQuaternion(const double axisx_, 
                                                       const double axisy_, 
                                                       const double axisz_, 
                                                       const double angle_)
    {
        double l = sqrt(axisx_*axisx_ + axisy_*axisy_ + axisz_*axisz_);
        return Quaternion(cos(angle_ / 2.0),
                                axisx_/l * sin(angle_ / 2.0),
                                axisy_/l * sin(angle_ / 2.0),
                                axisz_/l * sin(angle_ / 2.0));
    }

    const Quaternion Quaternion::GetRotationQuaternion(const Vector3& axis_, 
                                                       const double angle_)
    {
        double l = axis_.GetMagnitude();
        return Quaternion(cos(angle_ / 2.0),
                          axis_.x/l * sin(angle_ / 2.0),
                          axis_.y/l * sin(angle_ / 2.0),
                          axis_.z/l * sin(angle_ / 2.0));
    }

    const Quaternion Quaternion::GetRotationQuaternion(const Vector3& v_before, 
                                                       const Vector3& v_after)
    {
        Vector3 vu = v_before.GetNormalized();
        Vector3 vvu = v_after.GetNormalized();
        if(vu == vvu)
        {
            return Quaternion(1.0, 0.0, 0.0, 0.0);
        }
        Vector3 r = (vu ^ vvu).GetNormalized();
        if(r == Vector3(0.0, 0.0, 0.0))
        {
            r = ((vu ^ Vector3::i) + (vu ^ Vector3::j)).GetNormalized();
            return Quaternion::GetRotationQuaternion(r, pi);
        }
        else
        {
            double sin_theta = (vu ^ vvu).GetMagnitude();
            double cos_theta = vu * vvu;
            double theta = atan2(sin_theta, cos_theta);
            return Quaternion::GetRotationQuaternion(r, theta);
        }

    }

    const Quaternion Quaternion::GetRotationQuaternion(const Vector3& v1_before,
                                                       const Vector3& v1_after,
                                                       const Vector3& v2_before,
                                                       const Vector3& v2_after)
    {
        Quaternion q1 = Quaternion::GetRotationQuaternion(v1_before, v1_after);
        Vector3 p2(v2_before);
        Vector3 pp2 = p2.GetRotated(q1);
        Vector3 vv2(pp2);
        Vector3 r2 = v1_after;

        if((vv2 ^ v2_after)*v1_after > 0)
        {
            double sin_theta = (vv2 ^ v2_after).GetMagnitude();
            double cos_theta = vv2*v2_after;
            double theta = atan2(sin_theta, cos_theta);
            Quaternion q2 = Quaternion::GetRotationQuaternion(r2, theta);
            return q2*q1;
        }
        else
        {
            double sin_theta = -((vv2 ^ v2_after).GetMagnitude());
            double cos_theta = vv2*v2_after;
            double theta = atan2(sin_theta, cos_theta);
            Quaternion q2 = Quaternion::GetRotationQuaternion(r2, theta);
            return q2*q1;
        }
    }

    void Quaternion::Decompose(double& angle_1,
                               double& angle_2,
                               double& angle_3,
                               const Vector3& v1,
                               const Vector3& v2,
                               const Vector3& v3) const
    {
        Quaternion q1 = Quaternion::GetRotationQuaternion(v1, angle_1);
        Quaternion q2 = Quaternion::GetRotationQuaternion(v2, angle_2);
        Quaternion q3 = Quaternion::GetRotationQuaternion(v3, angle_3);
        Quaternion qt = q3*q2*q1;

        const double err = 0.0001;
        const double delta_angle = 0.0001;
        const unsigned int tries = 10000;
        unsigned int k = 0;

        Quaternion qr = *this;

        double dist = (qr - qt).GetModulus();

        while((dist >= err) && (k < tries))
        {
            for(int c1 = -1; c1 <= 1; ++c1)
            {
                for(int c2 = -1; c2 <= 1; ++c2)
                {
                    for (int c3 = -1; c3 <= 1; ++c3)
                    {
                        double new_angle_1 = angle_1 + delta_angle*c1;
                        double new_angle_2 = angle_2 + delta_angle*c2;
                        double new_angle_3 = angle_3 + delta_angle*c3;

                        Quaternion new_q1 = Quaternion::GetRotationQuaternion(v1, new_angle_1);
                        Quaternion new_q2 = Quaternion::GetRotationQuaternion(v2, new_angle_2);
                        Quaternion new_q3 = Quaternion::GetRotationQuaternion(v3, new_angle_3);
                        Quaternion new_qt = new_q3*new_q2*new_q1;

                        double new_dist = (qr - new_qt).GetModulus();

                        if(new_dist < dist)
                        {
                            angle_1 = new_angle_1;
                            angle_2 = new_angle_2;
                            angle_3 = new_angle_3;
                            dist = new_dist;
                        }
                    }
                }
            }
            ++k;
        }
    }

    void Quaternion::GetAxisAndAngle(Vector3& vector, double& angle) const
    {
        Quaternion qr = *this;
        Vector3 e(qr);
        e.Normalize();
        double cc = this->w;
        double ss = ((qr - Quaternion(this->w, 0, 0, 0)) * Quaternion(e).GetConjugate()).w;
        double ang = 2*atan2(ss, cc);
        
        vector = e;
        angle = ang;
    }
    
    void Quaternion::GetRPY(double& roll, double& pitch, double& yaw)
    {
        tf::Quaternion tf_q(this->x, this->y, this->z, this->w);
        tf::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    }
    
    const Quaternion Quaternion::GetDifference(const Quaternion& q1, const Quaternion& q2)
    {
        return (q1.GetConjugate())*q2;
    }

    const Quaternion Quaternion::identity = Quaternion(1,0,0,0);
    const Quaternion Quaternion::i        = Quaternion(0,1,0,0);
    const Quaternion Quaternion::j        = Quaternion(0,0,1,0);
    const Quaternion Quaternion::k        = Quaternion(0,0,0,1);
    
    std::ostream& operator<<(std::ostream& os, const Quaternion& q)
    {
        return os << q.ToString();
    }

}
