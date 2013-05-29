#include <xsens_driver/xsens_sensor_subscriber.h>

namespace xsens
{
    SensorSubscriber::SensorSubscriber(unsigned int mt_index_):
        mt_index(mt_index_)
    {
        std::stringstream ss;
        ss << "/xsens/sensor" << this->mt_index << "/acc";
        this->acc_topic_name = ss.str();
        
        ss.str(std::string());
        ss << "/xsens/sensor" << this->mt_index << "/gyr";
        this->gyr_topic_name = ss.str();
        
        ss.str(std::string());
        ss << "/xsens/sensor" << this->mt_index << "/mag";
        this->mag_topic_name = ss.str();
        
        ss.str(std::string());
        ss << "/xsens/sensor" << this->mt_index << "/ori_quat";
        this->ori_quat_topic_name = ss.str();
        
        ss.str(std::string());
        ss << "/xsens/sensor" << this->mt_index << "/ori_matrix";
        this->ori_matrix_topic_name = ss.str();
        
        ss.str(std::string());
        ss << "/xsens/sensor" << this->mt_index << "/ori_euler";
        this->ori_euler_topic_name = ss.str();
        
        int param;
        this->node_handle.param<int>("/xsens/output_mode", param, 0);
        this->output_mode = param;
        this->node_handle.param<int>("/xsens/output_settings", param, 0);
        this->output_settings = param;
        
    }
    
    SensorSubscriber::~SensorSubscriber()
    {
        
    }
    
    bool SensorSubscriber::SubscribeToTopics()
    {
        if((this->output_mode & CMT_OUTPUTMODE_CALIB) != 0)
        {
            this->acc_subscriber = this->node_handle.subscribe(this->acc_topic_name, 
                                                               1,
                                                               &SensorSubscriber::AccSubCallback,
                                                               this);
            this->gyr_subscriber = this->node_handle.subscribe(this->gyr_topic_name, 
                                                               1,
                                                               &SensorSubscriber::GyrSubCallback,
                                                               this);
            this->mag_subscriber = this->node_handle.subscribe(this->mag_topic_name, 
                                                               1,
                                                               &SensorSubscriber::MagSubCallback,
                                                               this);                                                   
        }
        
        if((this->output_mode & CMT_OUTPUTMODE_ORIENT) != 0)
        {
            if((this->output_settings & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK) == CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION)
            {
                this->ori_quat_subscriber = this->node_handle.subscribe(this->ori_quat_topic_name, 
                                                                        1,
                                                                        &SensorSubscriber::OriQuatSubCallback,
                                                                        this);
            }
            
            if((this->output_settings & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK) == CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX)
            {
                this->ori_matrix_subscriber = this->node_handle.subscribe(this->ori_matrix_topic_name, 
                                                                          1,
                                                                          &SensorSubscriber::OriMatrixSubCallback,
                                                                          this);
            }
            
            if((this->output_settings & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK) == CMT_OUTPUTSETTINGS_ORIENTMODE_EULER)
            {
                this->ori_euler_subscriber = this->node_handle.subscribe(this->ori_euler_topic_name, 
                                                                         1,
                                                                         &SensorSubscriber::OriEulerSubCallback,
                                                                         this);
            }
        }
        
        return true;
    }
    
    const dfv::Vector3 SensorSubscriber::GetAcc() const
    {
        return dfv::Vector3(this->acc);
    }
    
    const dfv::Vector3 SensorSubscriber::GetGyr() const
    {
        return dfv::Vector3(this->gyr);
    }
    
    const dfv::Vector3 SensorSubscriber::GetMag() const
    {
        return dfv::Vector3(this->mag);
    }
            
    const dfv::Quaternion SensorSubscriber::GetOriQuat() const
    {
        return dfv::Quaternion(this->ori_quat);
    }
    
    const dfv::Matrix SensorSubscriber::GetOriMatrix() const
    {
        return dfv::Matrix(this->ori_matrix);
    }
    
    const dfv::Vector3 SensorSubscriber::GetOriEuler() const
    {
        return dfv::Vector3(this->ori_euler);
    }
    
    void SensorSubscriber::AccSubCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
    {
        this->acc = dfv::Vector3(msg->vector.x, msg->vector.y, msg->vector.z);
    }
    
    void SensorSubscriber::GyrSubCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
    {
        this->gyr = dfv::Vector3(msg->vector.x, msg->vector.y, msg->vector.z);
    }
            
    void SensorSubscriber::MagSubCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
    {
        this->mag = dfv::Vector3(msg->vector.x, msg->vector.y, msg->vector.z);
    }
    
    void SensorSubscriber::OriQuatSubCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
    {
        this->ori_quat = dfv::Quaternion(msg->quaternion.w, msg->quaternion.x, msg->quaternion.y, msg->quaternion.z);
    }
    
    void SensorSubscriber::OriMatrixSubCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
        dfv::Matrix m(3);
        m.Set(0, 0 , msg->data[0]);
        m.Set(0, 1 , msg->data[1]);
        m.Set(0, 2 , msg->data[2]);
        m.Set(1, 0 , msg->data[3]);
        m.Set(1, 1 , msg->data[4]);
        m.Set(1, 2 , msg->data[5]);
        m.Set(2, 0 , msg->data[6]);
        m.Set(2, 1 , msg->data[7]);
        m.Set(2, 2 , msg->data[8]);
        this->ori_matrix = m;
    }
    
    void SensorSubscriber::OriEulerSubCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
        this->ori_euler = dfv::Vector3(msg->data[0], msg->data[1], msg->data[2]);
    }

// ============================================ //
//          Clase SensorSubscriberList          //
// ============================================ //

    SensorSubscriberList::SensorSubscriberList()
    {
        int param;
        this->node_handle.param<int>("/xsens/sensor_count", param, 0);
        
        for(int i = 0; i < param; ++i)
        {
            SensorSubscriber ss(i);
            this->sensors.push_back(ss);        
        }
    }
    
    SensorSubscriberList::~SensorSubscriberList()
    {
        
    }
    
}
