#include <dfv/xsens_listener.h>

namespace dfv
{
    SensorSubscriber::SensorSubscriber(unsigned int mt_index_, ros::NodeHandle& node_handle_):
        node_handle(node_handle_), mt_index(mt_index_)
    {
        std::stringstream ss;
        ss << "/xsens_node/sensor" << this->mt_index << "/acc";
        this->acc_topic_name = ss.str();
        
        ss.str(std::string());
        ss << "/xsens_node/sensor" << this->mt_index << "/gyr";
        this->gyr_topic_name = ss.str();
        
        ss.str(std::string());
        ss << "/xsens_node/sensor" << this->mt_index << "/mag";
        this->mag_topic_name = ss.str();
        
        ss.str(std::string());
        ss << "xsens_node/sensor" << this->mt_index << "/ori_quat";
        this->ori_quat_topic_name = ss.str();
        
        ss.str(std::string());
        ss << "/xsens_node/sensor" << this->mt_index << "/ori_matrix";
        this->ori_matrix_topic_name = ss.str();
        
        ss.str(std::string());
        ss << "/xsens_node/sensor" << this->mt_index << "/ori_euler";
        this->ori_euler_topic_name = ss.str();
        
        //int param;
        //this->node_handle.param<int>("/xsens_node/output_mode", param, 0);
        //this->output_mode = param;
        //this->node_handle.param<int>("/xsens_node/output_settings", param, 0);
        //this->output_settings = param;
        
        this->SubscribeToTopics();
        
    }
    
    SensorSubscriber::~SensorSubscriber()
    {
        
    }
    
    bool SensorSubscriber::SubscribeToTopics()
    {

        ROS_INFO("[SensorSubscriber] Subscribing to calibrated data topics...");
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



        ROS_INFO("[SensorSubscriber] Subscribing to ori_quat topic...");
        this->ori_quat_subscriber = this->node_handle.subscribe(this->ori_quat_topic_name, 
                                                                1,
                                                                &SensorSubscriber::OriQuatSubCallback,
                                                                this);

    

        ROS_INFO("[SensorSubscriber] Subscribing to ori_matrix topic...");
        this->ori_matrix_subscriber = this->node_handle.subscribe(this->ori_matrix_topic_name, 
                                                                  1,
                                                                  &SensorSubscriber::OriMatrixSubCallback,
                                                                  this);

    

        ROS_INFO("[SensorSubscriber] Subscribing to ori_euler topic...");
        this->ori_euler_subscriber = this->node_handle.subscribe(this->ori_euler_topic_name, 
                                                                 1,
                                                                 &SensorSubscriber::OriEulerSubCallback,
                                                                 this);


        
        /*if((this->output_mode & CMT_OUTPUTMODE_CALIB) != 0)
        {
            ROS_INFO("[SensorSubscriber] Subscribing to calibrated data topics...");
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
                ROS_INFO("[SensorSubscriber] Subscribing to ori_quat topic...");
                this->ori_quat_subscriber = this->node_handle.subscribe(this->ori_quat_topic_name, 
                                                                        1,
                                                                        &SensorSubscriber::OriQuatSubCallback,
                                                                        this);
            }
            
            if((this->output_settings & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK) == CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX)
            {
                ROS_INFO("[SensorSubscriber] Subscribing to ori_matrix topic...");
                this->ori_matrix_subscriber = this->node_handle.subscribe(this->ori_matrix_topic_name, 
                                                                          1,
                                                                          &SensorSubscriber::OriMatrixSubCallback,
                                                                          this);
            }
            
            if((this->output_settings & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK) == CMT_OUTPUTSETTINGS_ORIENTMODE_EULER)
            {
                ROS_INFO("[SensorSubscriber] Subscribing to ori_euler topic...");
                this->ori_euler_subscriber = this->node_handle.subscribe(this->ori_euler_topic_name, 
                                                                         1,
                                                                         &SensorSubscriber::OriEulerSubCallback,
                                                                         this);
            }
        }*/
        
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

// =========================================== //
//             Clase XsensListener             //
// =========================================== //

    XsensListener::XsensListener(ros::NodeHandle& node_handle_):
        node_handle(node_handle_)
    {
        int param;
        this->node_handle.param<int>("/xsens_node/sensor_count", param, 0);
        this->mt_count = param;
        //this->sensors = new SensorSubscriber*[this->mt_count];
        this->sensors.resize(this->mt_count);
        
        for(int i = 0; i < param; ++i)
        {
            this->sensors[i] = new SensorSubscriber(i, this->node_handle);                
        }
    }
    
    XsensListener::~XsensListener()
    {
        for(unsigned int i = 0; i < this->mt_count; ++i)
        {
            delete this->sensors[i];
        }
        //delete this->sensors;
    }
    
    unsigned int XsensListener::GetMtCount() const
    {
        return this->mt_count;
    }
    
    unsigned int XsensListener::Count() const
    {
        return this->mt_count;
    }
    
    const dfv::Vector3 XsensListener::GetAcc(unsigned int mt_index) const
    {
        return dfv::Vector3(this->sensors[mt_index]->GetAcc());
    }
    
    const dfv::Vector3 XsensListener::GetGyr(unsigned int mt_index) const
    {
        return dfv::Vector3(this->sensors[mt_index]->GetGyr());
    }
    
    const dfv::Vector3 XsensListener::GetMag(unsigned int mt_index) const
    {
        return dfv::Vector3(this->sensors[mt_index]->GetMag());
    }
            
    const dfv::Quaternion XsensListener::GetOriQuat(unsigned int mt_index) const
    {
        return dfv::Quaternion(this->sensors[mt_index]->GetOriQuat());
    }
    
    const dfv::Matrix XsensListener::GetOriMatrix(unsigned int mt_index) const
    {
        return dfv::Matrix(this->sensors[mt_index]->GetOriMatrix());
    }
    
    const dfv::Vector3 XsensListener::GetOriEuler(unsigned int mt_index) const
    {
        return dfv::Vector3(this->sensors[mt_index]->GetOriEuler());
    }
    
}
