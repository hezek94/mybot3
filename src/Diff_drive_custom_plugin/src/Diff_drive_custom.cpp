#include <algorithm>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include "Diff_drive_custom_plugin/Diff_drive_custom.hpp"
#include  <sdf/sdf.hh>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include<boost/bind.hpp>
#include<boost/thread/mutex.hpp>
#include <ignition/math/Vector3.hh>

#include<memory>
#include<string>
#include<cstring>
#include<vector>

#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif

// tranform and broadcaster 

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>


namespace gazebo_plugins
{
    class GazeboPrivateDiffDrive{
        public:
            enum{
                RIGHT = 0, //For rightneheel index
                LEFT = 1, //For left wheel index
            };
            enum Odom_source{
                WHEEL = 0, //the wheel enconder from the child frame
                WORLD = 1, //the world 
            };
            //params, subcriber, publisher needed in the plugin 
            std::vector<double> wheel_separation_; // distance btw a pair of wheel
            std::vector<double> wheel_diameter_; // diam of each wheel
            double max_wheel_accelaration_; //the max wheel accel
            double max_wheel_torque; // max torque of each wheel
            std::vector<double> wheel_speed;
            std::vector<double> wheel_speed_ini;
            
            gazebo_ros::Node::SharedPtr rosnode_;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
            gazebo::event::ConnectionPtr update_connection_;

            std::vector<gazebo::physics::JointPtr> joints_;
            gazebo::physics::ModelPtr parent_;

            std::shared_ptr<tf2_ros::TransformBroadcaster> TF_broadcaster_;
            std::mutex lock_;
            double targetX_{0.0}; //linear velocity from feedback cmd
            double targetZ{0.0}; //angular velocity from feedback command
            gazebo::common::Time last_update_time_;
            
            geometry_msgs::msg::Pose2D pose_ed_;
            std::string odometry_frame_;
            gazebo::common::Time last_encoder_update_;
            Odom_source odom_source_;  
            double update_period_;
            nav_msgs::msg::Odometry odom_;
            std::string robot_base_frame_;
            bool publish_odom_;
            bool publish_wheel_tf_;
            bool publish_odom_tf_;
            unsigned int wheel_pair_num;

            //function needed
            void onChildUpdate(const gazebo::common::UpdateInfo& _infr);
            void onCmdVelCbk(const geometry_msgs::msg::Twist::SharedPtr _msg);
            void updateOdometry(const gazebo::common::Time & _current_time);
            void updateViaWheelEc(const gazebo::common::Time &_current_time);
            void wheelTFPublisher(const gazebo::common::Time &_current_time);
            void odomTFPublisher(const gazebo::common::Time &current_time);
            void wheelVelUpdate();
            void PublishOdometry(const gazebo::common::Time &current_time); 
        };

    //differential drive plugin
    GazeboDiffDrive::GazeboDiffDrive(){
        _driveImpl = std::make_unique<GazeboPrivateDiffDrive>();
    }
    GazeboDiffDrive::~GazeboDiffDrive(){}
    void GazeboDiffDrive::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        _driveImpl->parent_ = _parent; 

        _driveImpl->rosnode_ = gazebo_ros::Node::Get(_sdf); // rosnode initializatn

        const gazebo_ros::QoS &qos = _driveImpl->rosnode_->get_qos();//Quality of Service profile setup

        _driveImpl->max_wheel_torque = _sdf->Get<double>("max_wheel_torque", 5.0).first;
        _driveImpl->max_wheel_accelaration_ = _sdf->Get<double>("max_wheel_accelaration", 0.0).first;
        _driveImpl->wheel_pair_num = static_cast<unsigned int>(_sdf->Get<int>("wheel_pair_num", 1).first);
        if(_driveImpl->wheel_pair_num<1){
            _driveImpl->wheel_pair_num = 1;

            RCLCPP_WARN(_driveImpl->rosnode_->get_logger(), "the driver required minimum of a wheel pair. let[num of wheel pair] to 1");
            
        }

        std::vector<gazebo::physics::JointPtr> left_joints, right_joints;
        auto left_jointElemPtr = _sdf->GetElement("left_joint");

        while(left_jointElemPtr != nullptr){
            auto joint_name = left_jointElemPtr->Get<std::string>();
            auto joint = _parent->GetJoint(joint_name);
            if (!joint)
            {
                RCLCPP_ERROR(_driveImpl->rosnode_->get_logger(), "Joint [%s] not found, plugin will not work.", joint_name.c_str());
                _driveImpl->rosnode_.reset();
                return;
                
            }
            joint->SetParam("fmax", 0, _driveImpl->max_wheel_torque);
            left_joints.push_back(joint);
            left_jointElemPtr = _sdf->GetNextElement("left_joint");
        }
        auto right_jointElemPtr = _sdf->GetElement("right_joint");
        
        while(right_jointElemPtr != nullptr){
            auto joint_name = right_jointElemPtr->Get<std::string>();
            auto joint = _parent->GetJoint(joint_name);
            if (!joint)
            {
                RCLCPP_ERROR(_driveImpl->rosnode_->get_logger(), "Joint [%s] not found, plugin will not work.", joint_name.c_str());
                _driveImpl->rosnode_.reset();
                return;
                
            }
            joint->SetParam("fmax", 0, _driveImpl->max_wheel_torque);
            right_joints.push_back(joint);
            right_jointElemPtr = _sdf->GetNextElement("right_joint");
        }
        //to verify if the num of right is same as num of left joint

        if(left_joints.size()!=right_joints.size() || right_joints.size()!=_driveImpl->wheel_pair_num)
        {
            RCLCPP_ERROR(_driveImpl->rosnode_->get_logger(), "inconsistency in the number of joint" );
            _driveImpl->rosnode_.reset();
            return;
            
        }
        
        for (unsigned int e{};e<_driveImpl->wheel_pair_num; e++){
            _driveImpl->joints_.push_back(right_joints[e]);
            _driveImpl->joints_.push_back(left_joints[e]);
        }

        unsigned int index;

        //the number of wheel pair mustbe assingend to the value
        //instead of 
        //_driveImpl->wheel_separation_ = _sdf->Get<double>("wheel_seperation", 0.14).first;
        _driveImpl->wheel_separation_.assign(_driveImpl->wheel_pair_num, 0.35);
        index=0;
        auto wheel_seperation = _sdf->GetElement("wheel_seperation");
        while(wheel_seperation != nullptr){
            _driveImpl->wheel_separation_[index] = wheel_seperation->Get<double>();
            if(index >= _driveImpl->wheel_pair_num){
                RCLCPP_WARN(_driveImpl->rosnode_->get_logger(), "index of will seperation exceeded the limit");
                break;
                
            }
            wheel_seperation = _sdf->GetNextElement("wheel_seperation");
            index++;
        }

        _driveImpl->wheel_diameter_.assign(_driveImpl->wheel_pair_num, 0.35);
        index=0;
        auto wheel_diameter = _sdf->GetElement("wheel_diameter");
        while(wheel_diameter != nullptr){
            _driveImpl->wheel_diameter_[index] = wheel_diameter->Get<double>();
            if(index >= _driveImpl->wheel_pair_num){
                RCLCPP_WARN(_driveImpl->rosnode_->get_logger(), "index of will seperation exceeded the limit");
                break;
                
            }
            wheel_diameter = _sdf->GetNextElement("wheel_diameter");
            index++;
        }
        _driveImpl->wheel_speed_ini.assign(_driveImpl->wheel_pair_num, 0);

        _driveImpl->wheel_speed.assign(_driveImpl->wheel_pair_num, 0);
        auto update_rate = _sdf->Get<double>("update_rate", 100.0).first;
        _driveImpl->update_period_ = (update_rate > 0.0) ? 1.0/update_rate : 0.0;
        _driveImpl->last_update_time_ = _parent->GetWorld()->SimTime();

        _driveImpl->cmd_vel_sub_ = _driveImpl->rosnode_->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)),
            std::bind(&GazeboPrivateDiffDrive::onCmdVelCbk, _driveImpl.get(), std::placeholders::_1));

        RCLCPP_INFO(_driveImpl->rosnode_->get_logger(), "Subscribed to [%s]", _driveImpl->cmd_vel_sub_->get_topic_name());
        //odometry advertise topic
        _driveImpl->publish_odom_ = _sdf->Get<bool>("publish_odom", true).first;
        if(_driveImpl->publish_odom_){
            _driveImpl->odom_pub_ = _driveImpl->rosnode_->create_publisher<nav_msgs::msg::Odometry>("odom", qos.get_publisher_qos("odom", rclcpp::QoS(1)));
            RCLCPP_INFO(_driveImpl->rosnode_->get_logger(), "Advertise odometry on [%s]",
            _driveImpl->odom_pub_->get_topic_name());
        }
        // Odometry
        _driveImpl->odometry_frame_ = _sdf->Get<std::string>("odometry_frame", "odom").first;

        _driveImpl->robot_base_frame_ = _sdf->Get<std::string>("robot_base_frame", "base_footprint").first;

        _driveImpl->odom_source_ = static_cast<GazeboPrivateDiffDrive::Odom_source>(_sdf->Get<int>("odometry_source", 1).first);

        _driveImpl->publish_wheel_tf_ = _sdf->Get<bool>("publish_wheel_tf", false).first;

        _driveImpl->publish_odom_tf_ = _sdf->Get<bool>("publish_odom_tf", false).first;

        if (_driveImpl->publish_wheel_tf_ || _driveImpl->publish_odom_tf_) {
            _driveImpl->TF_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(_driveImpl->rosnode_);
            if(_driveImpl->publish_odom_tf_){
                RCLCPP_INFO(_driveImpl->rosnode_->get_logger(),
                "Publishing odom transforms between [%s] and [%s]", _driveImpl->odometry_frame_.c_str(),
                _driveImpl->robot_base_frame_.c_str());}

            for(index = 0; index<_driveImpl->wheel_pair_num; ++index){
                if (_driveImpl->publish_odom_tf_)
                {
                    /* code */
                    RCLCPP_INFO(_driveImpl->rosnode_->get_logger(), "Publishing wheel transforms between [%s], [%s] and [%s]",
                    _driveImpl->robot_base_frame_.c_str(),
                    _driveImpl->joints_[2 * index + GazeboPrivateDiffDrive::LEFT]->GetName().c_str(),
                    _driveImpl->joints_[2 * index + GazeboPrivateDiffDrive::RIGHT]->GetName().c_str());
                    
                }
                
            }

        }
        _driveImpl->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&GazeboPrivateDiffDrive::onChildUpdate, _driveImpl.get(), std::placeholders::_1));

    }
    void GazeboDiffDrive::Reset(){
        _driveImpl->last_update_time_ =
            _driveImpl->joints_[GazeboPrivateDiffDrive::LEFT]->GetWorld()->SimTime();
        for (unsigned int i = 0; i < _driveImpl->wheel_pair_num; ++i) {
            if (_driveImpl->joints_[2 * i + GazeboPrivateDiffDrive::LEFT] &&
            _driveImpl->joints_[2 * i + GazeboPrivateDiffDrive::RIGHT])
            {
            _driveImpl->joints_[2 * i + GazeboPrivateDiffDrive::LEFT]->SetParam(
                "fmax", 0, _driveImpl->max_wheel_torque);
            _driveImpl->joints_[2 * i + GazeboPrivateDiffDrive::RIGHT]->SetParam(
                "fmax", 0, _driveImpl->max_wheel_torque);
                 }
    
            }
            _driveImpl->pose_ed_.x = 0;
            _driveImpl->pose_ed_.y = 0;
            _driveImpl->pose_ed_.theta = 0;
            _driveImpl->targetX_ = 0;
            _driveImpl->targetZ= 0;
    }

    void GazeboPrivateDiffDrive::onChildUpdate(const gazebo::common::UpdateInfo& _infr){
        #ifdef IGN_PROFILER_ENABLE
        IGN_PROFILE("GazeboPrivateDiffDrive::OnChildUpdate");
        IGN_PROFILE_BEGIN("UpdateOdometry");
        #endif
        gazebo::common::Time current_time = _infr.simTime;
        if (publish_odom_) {
        #ifdef IGN_PROFILER_ENABLE
            IGN_PROFILE_BEGIN("PublishOdometryMsg");
        #endif
            updateOdometry(current_time);
        #ifdef IGN_PROFILER_ENABLE
            IGN_PROFILE_END();
        #endif
        }
        
        double seconds_since_last_update = ( current_time - last_encoder_update_ ).Double();

        if (seconds_since_last_update < update_period_) {
            return;
        }

        if (odom_source_ == WHEEL) {
            updateViaWheelEc(current_time);
        }
        #ifdef IGN_PROFILER_ENABLE
        IGN_PROFILE_END();
        #endif

        if (publish_wheel_tf_) {
        #ifdef IGN_PROFILER_ENABLE
            IGN_PROFILE_BEGIN("PublishWheelsTf");
        #endif
            wheelTFPublisher(current_time);
        #ifdef IGN_PROFILER_ENABLE
            IGN_PROFILE_END();
        #endif
        }
        if (publish_odom_tf_) {
        #ifdef IGN_PROFILER_ENABLE
            IGN_PROFILE_BEGIN("PublishWheelJointState");
        #endif
            odomTFPublisher(current_time);
        #ifdef IGN_PROFILER_ENABLE
            IGN_PROFILE_END();
        #endif
        }
        if (publish_odom_tf_) {
        
            PublishOdometry(current_time);
        }
        
        #ifdef IGN_PROFILER_ENABLE
        IGN_PROFILE_END();
        IGN_PROFILE_BEGIN("UpdateWheelVelocities");
        #endif
        // Update robot in case new velocities have been requested
        wheelVelUpdate();
        #ifdef IGN_PROFILER_ENABLE
            IGN_PROFILE_END();
        #endif
        //  std::vector<double> current_speed(2 * wheel_pair_num);
        //     for (unsigned int i = 0; i < wheel_pair_num; ++i) {
        //         current_speed[2 * i + LEFT] =
        //         joints_[2 * i + LEFT]->SetVelocity(0,  wheel_speed[2 * i + LEFT] * wheel_diameter_[i] / 2.0);
        //         current_speed[2 * i + RIGHT] =
        //         joints_[2 * i + RIGHT]->SetVelocity(0, wheel_speed[2 * i + RIGHT] * wheel_diameter_[i] / 2.0);

        //     }
        std::vector<double> current_speed(2 * wheel_pair_num);
        for (unsigned int i = 0; i < wheel_pair_num; ++i) {
            current_speed[2 * i + LEFT] =
            joints_[2 * i + LEFT]->GetVelocity(0) * (wheel_diameter_[i] / 2.0);
            current_speed[2 * i + RIGHT] =
            joints_[2 * i + RIGHT]->GetVelocity(0) * (wheel_diameter_[i] / 2.0);
        }
        for (unsigned int i = 0; i < wheel_pair_num; ++i) {
            if (max_wheel_accelaration_ == 0 )
            {
            joints_[2 * i + LEFT]->SetParam(
                "vel", 0, wheel_speed[2 * i + LEFT] / (wheel_diameter_[i] / 2.0));
            joints_[2 * i + RIGHT]->SetParam(
                "vel", 0, wheel_speed[2 * i + RIGHT] / (wheel_diameter_[i] / 2.0));
            }
            
            }

        last_update_time_ = _infr.simTime;

    }
    void GazeboPrivateDiffDrive::PublishOdometry(const gazebo::common::Time &current_time){
       
        std::vector<double> cov;
        cov = {1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 1e3 
                };
            // Copy values from cov to odom_.pose.covariance
        memcpy(&odom_.pose.covariance[0], cov.data(), sizeof(double)*36);
            // Copy values from cov to odom_.twist.covariance
        memcpy(&odom_.twist.covariance[0], cov.data(), sizeof(double)*36);
        // Set header
        odom_.header.frame_id = odometry_frame_;
        odom_.child_frame_id = robot_base_frame_;
        odom_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);

        // Publish
        odom_pub_->publish(odom_);
    }
    void GazeboPrivateDiffDrive::updateOdometry(const gazebo::common::Time & _current_time){
        if(odom_source_ == WORLD){
            ignition::math::Pose3d pos = parent_->WorldPose();
            //get the position 
            odom_.pose.pose.position.x = pos.Pos().X();
            odom_.pose.pose.position.y = pos.Pos().Y();
            odom_.pose.pose.position.z = pos.Pos().Z();
            //orientation 
            odom_.pose.pose.orientation.x = pos.Rot().X();
            odom_.pose.pose.orientation.y = pos.Rot().Y();
            odom_.pose.pose.orientation.z = pos.Rot().Z();
            odom_.pose.pose.orientation.w = pos.Rot().W();

            // both position and orientation can also be represented using

            //odom_.pose.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pos.Pos());
            //odom_.pose.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pos.Rot());
            //twist msg
            ignition::math::Vector3d linear = parent_->WorldLinearVel();
            float yaw = pos.Rot().Yaw();
            odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
            odom_.twist.twist.linear.x = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
            odom_.twist.twist.angular.z = parent_->WorldAngularVel().Z();

        }
        
            }

    void GazeboPrivateDiffDrive::onCmdVelCbk(const geometry_msgs::msg::Twist::SharedPtr _msg){
        std::lock_guard<std::mutex> scoped_lock(lock_);
        targetX_ = _msg->linear.x;
        targetZ  = _msg->angular.z;
    }

    void GazeboPrivateDiffDrive::wheelVelUpdate(){
        std::lock_guard<std::mutex> scoped_lock(lock_);
        double vr = targetX_;
        double va = targetZ;
        unsigned int i{};
        while(i<wheel_pair_num){
            wheel_speed[2*i + LEFT] = vr - va * wheel_separation_[i]/2.0;
            wheel_speed[2*i + RIGHT] = vr + va * wheel_separation_[i]/2.0;
            i++;
        }
  
    }
   
    void GazeboPrivateDiffDrive::updateViaWheelEc(const gazebo::common::Time &_current_time){
        double vl = joints_[LEFT]->GetVelocity(0);
        double vr = joints_[RIGHT]->GetVelocity(0);

        double seconds_since_last_update = (_current_time - last_encoder_update_).Double();
        last_encoder_update_ = _current_time;

        double b = wheel_separation_[0];

        // Book: Sigwart 2011 Autonompus Mobile Robots page:337
        double sl = vl * (wheel_diameter_[0] / 2.0) * seconds_since_last_update;
        double sr = vr * (wheel_diameter_[0] / 2.0) * seconds_since_last_update;
        double ssum = sl + sr;

        double sdiff = sr - sl;

        double dx = (ssum) / 2.0 * cos(pose_ed_.theta + (sdiff) / (2.0 * b));
        double dy = (ssum) / 2.0 * sin(pose_ed_.theta + (sdiff) / (2.0 * b));
        double dtheta = (sdiff) / b;

        pose_ed_.x += dx;
        pose_ed_.y += dy;
        pose_ed_.theta += dtheta;

        double w = dtheta / seconds_since_last_update;
        double v = sqrt(dx * dx + dy * dy) / seconds_since_last_update;

        tf2::Quaternion qt;
        tf2::Vector3 vt;
        qt.setRPY(0, 0, pose_ed_.theta);
        vt = tf2::Vector3(pose_ed_.x, pose_ed_.y, 0);

        odom_.pose.pose.position.x = vt.x();
        odom_.pose.pose.position.y = vt.y();
        odom_.pose.pose.position.z = vt.z();

        odom_.pose.pose.orientation.x = qt.x();
        odom_.pose.pose.orientation.y = qt.y();
        odom_.pose.pose.orientation.z = qt.z();
        odom_.pose.pose.orientation.w = qt.w();

        odom_.twist.twist.angular.z = w;
        odom_.twist.twist.linear.x = v;
        odom_.twist.twist.linear.y = 0;
    }

    void GazeboPrivateDiffDrive::odomTFPublisher(const gazebo::common::Time &current_time){
        geometry_msgs::msg::TransformStamped msg;
        msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);
        msg.header.frame_id = odometry_frame_;
        msg.child_frame_id = robot_base_frame_;
        msg.transform.translation = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(odom_.pose.pose.position);
        msg.transform.rotation = odom_.pose.pose.orientation;
        TF_broadcaster_->sendTransform(msg);
    }
    void GazeboPrivateDiffDrive::wheelTFPublisher(const gazebo::common::Time &current_time){
        for (unsigned int i = 0; i < 2 * wheel_pair_num; ++i) {
        auto pose = joints_[i]->GetChild()->RelativePose();

        geometry_msgs::msg::TransformStamped msg;
        msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);
        msg.header.frame_id = joints_[i]->GetParent()->GetName();
        msg.child_frame_id = joints_[i]->GetChild()->GetName();
        msg.transform.translation = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(pose.Pos());
        msg.transform.rotation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());

        TF_broadcaster_->sendTransform(msg);
  }
    }
    GZ_REGISTER_MODEL_PLUGIN(GazeboDiffDrive)

    
}

