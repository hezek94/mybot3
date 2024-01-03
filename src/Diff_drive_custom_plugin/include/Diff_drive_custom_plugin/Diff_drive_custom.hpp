#ifndef GAZEBO_ROS_MODEL_PLUGIN_PAIR_OF_WHEEL_DIFF_DRIVE_HPP_
#define GAZEBO_ROS_MODEL_PLUGIN_PAIR_OF_WHEEL_DIFF_DRIVE_HPP_

#include <gazebo/common/Plugin.hh>
#include<gazebo-11/gazebo/common/Plugin.hh>
#include <memory>

namespace gazebo_plugins
{
    class GazeboPrivateDiffDrive;

    //diff drive plugin on gazebo 
    
    class GazeboDiffDrive : public gazebo::ModelPlugin
    {
        public:
            //constructor
            GazeboDiffDrive();
            //destructor
            ~GazeboDiffDrive();
        protected:
            void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;
            void Reset() override;

        private:
            std::unique_ptr<GazeboPrivateDiffDrive> _driveImpl;
    };
    
}



#endif //!GAZEBO_ROS_MODEL_PLUGIN_PAIR_OF_WHEEL_DIFF_DRIVE_HPP_