#include <ros/ros.h>
#include <shark_slam/iShark.hpp>

/** ROS Sensor messages **/
#include <sensor_msgs/Imu.h> //Orientation angular velocity and linear acceleration

/** ROS Nav messages **/
#include <nav_msgs/Odometry.h>

/** Transform listener **/
#include <tf/transform_listener.h>

/** ROS to Eigen conversions **/
#include <eigen_conversions/eigen_msg.h>

namespace dory_slam_node
{
    class Node
    {
        protected:
            shark_slam::iShark *ishark;
            sensor_msgs::Imu imu_msg;

        public:
            Node();
            virtual ~Node();
            void initialization(const tf::StampedTransform &transform);
            void imu_msgCallback(const ::sensor_msgs::Imu &msg);
            void gps_msgCallback(const ::nav_msgs::Odometry &msg);

        protected:
            void fromIMUMsgToIMUSensor(const ::sensor_msgs::Imu &msg, ::base::samples::IMUSensors &sample);
            void fromIMUSensorToIMUMsg(const ::base::samples::IMUSensors &sample, ::sensor_msgs::Imu &msg);

            void fromIMUMsgToOrientation(const ::sensor_msgs::Imu &msg, ::base::samples::RigidBodyState &sample);
    };
}
