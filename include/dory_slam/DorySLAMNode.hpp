#include <ros/ros.h>
#include <shark_slam/iShark.hpp>

/** ROS Sensor messages **/
#include <sensor_msgs/Imu.h> //Orientation angular velocity and linear acceleration

/** ROS Nav messages **/
#include <nav_msgs/Odometry.h>

/** ROS Transformation messages **/
#include <geometry_msgs/TransformStamped.h>

/** Transform listener **/
#include <tf2_ros/transform_listener.h>

/** ROS to Eigen conversions **/
#include <eigen_conversions/eigen_msg.h>

/** ROS to TF2 to Eigen **/
#include <tf2_eigen/tf2_eigen.h>

/** C++ std  libraries **/
#include <memory> // shared pointers

namespace dory_slam_node
{
    class Node
    {
	public:
	   EIGEN_MAKE_ALIGNED_OPERATOR_NEW //Structures having Eigen members

        protected:
            /** iShark slam variable **/
            std::shared_ptr<shark_slam::iShark> ishark;

            /** Input port messages **/
            sensor_msgs::Imu imu_msg;
            nav_msgs::Odometry slam_msg;
            ros::Publisher pose_port;

            /** Transformations **/
            Eigen::Affine3d imu_tf; //base_link to imu
            Eigen::Affine3d gps_tf; //base_link to gps


        public:
            /** Default constructor **/
            Node(::ros::NodeHandle &nh);
            virtual ~Node();

            /** Configuration **/
            void configureNode(::ros::NodeHandle &nh);

            /** IMU call back **/
            void imu_msgCallback(const ::sensor_msgs::Imu &msg);

            /** GPS callback **/
            void gps_msgCallback(const ::nav_msgs::Odometry &msg);

        protected:
            void fromIMUMsgToIMUSensor(const ::sensor_msgs::Imu &msg, ::base::samples::IMUSensors &sample);
            void fromIMUSensorToIMUMsg(const ::base::samples::IMUSensors &sample, ::sensor_msgs::Imu &msg);

            void fromIMUMsgToOrientation(const ::sensor_msgs::Imu &msg, ::base::samples::RigidBodyState &sample);
            void fromOrientationToIMUMsg(const ::base::samples::RigidBodyState &sample, ::sensor_msgs::Imu &msg);

            void fromOdometryMsgToRbs(const ::nav_msgs::Odometry &msg, ::base::samples::RigidBodyState &sample);
            void fromRbsToOdometryMsg(const ::base::samples::RigidBodyState &sample, ::nav_msgs::Odometry &msg);
    };
}
