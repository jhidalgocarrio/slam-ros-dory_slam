#include "dory_slam/DorySLAMNode.hpp"

using namespace dory_slam_node;

Node::Node()
{

    this->ishark = new shark_slam::iShark();
}

Node::~Node()
{
    delete this->ishark;
}

void Node::initialization(const tf::StampedTransform &transform)
{
    ROS_INFO_STREAM("DORY_SLAM INITIALIZATION\n");

    /** Convert ROS transform to Eigen Affine 3d **/
    Eigen::Affine3d tf;

    /** Initialize **/
    this->ishark->initialization(tf);

}

void Node::imu_msgCallback(const sensor_msgs::Imu &msg)
{
    ROS_INFO_STREAM("[DORY_SLAM] IMU_CALLBACK RECEIVED ");

    /** Convert ROS message to standard rock-types **/
    ::base::Time timestamp;
    ::base::samples::IMUSensors imu_sample;

    imu_sample.gyro <<msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z;

    /** Call the ishark function **/
    this->ishark->imu_samplesCallback(timestamp, imu_sample);

}

void Node::gps_msgCallback(const nav_msgs::Odometry &msg)
{
    ROS_INFO_STREAM("[DORY_SLAM] GPS_CALLBACK RECEIVED ");

    /** Convert ROS message to standard rock-types **/
    ::base::Time timestamp;
    ::base::samples::RigidBodyState gps_sample;

    /** Call the ishark function **/
    this->ishark->gps_pose_samplesCallback(timestamp, gps_sample);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dory_slam_node");
    ros::NodeHandle nh;

    /* Create ishark object **/
    dory_slam_node::Node node;

    /** Configure **/

    /** Initialize **/
    tf::StampedTransform init_tf;
    node.initialization(init_tf);

    /** Subscribe to the IMU sensor topics **/
    ros::Subscriber imu_sub = nh.subscribe("/dory/imu/data", 100, &dory_slam_node::Node::imu_msgCallback, &node);

    /** Subscribe to the GPS sensor topics **/
    ros::Subscriber gps_sub = nh.subscribe("/dory/odometry/gps2", 100, &dory_slam_node::Node::gps_msgCallback, &node);

    /** Let ROS take over and hope for the best **/
    ros::spin();
}
