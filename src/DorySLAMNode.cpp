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

void Node::imu_msgCallback(const ::sensor_msgs::Imu &msg)
{
    ROS_INFO_STREAM("[DORY_SLAM] IMU_CALLBACK RECEIVED ");

    /** Convert ROS message to standard rock-types **/
    ::base::samples::IMUSensors imu_sample;
    this->fromIMUMsg(msg, imu_sample);

    /** Call the ishark function for imu factor**/
    this->ishark->imu_samplesCallback(imu_sample.time, imu_sample);

    /** Convert ROS message to standard rock-types **/
    ::base::samples::RigidBodyState orient_sample;

    /** Call the ishark function for orientation factor**/
    this->ishark->orientation_samplesCallback(orient_sample.time, orient_sample);

}

void Node::gps_msgCallback(const ::nav_msgs::Odometry &msg)
{
    ROS_INFO_STREAM("[DORY_SLAM] GPS_CALLBACK RECEIVED ");

    /** Convert ROS message to standard rock-types **/
    ::base::Time timestamp;
    ::base::samples::RigidBodyState gps_sample;

    /** Call the ishark function **/
    this->ishark->gps_pose_samplesCallback(timestamp, gps_sample);

}

void Node::fromIMUMsg(const ::sensor_msgs::Imu &msg, ::base::samples::IMUSensors &sample)
{
    sample.time.fromMicroseconds(msg.header.stamp.toNSec() / 1000.00);
    sample.gyro <<msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z;
    sample.acc <<msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z;
}

void Node::toIMUMsg(const ::base::samples::IMUSensors &sample, ::sensor_msgs::Imu &msg)
{
    msg.header.stamp.fromNSec(sample.time.microseconds * 1000.00);
    ::tf::vectorEigenToMsg(sample.gyro, msg.angular_velocity); // gyro -> angular velocity
    ::tf::vectorEigenToMsg(sample.acc, msg.linear_acceleration); // acc -> linear acc
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dory_slam_node");
    ros::NodeHandle nh;

    /* Create ishark object **/
    dory_slam_node::Node node;

    /** Configure SLAM properties **/

    /** Initialize SLAM node **/
    tf::StampedTransform init_tf;
    node.initialization(init_tf);

    /** Subscribe to the IMU sensor topics **/
    ros::Subscriber imu_sub = nh.subscribe("/dory/imu/data", 100, &dory_slam_node::Node::imu_msgCallback, &node);

    /** Subscribe to the GPS sensor topics **/
    ros::Subscriber gps_sub = nh.subscribe("/dory/odometry/gps2", 100, &dory_slam_node::Node::gps_msgCallback, &node);

    /** Let ROS take over and hope for the best **/
    ros::spin();
}
