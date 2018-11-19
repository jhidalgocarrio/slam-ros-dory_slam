#include <dory_slam/DorySLAMNode.hpp>

#define DEBUG_PRINTS 1

using namespace dory_slam_node;
            
const double GRAVITY  = 9.80665; //[m/s^2] Real 9.80665

Node::Node(::ros::NodeHandle &nh)
{
    /** Output port **/
    std::string output_port_name;
    nh.param("output_port_name", output_port_name, std::string("shark_slam/pose"));
    ROS_INFO("got output_port_name: %s", output_port_name.c_str());
    this->pose_port = nh.advertise<::nav_msgs::Odometry>(output_port_name, 10);

    /** Transformer listener **/
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener listener(tf_buffer);

    /** Default parameters for the transformer **/
    std::string body_frame, imu_frame, gps_frame;
    nh.param("body_frame", body_frame, std::string("base_link"));
    ROS_INFO("got param body_frame: %s", body_frame.c_str());
    nh.param("imu_frame", imu_frame, std::string("imu_link"));
    ROS_INFO("got param imu_frame: %s", imu_frame.c_str());
    nh.param("gps_frame", gps_frame, std::string("gps_link"));
    ROS_INFO("got param gps_frame: %s", gps_frame.c_str());

    /** Get the IMU transformation **/
    try
    {
        geometry_msgs::TransformStamped ros_tf;//base_link to imu
        ros_tf = tf_buffer.lookupTransform(body_frame, imu_frame, ros::Time(0), ros::Duration(10));
        this->imu_tf = tf2::transformToEigen(ros_tf);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("IMU TF %s",ex.what());
        ros::Duration(1.0).sleep();
    }

    /** Get the GPS transformation **/
    try
    {
        geometry_msgs::TransformStamped ros_tf;//base_link to gps
        ros_tf = tf_buffer.lookupTransform(body_frame, gps_frame, ros::Time(0), ros::Duration(10));
        this->gps_tf = tf2::transformToEigen(ros_tf);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("GPS TF %s",ex.what());
        ros::Duration(1.0).sleep();
    }

    std::cout<<"[DORY_SLAM] IMU TF:\n"<<this->imu_tf.matrix()<<std::endl;
    std::cout<<"[DORY_SLAM] GPS TF:\n"<<this->gps_tf.matrix()<<std::endl;

    /** The slam object initialization **/
    this->ishark.reset(new shark_slam::iShark());
}

Node::~Node()
{
    /** The slam object destruction **/
    this->ishark.reset();
}

void Node::configureNode(::ros::NodeHandle &nh)
{
    /** config values **/
    double accel_noise_sigma, gyro_noise_sigma;
    double accel_bias_rw_sigma, gyro_bias_rw_sigma;
    double gps_noise_sigma;

    nh.param("accel_noise_sigma", accel_noise_sigma, 0.5);
    ROS_INFO("got param accel_noise_sigma: %f", accel_noise_sigma);
    nh.param("gyro_noise_sigma", gyro_noise_sigma, 0.02);
    ROS_INFO("got param gyro_noise_sigma: %f", gyro_noise_sigma);
    nh.param("accel_bias_rw_sigma", accel_bias_rw_sigma, 0.4905);
    ROS_INFO("got param accel_bias_rw_sigma: %f", accel_bias_rw_sigma);
    nh.param("gyro_bias_rw_sigma", gyro_bias_rw_sigma, 0.01454441043);
    ROS_INFO("got param gyro_bias_rw_sigma: %f", gyro_bias_rw_sigma);
    nh.param("gps_noise_sigma", gps_noise_sigma, 0.3);
    ROS_INFO("got param gps_noise_sigma: %f", gps_noise_sigma);

    this->ishark->configuration(accel_noise_sigma, gyro_noise_sigma,
                                accel_bias_rw_sigma, gyro_bias_rw_sigma,
                                gps_noise_sigma);

}

void Node::imu_msgCallback(const ::sensor_msgs::Imu &msg)
{
    #ifdef DEBUG_PRINTS
    ROS_INFO_STREAM("[DORY_SLAM] IMU_CALLBACK RECEIVED ");
    #endif

    /** Convert ROS message to standard rock-types **/
    ::base::samples::IMUSensors imu_sample;
    this->fromIMUMsgToIMUSensor(msg, imu_sample);

    /** Convert ROS message to standard rock-types **/
    ::base::samples::RigidBodyState orient_sample;
    this->fromIMUMsgToOrientation(msg, orient_sample);

    /** Convert IMU values in robot body frame **/
    imu_sample.gyro = this->imu_tf * imu_sample.gyro;
    imu_sample.acc = this->imu_tf * imu_sample.acc;
    imu_sample.mag = this->imu_tf * imu_sample.mag;

    /** Convert orientation in robot body frame **/
    orient_sample.orientation = orient_sample.orientation * Eigen::Quaterniond(this->imu_tf.rotation().inverse());

    /** Eliminate earth gravity from acceleration **/
    ::Eigen::Vector3d gravity (0.00, 0.00, GRAVITY);
    std::cout<<"acc(w g):\n"<<imu_sample.acc<<"\n";
    gravity = orient_sample.orientation.inverse() * gravity;
    std::cout<<"GRAVITY IN BODY: "<<gravity[0]<<", "<<gravity[1]<<", "<<gravity[2]<<"\n";
    imu_sample.acc = imu_sample.acc - gravity;
    std::cout<<"acc(w/o g):\n"<<imu_sample.acc<<"\n";

    /** Call the ishark function for imu factor**/
    this->ishark->imu_samplesCallback(imu_sample.time, imu_sample);

    /** Call the ishark function for orientation factor **/
    this->ishark->orientation_samplesCallback(orient_sample.time, orient_sample);

    /** Get the pose **/
    this->fromRbsToOdometryMsg(this->ishark->getPose(), this->slam_msg);
    this->pose_port.publish(this->slam_msg);
}

void Node::gps_msgCallback(const ::nav_msgs::Odometry &msg)
{

    #ifdef DEBUG_PRINTS
    ROS_INFO_STREAM("[DORY_SLAM] GPS_CALLBACK RECEIVED ");
    #endif

    /** Convert ROS message to standard rock-types **/
    ::base::samples::RigidBodyState gps_sample;
    this->fromOdometryMsgToRbs(msg, gps_sample);

    /** Convert gps in robot body frame **/
    gps_sample.position = this->gps_tf * gps_sample.position;
    gps_sample.orientation = gps_sample.orientation * Eigen::Quaterniond(this->gps_tf.rotation().inverse());

    /** Call the ishark function **/
    this->ishark->gps_pose_samplesCallback(gps_sample.time, gps_sample);

}

void Node::fromIMUMsgToIMUSensor(const ::sensor_msgs::Imu &msg, ::base::samples::IMUSensors &sample)
{
    sample.time = base::Time::fromMicroseconds(static_cast<int64_t>(msg.header.stamp.toNSec() / 1000.00));
    sample.gyro <<msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z;
    sample.acc <<msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z;
}

void Node::fromIMUSensorToIMUMsg(const ::base::samples::IMUSensors &sample, ::sensor_msgs::Imu &msg)
{
    msg.header.stamp.fromNSec(sample.time.microseconds * 1000.00);
    ::tf::vectorEigenToMsg(sample.gyro, msg.angular_velocity); // gyro -> angular velocity
    ::tf::vectorEigenToMsg(sample.acc, msg.linear_acceleration); // acc -> linear acc
}

void Node::fromIMUMsgToOrientation(const ::sensor_msgs::Imu &msg, ::base::samples::RigidBodyState &sample)
{
    sample.time = base::Time::fromMicroseconds(static_cast<int64_t>(msg.header.stamp.toNSec() / 1000.00));
    sample.orientation = ::base::Orientation(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
}

void Node::fromOrientationToIMUMsg(const ::base::samples::RigidBodyState &sample, ::sensor_msgs::Imu &msg)
{
    msg.header.stamp.fromNSec(sample.time.microseconds * 1000.00);
    ::tf::quaternionEigenToMsg(sample.orientation, msg.orientation);
}

void Node::fromOdometryMsgToRbs(const ::nav_msgs::Odometry &msg, ::base::samples::RigidBodyState &sample)
{
    /* Time **/
    sample.time = base::Time::fromMicroseconds(static_cast<int64_t>(msg.header.stamp.toNSec() / 1000.00));

    /** Pose and Twist **/
    ::Eigen::Affine3d tf_pose;
    tf::poseMsgToEigen(msg.pose.pose, tf_pose);
    sample.setTransform(tf_pose);
    sample.velocity <<  msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z;
    sample.angular_velocity <<  msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z;

    /** Frames **/
    sample.sourceFrame = msg.header.frame_id;
    sample.targetFrame =  msg.child_frame_id;
}

void Node::fromRbsToOdometryMsg(const ::base::samples::RigidBodyState &sample, ::nav_msgs::Odometry &msg)
{
    /* Time **/
    msg.header.stamp.fromNSec(sample.time.microseconds * 1000.00);

    /** Pose and Twist **/
    tf::poseEigenToMsg(sample.getTransform(), msg.pose.pose);
    ::Eigen::Matrix<double, 6, 1> twist; twist << sample.velocity, sample.angular_velocity;
    ::tf::twistEigenToMsg(twist, msg.twist.twist);

    /** Covariance for pose **/
    ::Eigen::Matrix<double, 6, 6> cov = ::Eigen::Matrix<double, 6, 6>::Zero();
    cov.block<3,3>(0,0) = sample.cov_position;
    cov.block<3,3>(3,3) = sample.cov_orientation;
    ::Eigen::Map<Eigen::Matrix<double, 6, 6>>(msg.pose.covariance.data()) = cov;

    /** Covariance for twist **/
    cov.block<3,3>(0,0) = sample.cov_velocity;
    cov.block<3,3>(3,3) = sample.cov_angular_velocity;
    ::Eigen::Map<Eigen::Matrix<double, 6, 6>>(msg.twist.covariance.data()) = cov;

    /** Frames **/
    msg.header.frame_id = sample.sourceFrame;
    msg.child_frame_id = sample.targetFrame;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dory_slam_node");
    ros::NodeHandle nh("~");

    /* Create ishark object **/
    dory_slam_node::Node node(nh);

    /** Configure SLAM node **/
    node.configureNode(nh);

    /** Subscribe to the IMU sensor topic **/
    std::string imu_port_name;
    nh.param("imu_port_name", imu_port_name, std::string("/dory/imu/data"));
    ROS_INFO("got param imu_port_name: %s", imu_port_name.c_str());
    ros::Subscriber imu_sub = nh.subscribe(imu_port_name, 100, &dory_slam_node::Node::imu_msgCallback, &node);

    /** Subscribe to the GPS sensor topic **/
    std::string gps_port_name;
    nh.param("gps_port_name", gps_port_name, std::string("/dory/odometry/gps2"));
    ROS_INFO("got param gps_port_name: %s", gps_port_name.c_str());
    ros::Subscriber gps_sub = nh.subscribe(gps_port_name, 100, &dory_slam_node::Node::gps_msgCallback, &node);

    /** Let ROS take over and hope for the best **/
    ros::spin();
}
