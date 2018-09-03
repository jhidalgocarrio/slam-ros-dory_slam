#include "dory_slam/DorySLAMNode.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dory_slam_node");
    ros::NodeHandle nh;

    /* Create ishark object **/
   // auto *ishark = new shark_slam::iShark();

    /** Configure **/

    /** Initialize **/

    /** Subscribe to the topics **/

    /** Let ROS take over and hope for the best **/
    ros::spin();

   // delete ishark;

}
