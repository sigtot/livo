#include <ros/ros.h>
#include <sensor_msgs/Image.h>

using namespace std;

void imageCallback(const sensor_msgs::Image::ConstPtr& img) {
    ROS_INFO("Recvd img\n");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "orb_test");
    ros::NodeHandle nh;

    auto sub = nh.subscribe("/camera/image_mono", 1000, imageCallback);

    ros::spin();
    return 0;
}
