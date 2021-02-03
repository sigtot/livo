#include <ros/ros.h>

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "orb_test");
    ros::NodeHandle nh;
    cout << "Hello world" << endl;
    return 0;
}
