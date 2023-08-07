#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

ros::Publisher cmd_vel_pub;
double left_threshold;
double right_threshold;
double y_velocity;
double x_velocity;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  float fMidDist = msg->ranges[450];
  float fMidDist1 = msg->ranges[180];
  ROS_INFO("后方测距 range[0] = %f m, 左方距离 range[90] = %f m", fMidDist, fMidDist1);
    // if (left_distance > left_threshold) {
    //     // 发送Y轴速度
    //     geometry_msgs::Twist velocity;
    //     velocity.linear.y = y_velocity;
    //     cmd_vel_pub.publish(velocity);
    // } else if (right_distance > right_threshold) {
    //     // 发送Y轴速度
    //     geometry_msgs::Twist velocity;
    //     velocity.linear.y = -y_velocity;
    //     cmd_vel_pub.publish(velocity);
    // } else {
    //     // 发送X轴速度
    //     geometry_msgs::Twist velocity;
    //     velocity.linear.x = x_velocity;
    //     cmd_vel_pub.publish(velocity);
    // }
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "ladar_distance_controller");
    ros::NodeHandle nh;
    
    nh.param<double>("left_threshold", left_threshold, 0.3);
    nh.param<double>("right_threshold", right_threshold, 0.3);
    nh.param<double>("y_velocity", y_velocity, 0.2);
    nh.param<double>("x_velocity", x_velocity, 0.2);

    ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, laserCallback);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    ros::spin();

    return 0;
}
