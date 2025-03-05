#ifndef JET_WALKING_Sensor_HPP
#define JET_WALKING_Sensor_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <Eigen/Dense>

class Sensor : public rclcpp::Node {
public:
    Sensor();  //  생성자 선언
    ~Sensor();  // 소멸자 선언

private:
    Eigen::Vector3d directionChange(Eigen::Vector3d &direction);
    void sensorCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr accel_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr ang_vel_publisher;
};

#endif  
