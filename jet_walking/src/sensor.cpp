#include "jet_walking/sensor.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cstdlib>
#include "sensor_msgs/msg/imu.hpp"
#include <Eigen/Dense>

Sensor::Sensor() : Node("sensor") {  // 🔹 부모 클래스 Node의 생성자 명확히 호출
    RCLCPP_INFO(this->get_logger(), "Sensor Node Started");

    imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data_raw", 10, 
        std::bind(&Sensor::sensorCallback, this, std::placeholders::_1));

    accel_publisher = this->create_publisher<geometry_msgs::msg::Vector3>("imu_accel", 10);
    ang_vel_publisher = this->create_publisher<geometry_msgs::msg::Vector3>("imu_ang_vel", 10);  // 🔹 추가

}

Sensor::~Sensor() {
    RCLCPP_INFO(this->get_logger(), "Sensor Node Shut Down");
}

Eigen::Vector3d Sensor::directionChange(Eigen::Vector3d &direction) {
    Eigen::Matrix3d imu_pelvis;
    imu_pelvis <<  0,  0,  -1,
                   0,  1,  0,
                   -1,  0,  0;
    return imu_pelvis * direction;
}

void Sensor::sensorCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // IMU의 각속도 데이터를 로봇 좌표계로 변환
    Eigen::Vector3d imu_ang_vel(msg->angular_velocity.x, 
                                msg->angular_velocity.y, 
                                msg->angular_velocity.z);
    Eigen::Vector3d robot_ang_vel = directionChange(imu_ang_vel);

    RCLCPP_INFO(this->get_logger(), "Angular Velocity - x: %f, y: %f, z: %f",
    robot_ang_vel.x(), robot_ang_vel.y(), robot_ang_vel.z());

    // IMU의 선형 가속도 데이터를 로봇 좌표계로 변환
    Eigen::Vector3d imu_accel(msg->linear_acceleration.x, 
                              msg->linear_acceleration.y, 
                              msg->linear_acceleration.z);
    Eigen::Vector3d robot_accel = directionChange(imu_accel);

    RCLCPP_INFO(this->get_logger(), "Linear Acceleration - x: %f, y: %f, z: %f",
    robot_accel.x(), robot_accel.y(), robot_accel.z());

    // ⚡ 가속도 데이터 메시지 생성 후 퍼블리시
    auto accel_msg = geometry_msgs::msg::Vector3();
    accel_msg.x = robot_accel.x();
    accel_msg.y = robot_accel.y();
    accel_msg.z = robot_accel.z();
    accel_publisher->publish(accel_msg);  // ✅ 가속도 퍼블리시

    // ⚡ 각속도 데이터 메시지 생성 후 퍼블리시
    auto ang_vel_msg = geometry_msgs::msg::Vector3();
    ang_vel_msg.x = robot_ang_vel.x();
    ang_vel_msg.y = robot_ang_vel.y();
    ang_vel_msg.z = robot_ang_vel.z();
    ang_vel_publisher->publish(ang_vel_msg);  // ✅ 각속도 퍼블리시
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto sensor_node = std::make_shared<Sensor>();
    rclcpp::spin(sensor_node);
    rclcpp::shutdown();
    return 0;
}

