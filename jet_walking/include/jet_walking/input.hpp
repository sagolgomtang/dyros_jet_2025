#ifndef JET_WALKING_INPUT_HPP
#define JET_WALKING_INPUT_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <thread>
#include <atomic>

class Input : public rclcpp::Node {
public:
    Input();
    ~Input();
    void startInputListener();  // 🔹 키보드 입력 감지 시작

private:
    void inputLoop();
    void motionCallback(const std_msgs::msg::String::SharedPtr msg);
    std::atomic<bool> running_;
    std::atomic<bool> motion_running_;
    std::thread input_thread_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr input_publisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motion_subscriber;
};

#endif  // JET_WALKING_INPUT_HPP
