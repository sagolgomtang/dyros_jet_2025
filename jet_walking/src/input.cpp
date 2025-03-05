#include "jet_walking/input.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <sstream>

Input::Input() : Node("input"), running_(true), motion_running_(false) {
    RCLCPP_INFO(this->get_logger(), "Input node started");

    // 🔹 퍼블리셔 생성 (input_command 토픽 발행)
    input_publisher = this->create_publisher<std_msgs::msg::String>("input_command", 10);

    // 🔹 모션 상태 구독 (motion_status 토픽)
    motion_subscriber = this->create_subscription<std_msgs::msg::String>(
        "motion_status", 10,
        std::bind(&Input::motionCallback, this, std::placeholders::_1));

    // 🔹 키 입력 감지 스레드 실행
    input_thread_ = std::thread(&Input::inputLoop, this);
}

Input::~Input() {
    running_ = false;
    if (input_thread_.joinable()) {
        input_thread_.join();
    }
}

// 🔹 터미널 설정 복구
void restoreTerminalSettings(struct termios &oldt) {
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

// 🔹 비동기 입력 감지
void Input::inputLoop() {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    RCLCPP_INFO(this->get_logger(), "Listening for input (WASD to move, Q to quit)...");

    while (running_) {
        char key = getchar();
        std_msgs::msg::String msg;
        std::string direction = "";
        switch (key) {
            case 'w': case 'W':
                RCLCPP_INFO(this->get_logger(), "Moving Forward");
                msg.data = "forward";
                break;
            case 's': case 'S':
                RCLCPP_INFO(this->get_logger(), "Moving Backward");
                msg.data = "backward";
                break;
            case 'a': case 'A':
                RCLCPP_INFO(this->get_logger(), "Turning Left");
                msg.data = "left";
                break;
            case 'd': case 'D':
                RCLCPP_INFO(this->get_logger(), "Turning Right");
                msg.data = "right";
                break;
            case 'e': case 'E':
                RCLCPP_INFO(this->get_logger(), "Squat");
                msg.data = "squat";
                break;
            case 'q': case 'Q':
                RCLCPP_INFO(this->get_logger(), "Quitting all nodes...");
                msg.data = "quit";
                input_publisher->publish(msg);
                restoreTerminalSettings(oldt);
            default:
                RCLCPP_INFO(this->get_logger(), "Invalid input");
                continue;
        }
        RCLCPP_INFO(this->get_logger(), "Select step count (1-9) or press C to cancel...");

        while (true) {
            char step_key = getchar(); // 🔹 숫자 입력받기

            if (step_key == 'q' || step_key == 'Q') {
                RCLCPP_INFO(this->get_logger(), "Quitting all nodes...");
                msg.data = "quit";
                input_publisher->publish(msg);
                restoreTerminalSettings(oldt);
                return;  // 🔹 프로그램 즉시 종료
            }
            if (step_key == 'c' || step_key == 'C') {
                RCLCPP_INFO(this->get_logger(), "Listening for input (WASD to move, Q to quit)...");
                break; // 🔹 다시 방향 선택으로 돌아감
            }

            if (step_key >= '1' && step_key <= '9') {
                int steps = step_key - '0'; // 🔹 문자 '1'-'9'를 숫자로 변환

                // 🔹 방향 + 스텝 수 메시지 생성
                std::ostringstream oss;
                oss << direction << " " << steps;
                msg.data = oss.str();
                
                input_publisher->publish(msg);  // 🔹 메시지 발행
                RCLCPP_INFO(this->get_logger(), "Moving %s for %d steps.", direction.c_str(), steps);
                break; // 입력 완료 후 반복 종료
            }

            RCLCPP_WARN(this->get_logger(), "Invalid step count! Press 1-9 or C to cancel.");
        }
    }

    restoreTerminalSettings(oldt);
}

void Input::motionCallback(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data == "running") {
        motion_running_ = true; // 🔹 모션 실행 중
    } else if (msg->data == "idle") {
        motion_running_ = false; // 🔹 모션 종료됨
    }
}