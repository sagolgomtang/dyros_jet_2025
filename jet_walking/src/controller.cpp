#include "jet_walking/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <chrono>
#include <thread>


#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 562
#define ADDR_GOAL_POSITION 596
#define ADDR_PRESENT_POSITION 611

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE 3000000  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME_MOTOR "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"
#define COMM_SUCCESS 0

uint8_t dxl_error = 0;
uint32_t goal_position = 0;
int dxl_comm_result = COMM_TX_FAIL;

dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME_MOTOR);
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

Controller::Controller() : Node("controller")
{
    RCLCPP_INFO(this->get_logger(), "Controller Node Started");
    motor_flag = false;

    // 🔹 Dynamixel 모터 초기화
    scanMotors();
    // if (false) {
    if (motor_flag == false) {
        RCLCPP_ERROR(this->get_logger(), "No Dynamixel motors detected! Shutting down...");
        sleep(3);
        shutDown();
        std::exit(1);
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Dynamixel motors detected.");
        setInitialConfiguration();
    }

    // 🔹 motion, sensor, input 실행
    motion_node = std::make_shared<Motion>();
    input_node = std::make_shared<Input>();

    executor.add_node(motion_node);
    executor.add_node(input_node);

    input_subscriber = this->create_subscription<std_msgs::msg::String>(
        "input_command", 10, 
        std::bind(&Controller::inputCallback, this, std::placeholders::_1));
    
    // 🔹 별도 스레드에서 실행
    executor_thread = std::thread([this]() { executor.spin(); });
}

Controller::~Controller()
{
    input_subscriber.reset();
    executor.cancel();

    if (executor_thread.joinable()) {
        executor_thread.join();
    }

    rclcpp::shutdown();
    std::exit(0);
}

// 🔹 Dynamixel 모터 초기화 (ID 검색 + 현재 위치 확인)
void Controller::scanMotors() {
    if (!portHandler->openPort())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open port!");
        return;
    }
    if (!portHandler->setBaudRate(BAUDRATE))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set baudrate!");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Scanning for connected Dynamixel motors...");

    for (int id = 15; id <= 26; id++)
    {
        
        int dxl_comm_result = packetHandler->ping(portHandler, id, &dxl_error);

        if (dxl_comm_result == COMM_SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Found Dynamixel ID: %d", id);
            motor_flag = true;

            // 🔹 현재 위치 읽기
            int32_t position = 0;
            int32_t velocity = 0;
            int32_t acceleration = 0;
            dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, id, 611, (uint32_t *)&position, &dxl_error);
            if (dxl_comm_result == COMM_SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Motor ID %d - Current Position: %d", id, position);
            }
            else {
                RCLCPP_WARN(this->get_logger(), "Failed to get position for ID %d", id);
            }
            packetHandler->write4ByteTxRx(portHandler, id, 32, 1000, &dxl_error);
            packetHandler->read4ByteTxRx(portHandler, id, 32, (uint32_t *)&velocity, &dxl_error);
            RCLCPP_INFO(this->get_logger(), "Motor ID %d - Goal velocity: %d", id, velocity);
            packetHandler->write4ByteTxRx(portHandler, id, 26, 50, &dxl_error);
            packetHandler->read4ByteTxRx(portHandler, id, 26, (uint32_t *)&acceleration, &dxl_error);
            RCLCPP_INFO(this->get_logger(), "Motor ID %d - Goal acceleration: %d", id, acceleration);
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_OPERATING_MODE, 3, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set Position Control Mode for ID %d", id);
            }
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Failed to enable torque for ID %d", id);
            }
        }
    }
}

void Controller::setInitialConfiguration() {
    sleep(5);
    RCLCPP_INFO(this->get_logger(), "Setting initial configuration...");
    std::map<int, int> initial_positions = {
            // {15, 0}, {16, 0}, {17, 0}, {18, 0}, {19, 36600}, {20, -36600},
            // {21, -112000}, {22, 112000}, {23, 75400}, {24, -75400},
            // {25, 0}, {26, 0}};

        {15, 0}, {16, 0}, {17, 0}, {18, 0}, {19, 0}, {20, 0},
        {21, -34000}, {22, 34000}, {23, 34000}, {24, -34000},
        {25, 0}, {26, 0}};

    //     {15, 54000}, {16, -54000}, {17, 57700}, {18, 57700}, {19, 20000}, {20, -20000},
    //     {21, -34000}, {22, 34000}, {23, 34000}, {24, -34000},
    //     {25, 0}, {26, 0}};

        double transition_time = 2.0; // 목표 위치까지 이동하는 시간 (초)
        int steps = 2;  // 몇 단계로 나눠서 움직일지 (값이 클수록 더 천천히 이동)
        double step_time = transition_time / steps; // 각 스텝당 대기 시간

        for (int i = 0; i <= steps; i++) {
            for (const auto &[id, goal_position] : initial_positions)
            {
                int current_position;
                packetHandler->read4ByteTxRx(portHandler, id, ADDR_PRESENT_POSITION,
                                             reinterpret_cast<uint32_t *>(&current_position), &dxl_error);

                int interpolated_position = current_position + (goal_position - current_position) * i / steps;
                packetHandler->write4ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, interpolated_position, &dxl_error);
            }

            // rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(step_time * 1000))); // 일정 시간 대기
        }
        RCLCPP_INFO(this->get_logger(), "Setting initial configuration is done!");
        RCLCPP_INFO(this->get_logger(), "Entering standby mode...");

        portHandler->closePort();
}

void Controller::inputCallback(const std_msgs::msg::String::SharedPtr msg) {
    //RCLCPP_INFO(this->get_logger(), "Received Input: %s", msg->data.c_str());

    if (msg->data == "quit") {
        shutDown();
    }  
    else {
        auto command_msg = std::make_shared<std_msgs::msg::String>();
        command_msg->data = msg->data;

        motion_node->executeMotion(command_msg); // 🔹 Motion 실행
    }
}

void Controller::disableMotor() {
    RCLCPP_INFO(this->get_logger(), "Disabling all Dynamixel motors...");

    if (!portHandler->openPort()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open port!");
        return;
    }
    if (!portHandler->setBaudRate(BAUDRATE)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set baudrate!");
        return;
    }

    for (int id = 15; id <= 26; id++) {
        int dxl_comm_result = packetHandler->ping(portHandler, id, &dxl_error);

        if (dxl_comm_result == COMM_SUCCESS)
        {
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 0, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to enable torque for ID %d", id);
            }
        }
    }
    RCLCPP_INFO(this->get_logger(), "All Dynamixel motors disabled.");
    portHandler->closePort();
}

void Controller::shutDown() {
    RCLCPP_INFO(this->get_logger(), "Shutting down Controller...");
    disableMotor();
    system("pkill -f /sensor");
    RCLCPP_INFO(this->get_logger(), "Controller fully shut down.");
    rclcpp::shutdown();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto controller = std::make_shared<Controller>();
    rclcpp::Rate rate(1);

    // rclcpp::spin(controller);
    while (rclcpp::ok())
    {
        // RCLCPP_INFO(node->get_logger(), "Running at 10Hz...");

        // ROS 2 노드 실행
        rclcpp::spin_some(controller);

        // 🔹 지정된 주기만큼 대기
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}