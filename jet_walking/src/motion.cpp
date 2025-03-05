#include "rclcpp/rclcpp.hpp"
#include "jet_walking/motion.hpp"
#include "std_msgs/msg/string.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>
#include <time.h>


#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 562
#define ADDR_GOAL_POSITION 596
#define ADDR_PRESENT_POSITION 611

#define PROTOCOL_VERSION 2.0

#define BAUDRATE 3000000  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME_MOTOR "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

Motion::Motion() : Node("motion") {  
    RCLCPP_INFO(this->get_logger(), "Motion node Started");
    // 포트 핸들러 초기화
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME_MOTOR);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create port handler!");
        return;
    }
    
    if (!portHandler->openPort()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open port!");
        return;
    }

    if (!portHandler->setBaudRate(BAUDRATE)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set baudrate!");
        return;
    }
    accel_subscriber = this->create_subscription<geometry_msgs::msg::Vector3>(
        "imu_accel", 10,
        std::bind(&Motion::accelCallback, this, std::placeholders::_1));

    ang_vel_subscriber = this->create_subscription<geometry_msgs::msg::Vector3>(
        "imu_ang_vel", 10,
        std::bind(&Motion::angVelCallback, this, std::placeholders::_1));

    command_subscriber = this->create_subscription<std_msgs::msg::String>(
        "motion_command", 10,
        std::bind(&Motion::executeMotion, this, std::placeholders::_1));

    motion_status_pub = this->create_publisher<std_msgs::msg::String>("motion_status", 10);

    standby();
}

void Motion::accelCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    // 선형 가속도 데이터 저장
    pelvis_acc[0] = msg->x;
    pelvis_acc[1] = msg->y;
    pelvis_acc[2] = msg->z;
}
void Motion::angVelCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    // 각속도 데이터 저장
    pelvis_ang_vel[0] = msg->x;
    pelvis_ang_vel[1] = msg->y;
    pelvis_ang_vel[2] = msg->z;
}

void Motion::updateInitialState() {
    int32_t position = 0;
    for (int i = 15; i <= 26; i++) {
        int dxl_comm_result = packetHandler->ping(portHandler, i, &dxl_error);
        try {
            dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, i, 611, (uint32_t *)&position, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS) {
                throw std::runtime_error("Dynamixel communication failed");
            }
            RCLCPP_INFO(this->get_logger(), "Motor ID %d - Current Position: %d", i, position);
            q_current[i - 15] = position;  // ID 15~26 -> 벡터 index 0~11
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in updateInitialState: %s", e.what());
        }
    }
}

void Motion::executeMotion(const std_msgs::msg::String::SharedPtr msg) {  
    std::istringstream iss(msg->data);
    std::string command;
    int steps;
    iss >> command >> steps; // 방향과 걸음 수 분리

    RCLCPP_INFO(this->get_logger(), "Received command: '%s' with steps: %d", command.c_str(), steps);

    RCLCPP_INFO(this->get_logger(), "Entering walking mode..");

    auto status_msg = std::make_shared<std_msgs::msg::String>();
    status_msg->data = "running";
    motion_status_pub->publish(*status_msg);

    updateInitialState();
    torqueEnable(true);
    circle();
    // squat();
    // for(int i=0; i<=steps;i++){
    //     if (command == "forward") {
    //         RCLCPP_INFO(this->get_logger(), "Executing forward()");
    //         forward();
    //     } 
    //     else if (command == "backward") {
    //         RCLCPP_INFO(this->get_logger(), "Executing backward()");
    //         backward();
    //     }
    //     else if (command == "left") {
    //         RCLCPP_INFO(this->get_logger(), "Executing left()");
    //         left();
    //     }
    //     else if (command == "right") {
    //         RCLCPP_INFO(this->get_logger(), "Executing right()");
    //         right();
    //     }
    //     else if (command == "squat") {
    //         RCLCPP_INFO(this->get_logger(), "Executing squat()");
    //         squat();
    //         stopCheck();
    //     }
    //     else {
    //         RCLCPP_WARN(this->get_logger(), "Unknown command received: '%s'", command.c_str());
    //     }
    // }


    std::this_thread::sleep_for(std::chrono::seconds(1)); // 한 걸음당 1초 대기
    
    
    standby();

    status_msg->data = "idle";
    motion_status_pub->publish(*status_msg);
}


void Motion::forward() {
    RCLCPP_INFO(this->get_logger(), "Moving forward...");
    decideFirstFoot();
    //getZMPTrajectory(zmp_positions, num_steps, step_time);  // 올바른 호출
    pelvisToSupport(q_current);
    // target_positions = pelvis_support.translation() += pelvis_support.linear() * Eigen::Vector3d(forward_distance, 0, 0);
    // RCLCPP_INFO(this->get_logger(), "target position: %f, %f, %f",
    //             target_positions.x(),
    //             target_positions.y(),
    //             target_positions.z());
}

void Motion::backward() {
    RCLCPP_INFO(this->get_logger(), "Moving backward...");
    decideFirstFoot();
    pelvisToSupport(q_current);
    // target_positions = pelvis_support.translation() += pelvis_support.linear() * Eigen::Vector3d(-forward_distance, 0, 0);
    // RCLCPP_INFO(this->get_logger(), "target position: %f, %f, %f",
    //             target_positions.x(),
    //             target_positions.y(),
    //             target_positions.z());
}

void Motion::left() {
    RCLCPP_INFO(this->get_logger(), "Turning left...");
    is_right = false;
    RCLCPP_INFO(this->get_logger(), "Left foot is first.");
    pelvisToSupport(q_current);
    target_angle = turn_angle;
}

void Motion::right() {
    RCLCPP_INFO(this->get_logger(), "Turning right...");
    is_right = true;
    RCLCPP_INFO(this->get_logger(), "Right foot is first.");
    pelvisToSupport(q_current);
    target_angle = -turn_angle;
}

void Motion::squat() {
    sleep(5);
    RCLCPP_INFO(this->get_logger(), "Executing squat...");
    updateInitialState();
    pelvisToSupport(q_current);
    pelvisToSwing(q_current);
    RCLCPP_INFO(this->get_logger(), "Pelvis Support Position: (%.3f, %.3f, %.3f)", pelvis_support.translation().x(), pelvis_support.translation().y(), pelvis_support.translation().z());
    RCLCPP_INFO(this->get_logger(), "Pelvis Swing Position: (%.3f, %.3f, %.3f)", pelvis_swing.translation().x(), pelvis_swing.translation().y(), pelvis_swing.translation().z());
    
    generatePelvisTrajectory(100);
}

void Motion::circle() {
    is_right = 1;
    // RCLCPP_INFO(this->get_logger(), "Executing circle...");

    updateInitialState();
    pelvisToSupport(q_current);
    pelvisToSwing(q_current);
    generateCircleTrajectory(40);

}
void Motion::standby() {
    RCLCPP_INFO(this->get_logger(), "Entering standby mode...");
    updateInitialState();
    // torqueEnable(false);
}

void Motion::stopCheck() {
    float threshold_ang_vel = 0.1;
    float threshold_accel = 0.2 + 9.8;  // 움직임 감지 임계값
    rclcpp::Rate rate(2);  // 2Hz (500ms 주기)

    int stationary_count = 0;  // 정지 상태 카운트

    for (int i = 0; i < 11; i++) {
        float ang_vel_magnitude = sqrt(pow(pelvis_ang_vel[0], 2) + 
                                       pow(pelvis_ang_vel[1], 2) + 
                                       pow(pelvis_ang_vel[2], 2));
        float accel_magnitude = sqrt(pow(pelvis_acc[0], 2) + 
                                     pow(pelvis_acc[1], 2) + 
                                     pow(pelvis_acc[2], 2));

        if (accel_magnitude < threshold_accel && ang_vel_magnitude < threshold_ang_vel) {
            stationary_count++;  // 정지 상태 횟수 증가
            RCLCPP_INFO(this->get_logger(), "Robot is stationary. (%d/10)", stationary_count);

            // 10번 체크 전에 통과하면 바로 종료
            if (stationary_count >= 4) {
                RCLCPP_INFO(this->get_logger(), "Stable condition met. Proceeding to next step.");
                return;
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Robot is moving...");
            stationary_count = 0;  // 움직이면 다시 초기화
        }

        rate.sleep();  // 🔹 500ms 대기
    }

    // 11번째 반복에서 강제 종료
    RCLCPP_WARN(this->get_logger(), "Max checks reached. Moving to next step.");
}

void Motion::writePosition(std::vector<int32_t> &goal_positions) {
    for (int i = 15; i <= 26; i++) {
        int dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i, ADDR_GOAL_POSITION, goal_positions[i-15], &dxl_error);
    }
}

// vo


void Motion::torqueEnable(bool torque_flag) {
    for (int i = 15; i <= 26; i++) {
        int dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, torque_flag, &dxl_error);

        if (dxl_comm_result == COMM_SUCCESS && torque_flag == 0) {
            RCLCPP_INFO(this->get_logger(), "Motor ID %d, torque off.", i);
        }
        else if (dxl_comm_result == COMM_SUCCESS && torque_flag == 1) {
            RCLCPP_INFO(this->get_logger(), "Motor ID %d, torque on.", i);
        } 
        else {
            RCLCPP_ERROR(this->get_logger(), "Failed to torque off for motor ID %d", i);
        }
    }
}

void Motion::decideFirstFoot() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(0, 1);
    is_right = distrib(gen);
    if (is_right == 0) {
        RCLCPP_INFO(this->get_logger(), "Left foot is first.");
    } else {
        RCLCPP_INFO(this->get_logger(), "Right foot is first.");
    }
}

void Motion::pelvisToSupport(std::vector<int32_t> &present_positions) {
    pelvis_support = Eigen::Isometry3d::Identity();  // 기본값으로 초기화
    Eigen::Matrix3d rotation_matrix_support;
    std::vector<double> present_positions_rad(present_positions.size());

    for (size_t i = 0; i < present_positions.size(); i++) {
        present_positions_rad[i] = (static_cast<double>(present_positions[i]) / 250961.0) * M_PI;
    }
    if(is_right == 1) {
        pelvis_support.translation() += Eigen::Vector3d(-0.04, 0.105, -0.20);

        Eigen::AngleAxisd rotation_hip_yaw(-present_positions_rad[1], Eigen::Vector3d::UnitZ());
        pelvis_support.rotate(rotation_hip_yaw);  
    
        Eigen::AngleAxisd rotation_hip_roll(present_positions_rad[3], Eigen::Vector3d::UnitX());
        pelvis_support.rotate(rotation_hip_roll);  
    
        Eigen::AngleAxisd rotation_hip_pitch(present_positions_rad[5], Eigen::Vector3d::UnitY());
        pelvis_support.rotate(rotation_hip_pitch);  
    
        pelvis_support.translation() += pelvis_support.linear() * Eigen::Vector3d(0.1515, 0, -0.339);
        
        Eigen::AngleAxisd rotation_knee_pitch(present_positions_rad[7], Eigen::Vector3d::UnitY());
        pelvis_support.rotate(rotation_knee_pitch);
    
        pelvis_support.translation() += pelvis_support.linear() * Eigen::Vector3d(0.06, 0, -0.368);
        
        Eigen::AngleAxisd rotation_ankle_pitch(present_positions_rad[9], Eigen::Vector3d::UnitY());
        pelvis_support.rotate(rotation_ankle_pitch);
    
        Eigen::AngleAxisd rotation_ankle_roll(present_positions_rad[11], Eigen::Vector3d::UnitX());
        pelvis_support.rotate(rotation_ankle_roll);

        // pelvis_support.translation() += pelvis_support.linear() * Eigen::Vector3d(0, 0, -0.075);
        
    }
    else {
        pelvis_support.translation() += Eigen::Vector3d(-0.04, -0.105, -0.20);

        Eigen::AngleAxisd rotation_hip_yaw(-present_positions_rad[0], Eigen::Vector3d::UnitZ());
        pelvis_support.rotate(rotation_hip_yaw);  
    
        Eigen::AngleAxisd rotation_hip_roll(present_positions_rad[2], Eigen::Vector3d::UnitX());
        pelvis_support.rotate(rotation_hip_roll);  
    
        Eigen::AngleAxisd rotation_hip_pitch(-present_positions_rad[4], Eigen::Vector3d::UnitY());
        pelvis_support.rotate(rotation_hip_pitch);  
    
        pelvis_support.translation() += pelvis_support.linear() * Eigen::Vector3d(0.1515, 0, -0.339);
        
        Eigen::AngleAxisd rotation_knee_pitch(-present_positions_rad[6], Eigen::Vector3d::UnitY());
        pelvis_support.rotate(rotation_knee_pitch);
    
        pelvis_support.translation() += pelvis_support.linear() * Eigen::Vector3d(0.06, 0, -0.368);
        
        Eigen::AngleAxisd rotation_ankle_pitch(-present_positions_rad[8], Eigen::Vector3d::UnitY());
        pelvis_support.rotate(rotation_ankle_pitch);
    
        Eigen::AngleAxisd rotation_ankle_roll(present_positions_rad[10], Eigen::Vector3d::UnitX());
        pelvis_support.rotate(rotation_ankle_roll);
        
        // pelvis_support.translation() += pelvis_support.linear() * Eigen::Vector3d(0, 0, -0.075);
    }
    RCLCPP_INFO(this->get_logger(), "Pelvis Support Position: (%.3f, %.3f, %.3f)", pelvis_support.translation().x(), pelvis_support.translation().y(), pelvis_support.translation().z());
    
}

void Motion::pelvisToSwing(std::vector<int32_t> &present_positions) {
    pelvis_swing = Eigen::Isometry3d::Identity();  // 기본값으로 초기화
    Eigen::Matrix3d rotation_matrix_swing;
    std::vector<double> present_positions_rad(present_positions.size());

    for (size_t i = 0; i < present_positions.size(); i++) {
        present_positions_rad[i] = (static_cast<double>(present_positions[i]) / 250961.0) * M_PI;
    }

    if(is_right == 0) {  // 왼쪽 발이 서포트 -> 오른쪽 발이 스윙
        pelvis_swing.translation() += Eigen::Vector3d(-0.04, 0.105, -0.20);

        Eigen::AngleAxisd rotation_hip_yaw(-present_positions_rad[1], Eigen::Vector3d::UnitZ());
        pelvis_swing.rotate(rotation_hip_yaw);  
    
        Eigen::AngleAxisd rotation_hip_roll(present_positions_rad[3], Eigen::Vector3d::UnitX());
        pelvis_swing.rotate(rotation_hip_roll);  
    
        Eigen::AngleAxisd rotation_hip_pitch(present_positions_rad[5], Eigen::Vector3d::UnitY());
        pelvis_swing.rotate(rotation_hip_pitch);  
    
        pelvis_swing.translation() += pelvis_swing.linear() * Eigen::Vector3d(0.1515, 0, -0.339);
        
        Eigen::AngleAxisd rotation_knee_pitch(present_positions_rad[7], Eigen::Vector3d::UnitY());
        pelvis_swing.rotate(rotation_knee_pitch);
    
        pelvis_swing.translation() += pelvis_swing.linear() * Eigen::Vector3d(0.06, 0, -0.368);
        
        Eigen::AngleAxisd rotation_ankle_pitch(present_positions_rad[9], Eigen::Vector3d::UnitY());
        pelvis_swing.rotate(rotation_ankle_pitch);
    
        Eigen::AngleAxisd rotation_ankle_roll(present_positions_rad[11], Eigen::Vector3d::UnitX());
        pelvis_swing.rotate(rotation_ankle_roll);

        // pelvis_swing.translation() += pelvis_swing.linear() * Eigen::Vector3d(0, 0, -0.075);
    }
    else {  // 오른쪽 발이 서포트 -> 왼쪽 발이 스윙
        pelvis_swing.translation() += Eigen::Vector3d(-0.04, -0.105, -0.20);

        Eigen::AngleAxisd rotation_hip_yaw(-present_positions_rad[0], Eigen::Vector3d::UnitZ());
        pelvis_swing.rotate(rotation_hip_yaw);  
    
        Eigen::AngleAxisd rotation_hip_roll(present_positions_rad[2], Eigen::Vector3d::UnitX());
        pelvis_swing.rotate(rotation_hip_roll);  
    
        Eigen::AngleAxisd rotation_hip_pitch(-present_positions_rad[4], Eigen::Vector3d::UnitY());
        pelvis_swing.rotate(rotation_hip_pitch);  
    
        pelvis_swing.translation() += pelvis_swing.linear() * Eigen::Vector3d(0.1515, 0, -0.339);
        
        Eigen::AngleAxisd rotation_knee_pitch(-present_positions_rad[6], Eigen::Vector3d::UnitY());
        pelvis_swing.rotate(rotation_knee_pitch);
    
        pelvis_swing.translation() += pelvis_swing.linear() * Eigen::Vector3d(0.06, 0, -0.368);

        Eigen::AngleAxisd rotation_ankle_pitch(-present_positions_rad[8], Eigen::Vector3d::UnitY());
        pelvis_swing.rotate(rotation_ankle_pitch);
    
        Eigen::AngleAxisd rotation_ankle_roll(present_positions_rad[10], Eigen::Vector3d::UnitX());
        pelvis_swing.rotate(rotation_ankle_roll);
        
        // pelvis_swing.translation() += pelvis_swing.linear() * Eigen::Vector3d(0, 0, -0.075);
    }

    // 디버깅용 로그 추가
    RCLCPP_INFO(this->get_logger(), "Pelvis Swing Position: (%.3f, %.3f, %.3f)", 
                pelvis_swing.translation().x(), pelvis_swing.translation().y(), pelvis_swing.translation().z());
}

void Motion::generatePelvisTrajectory(int steps) {  // ✅ 올바른 함수 정의
    RCLCPP_INFO(this->get_logger(), "Generating cubic spline pelvis trajectory...");

    float initial_height = (pelvis_support.inverse().translation().z() + pelvis_swing.inverse().translation().z()) / 2;
    float final_height = initial_height - 0.4;

    Eigen::VectorXd heights(steps);
    RCLCPP_INFO(this->get_logger(), "hi 000");


    // ▶ **내려가는 구간** (스쿼트)
    for (int i = 0; i < steps; i++) {
        double t = static_cast<double>(i) / (steps - 1);
        heights(i) = (1 - 3 * t * t + 2 * t * t * t) * initial_height + (3 * t * t - 2 * t * t * t) * final_height;
    }

    RCLCPP_INFO(this->get_logger(), "hi 111");

    for (int i = 0; i < steps; i++) {
        clock_t start1 = clock(); 
        double height = heights(i);

        // pelvis_support 업데이트
        Eigen::Isometry3d support_pelvis = pelvis_support.inverse();
    
        Eigen::Vector3d new_translation_support = support_pelvis.translation();
        new_translation_support.z() = height;
        support_pelvis.translation() = new_translation_support;
        
        pelvis_support = support_pelvis.inverse();
    
        // 🔹 pelvis_swing 업데이트
        Eigen::Isometry3d swing_pelvis = pelvis_swing.inverse();
    
        Eigen::Vector3d new_translation_swing = swing_pelvis.translation();
        new_translation_swing.z() = height;
        swing_pelvis.translation() = new_translation_swing;
    
        pelvis_swing = swing_pelvis.inverse();

        executeIK();
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
        clock_t end1 = clock();
        RCLCPP_INFO(this->get_logger(), "end1: (%.3f)", (double)(start1 - end1));
        // printf("시작 시간 : %f, 종료 시간 :%f\n",((float)start1) / CLOCKS_PER_SEC, ((float)end1) / CLOCKS_PER_SEC); 
        
    }

    // ▶ **올라오는 구간**
    for (int i = 0; i < steps; i++) {
        double t = static_cast<double>(i) / (steps - 1);
        heights(i) = (1 - 3 * t * t + 2 * t * t * t) * final_height + (3 * t * t - 2 * t * t * t) * initial_height;
    }

    for (int i = 0; i < steps; i++) {
        double height = heights(i);

        // pelvis_support 업데이트
        Eigen::Isometry3d support_pelvis = pelvis_support.inverse();
    
        Eigen::Vector3d new_translation_support = support_pelvis.translation();
        new_translation_support.z() = height;
        support_pelvis.translation() = new_translation_support;
        
        pelvis_support = support_pelvis.inverse();
    
        // 🔹 pelvis_swing 업데이트
        Eigen::Isometry3d swing_pelvis = pelvis_swing.inverse();
    
        Eigen::Vector3d new_translation_swing = swing_pelvis.translation();
        new_translation_swing.z() = height;
        swing_pelvis.translation() = new_translation_swing;
    
        pelvis_swing = swing_pelvis.inverse();

        executeIK();
        // std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    RCLCPP_INFO(this->get_logger(), "Squat motion completed.");
}

void Motion::generateCircleTrajectory(int steps) {
    // RCLCPP_INFO(this->get_logger(), "Generating cubic spline pelvis trajectory...");
    double r = 0.15;
    float initial_height_l = (pelvis_support.translation().z() + pelvis_swing.translation().z()) / 2;
    float final_height_l = initial_height_l - r; 
    // RCLCPP_INFO(this->get_logger(), "finalheight: (%.3f)", final_height_l);
    float initial_height_r = (pelvis_support.translation().z() + pelvis_swing.translation().z()) / 2;
    float final_height_r = initial_height_r + r; 
    // RCLCPP_INFO(this->get_logger(), "finalheight: (%.3f)", final_height_r);
    Eigen::Vector3d origin_l = pelvis_support.translation();
    Eigen::Vector3d origin_r = pelvis_swing.translation();

    // RCLCPP_INFO(this->get_logger(), "left origin: (%.3f, %.3f, %.3f)", 
    // origin_l.x(), origin_l.y(), origin_l.z());
    // RCLCPP_INFO(this->get_logger(), "right origin: (%.3f, %.3f, %.3f)", 
    // origin_r.x(), origin_r.y(), origin_r.z());

    Eigen::VectorXd heights_l(steps);
    Eigen::VectorXd heights_r(steps);

    Eigen::Vector3d new_translation_support = pelvis_support.translation();
    Eigen::Vector3d new_translation_swing = pelvis_swing.translation();

    for (int i = 0; i < 10; i++) {
        double t = static_cast<double>(i) / (10 - 1);
        heights_l(i) = (1 - t) * initial_height_l + (t) * final_height_l;
        heights_r(i) = (1 - t) * initial_height_r + (t) * final_height_r;
        new_translation_support.z() = heights_l(i);
        pelvis_support.translation() = new_translation_support;
        new_translation_swing.z() = heights_r(i);
        pelvis_swing.translation() = new_translation_swing;
        executeIK();
    }

    for (int i = 0; i < steps; i++) {
        double theta = static_cast<double>(i) / (steps - 1)*5*M_PI;
        pelvis_support.translation() = origin_l + Eigen::Vector3d(-r*sin(theta), 0, -r*cos(theta));
        pelvis_swing.translation() = origin_r + Eigen::Vector3d(r*sin(theta), 0, r*cos(theta));
        // RCLCPP_INFO(this->get_logger(), "pelvis_support: (%.3f, %.3f, %.3f)", 
        // pelvis_support.translation().x(), pelvis_support.translation().y(), pelvis_support.translation().z());
        // RCLCPP_INFO(this->get_logger(), "pelvis_swing: (%.3f, %.3f, %.3f)", 
        // pelvis_swing.translation().x(), pelvis_swing.translation().y(), pelvis_swing.translation().z());
        executeIK();
    }

    for (int i = 0; i < 10; i++) {
        double t = static_cast<double>(i) / (10 - 1);
        heights_l(i) = (1 - t) * final_height_l + (t) * initial_height_l;
        heights_r(i) = (1 - t) * final_height_r + (t) * initial_height_r;
        new_translation_support.z() = heights_l(i);
        pelvis_support.translation() = new_translation_support;
        new_translation_swing.z() = heights_r(i);
        pelvis_swing.translation() = new_translation_swing;
        executeIK();
    }
}


void Motion::executeIK() {
    Eigen::Isometry3d float_trunk_transform = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d float_lleg_transform;
    Eigen::Isometry3d float_rleg_transform;

    if (is_right == 1) {
        float_lleg_transform = pelvis_support;
        float_rleg_transform = pelvis_swing;
    } else {
        float_lleg_transform = pelvis_swing;
        float_rleg_transform = pelvis_support;
    }

    Eigen::Vector3d R_r, R_D, L_r, L_D;
    L_D << -0.04, +0.105, -0.20;
    R_D << -0.04, -0.105, -0.20;

    double R_C = 0, L_C = 0, L_upper = 0.3713, L_lower = 0.3728, R_alpha = 0, L_alpha = 0;

    Eigen::Isometry3d pelvis_lhip = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d pelvis_rhip = Eigen::Isometry3d::Identity();
    
    pelvis_lhip.translation() = L_D;
    pelvis_rhip.translation() = R_D;

    Eigen::Isometry3d lhip_lleg = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d rhip_rleg = Eigen::Isometry3d::Identity();

    lhip_lleg = pelvis_lhip.inverse() * float_lleg_transform;
    rhip_rleg = pelvis_rhip.inverse() * float_rleg_transform;

    L_r = lhip_lleg.translation();
    R_r = rhip_rleg.translation();

    L_C = sqrt(pow(L_r(0), 2) + pow(L_r(1), 2) + pow(L_r(2), 2));
    R_C = sqrt(pow(R_r(0), 2) + pow(R_r(1), 2) + pow(R_r(2), 2));

    q_desire[6] = (-acos((pow(L_upper, 2) + pow(L_lower, 2) - pow(R_C, 2)) / (2 * L_upper * L_lower)) + M_PI); // Knee pitch (Right)
    q_desire[7] = (-acos((pow(L_upper, 2) + pow(L_lower, 2) - pow(L_C, 2)) / (2 * L_upper * L_lower)) + M_PI); // Knee pitch (Left)

    L_alpha = asin(L_upper / L_C * sin(M_PI - q_desire[7]));
    R_alpha = asin(L_upper / R_C * sin(M_PI - q_desire[6]));

    q_desire[8] = -atan2(R_r(0), sqrt(pow(R_r(1), 2) + pow(R_r(2), 2))) - R_alpha; // Ankle Pitch (Right)
    q_desire[9] = -atan2(L_r(0), sqrt(pow(L_r(1), 2) + pow(L_r(2), 2))) - L_alpha; // Ankle Pitch (Left)

    Eigen::Matrix3d R_Knee_Ankle_Y_rot_mat = Eigen::AngleAxisd(-q_desire[6] - q_desire[8], Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix3d L_Knee_Ankle_Y_rot_mat = Eigen::AngleAxisd(-q_desire[7] - q_desire[9], Eigen::Vector3d::UnitY()).toRotationMatrix();

    Eigen::Matrix3d R_Ankle_X_rot_mat = Eigen::AngleAxisd(-q_desire[10], Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Matrix3d L_Ankle_X_rot_mat = Eigen::AngleAxisd(-q_desire[11], Eigen::Vector3d::UnitX()).toRotationMatrix();

    Eigen::Matrix3d R_Hip_rot_mat = float_trunk_transform.rotation().transpose() * float_rleg_transform.rotation() * R_Ankle_X_rot_mat * R_Knee_Ankle_Y_rot_mat;
    Eigen::Matrix3d L_Hip_rot_mat = float_trunk_transform.rotation().transpose() * float_lleg_transform.rotation() * L_Ankle_X_rot_mat * L_Knee_Ankle_Y_rot_mat;
    q_desire[6] = -q_desire[6]+0.2578;
    q_desire[7] = q_desire[7]-0.2578;
    q_desire[8] = -q_desire[8]+0.0063;
    q_desire[9] = q_desire[9]-0.0063;

    q_desire[0] = -atan2(-R_Hip_rot_mat(0, 1), R_Hip_rot_mat(1, 1)); // Hip Yaw (Right)
    q_desire[2] = atan2(R_Hip_rot_mat(2, 1), -R_Hip_rot_mat(0, 1) * sin(q_desire[0]) + R_Hip_rot_mat(1, 1) * cos(q_desire[0])); // Hip Roll (Right)
    q_desire[4] = -atan2(-R_Hip_rot_mat(2, 0), R_Hip_rot_mat(2, 2)) - 0.2651; // Hip Pitch (Right)

    if (R_r(1) < 0){
        q_desire[10] = -M_PI/2 - atan2(R_r(2),R_r(1));
    }
    else {
        q_desire[10] = M_PI/2 + atan2(R_r(2),-R_r(1));
    }

    q_desire[1] = -atan2(-L_Hip_rot_mat(0, 1), L_Hip_rot_mat(1, 1)); // Hip Yaw (Left)
    q_desire[3] = atan2(L_Hip_rot_mat(2, 1), -L_Hip_rot_mat(0, 1) * sin(q_desire[1]) + L_Hip_rot_mat(1, 1) * cos(q_desire[1])); // Hip Roll (Left)
    q_desire[5] = atan2(-L_Hip_rot_mat(2, 0), L_Hip_rot_mat(2, 2)) + 0.2651;// Hip Pitch (Left)
    q_desire[11] = 0;
    if (L_r(1) < 0){
        q_desire[11] = -M_PI/2 - atan2(L_r(2),L_r(1));
    }
    else {
        q_desire[11] = M_PI/2 + atan2(L_r(2),-L_r(1));
    }


    std::vector<int32_t> q_desire_int(12, 0);

    for (size_t i = 0; i < q_desire.size(); i++) {
        q_desire_int[i] = static_cast<int32_t>((q_desire[i] / M_PI) * 250961);

    }
    
    writePosition(q_desire_int);
}


// void Motion::executeIK() {
//     Eigen::Isometry3d float_trunk_transform = Eigen::Isometry3d::Identity();
//     Eigen::Isometry3d float_lleg_transform;
//     Eigen::Isometry3d float_rleg_transform;

//     if (is_right == 1) {
//         float_lleg_transform = pelvis_support;
//         float_rleg_transform = pelvis_swing;
//     } else {
//         float_rleg_transform = pelvis_swing;
//         float_lleg_transform = pelvis_support;
//     }

//     double offset_hip_pitch = 24.0799945102 * DEG2RAD;
//     double offset_knee_pitch = 14.8197729791 * DEG2RAD;
//     double offset_ankle_pitch = 9.2602215311 * DEG2RAD;

//     Eigen::Vector3d R_r, R_D, L_r, L_D;
//     L_D << -0.04, +0.105, -0.20;
//     R_D << -0.04, -0.105, -0.20;

//     double R_C = 0, L_C = 0, L_upper = 0.3713, L_lower = 0.3728 , R_alpha = 0, L_alpha = 0;

//     Eigen::Isometry3d pelvis_lhip = Eigen::Isometry3d::Identity();
//     Eigen::Isometry3d pelvis_rhip = Eigen::Isometry3d::Identity();
    
//     pelvis_lhip.translation() = L_D;
//     pelvis_rhip.translation() = R_D;

//     Eigen::Isometry3d lhip_lleg = Eigen::Isometry3d::Identity();
//     Eigen::Isometry3d rhip_rleg = Eigen::Isometry3d::Identity();

//     lhip_lleg = float_lleg_transform.inverse() * pelvis_lhip;
//     rhip_rleg = float_rleg_transform.inverse() * pelvis_rhip;

//     L_r = lhip_lleg.translation();
//     R_r = rhip_rleg.translation();

//     RCLCPP_INFO(this->get_logger(), "L_r: (%.3f, %3f, %.3f)", L_r.x(), L_r.y(), L_r.z());
//     RCLCPP_INFO(this->get_logger(), "R_r: (%.3f, %.3f, %.write), 2));

//     q_desire[7] = (-acos((pow(L_upper, 2) + pow(L_lower, 2) - pow(L_C, 2)) / (2 * L_upper * L_lower)) + M_PI);
//     q_desire[6] = (-acos((pow(L_upper, 2) + pow(L_lower, 2) - pow(R_C, 2)) / (2 * L_upper * L_lower)) + M_PI);

//     L_alpha = asin(L_upper / L_C * sin(M_PI - q_desire[3]));
//     R_alpha = asin(L_upper / R_C * sin(M_PI - q_desire[9]));

//     q_desire[4] = -atan2(L_r(0), sqrt(pow(L_r(1), 2) + pow(L_r(2), 2))) - asin(0.3713 / L_C * sin(M_PI - q_desire[3]));
//     q_desire[10] = -atan2(R_r(0), sqrt(pow(R_r(1), 2) + pow(R_r(2), 2))) - asin(0.3713 / R_C * sin(M_PI - q_desire[9]));

//     Eigen::Matrix3d L_Knee_Ankle_Y_rot_mat = Eigen::AngleAxisd(-q_desire[3]- q_desire[4], Eigen::Vector3d::UnitY()).toRotationMatrix();
//     Eigen::Matrix3d L_Ankle_X_rot_mat = Eigen::AngleAxisd(-q_desire[5], Eigen::Vector3d::UnitX()).toRotationMatrix();
//     Eigen::Matrix3d R_Knee_Ankle_Y_rot_mat = Eigen::AngleAxisd(-q_desire[9] - q_desire[10], Eigen::Vector3d::UnitY()).toRotationMatrix();
//     Eigen::Matrix3d R_Ankle_X_rot_mat = Eigen::AngleAxisd(-q_desire[11], Eigen::Vector3d::UnitX()).toRotationMatrix();

//     Eigen::Matrix3d L_Hip_rot_mat = float_trunk_transform.rotation().transpose() * float_lleg_transform.rotation() * L_Ankle_X_rot_mat * L_Knee_Ankle_Y_rot_mat;
//     Eigen::Matrix3d R_Hip_rot_mat = float_trunk_transform.rotation().transpose() * float_rleg_transform.rotation() * R_Ankle_X_rot_mat * R_Knee_Ankle_Y_rot_mat;

//     // 왼쪽 다리 IK 계산
//     q_desire[0] = -atan2(-L_Hip_rot_mat(0, 1), L_Hip_rot_mat(1, 1)); // Hip yaw
//     q_desire[1] = atan2(L_Hip_rot_mat(2, 1), -L_Hip_rot_mat(0, 1) * sin(q_desire[0]) + L_Hip_rot_mat(1, 1) * cos(q_desire[0])); // Hip roll
//     q_desire[2] = atan2(-L_Hip_rot_mat(2, 0), L_Hip_rot_mat(2, 2)) + offset_hip_pitch; // Hip pitch
//     q_desire[5] = atan2(L_r(1), L_r(2));  // Ankle roll

//     // 오른쪽 다리 IK 계산
//     q_desire[6] = -atan2(-R_Hip_rot_mat(0, 1), R_Hip_rot_mat(1, 1)); // Hip yaw
//     q_desire[7] = atan2(R_Hip_rot_mat(2, 1), -R_Hip_rot_mat(0, 1) * sin(q_desire[6]) + R_Hip_rot_mat(1, 1) * cos(q_desire[6])); // Hip roll
//     q_desire[8] = -atan2(-R_Hip_rot_mat(2, 0), R_Hip_rot_mat(2, 2)) - offset_hip_pitch; // Hip pitch
//     q_desire[9] = -q_desire[9];
//     q_desire[10] = -q_desire[10];
//     q_desire[11] = atan2(R_r(1), R_r(2)); // Ankle roll

// }


    // 🔹 최종 pelvis_support의 위치 확인 (디버깅용)
    // RCLCPP_INFO(this->get_logger(), "Pelvis-swing position: %f, %f, %f", 
    //             pelvis_support.translation().x(),
    //             pelvis_support.translation().y(),
    //             pelvis_support.translation().z());

    // Eigen::Matrix3d rotation_matrix = pelvis_support.rotation();

    // RCLCPP_INFO(this->get_logger(), "Rotation Matrix:");
    // RCLCPP_INFO(this->get_logger(), "[%f, %f, %f]",
    //             rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2));
    // RCLCPP_INFO(this->get_logger(), "[%f, %f, %f]",
    //             rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2));
    // RCLCPP_INFO(this->get_logger(), "[%f, %f, %f]",
    //             rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2));
    // for (int i = 0; i <= 11; i++){
    //     RCLCPP_INFO(this->get_logger(), "q_current[%d] = %d", i, q_current[i]);  


// void Motion::generateWalkingPattern() {
//     RCLCPP_INFO(this->get_logger(), "Generating Walking Pattern...");

//     getZMPTrajectory(); // ZMP 궤적 생성

//     for (int i = 0; i < steps; i++) {writepositions.y()); // ZMP 목표 위치로 이동
//         std::this_thread::sleep_for(std::chrono::milliseconds(500));
//     }

//     stopCheck(); // 안정성 확인
// }

// void Motion::getZMPTrajectory(std::vector<Eigen::Vector3d>& zmp_positions, int num_steps, double step_time) {
//     RCLCPP_INFO(this->get_logger(), "Generating ZMP trajectory...");

//     int num_tick = step_time * hz; // 한 스텝당 Tick 개수
//     zmp_positions.clear();

//     for (int step = 0; step < num_steps; step++) {
//         double x_start = step * forward_distance; // 이전 스텝의 끝점이 시작점
//         double x_end = (step + 1) * forward_distance; // 다음 스텝의 목표 ZMP 위치
//         double y_start = (step % 2 == 0) ? -0.105 : 0.105; // 좌우 번갈아가며 발 위치
//         double y_end = (step % 2 == 0) ? 0.105 : -0.105; // 다음 스텝의 목표 발 위치

//         for (int i = 0; i <= num_tick; i++) {
//             double t = i / static_cast<double>(num_tick);
//             double x = x_start + t * (x_end - x_start); // 선형 보간
//             double y = y_start + 0.5 * (y_end - y_start) * (1 - cos(M_PI * t)); // 부드러운 이동
//             double z = 0.0; // ZMP는 항상 지면에 위치

//             zmp_positions.push_back(Eigen::Vector3d(x, y, z));
//         }
//     }

//     RCLCPP_INFO(this->get_logger(), "ZMP trajectory generated with %lu points", zmp_positions.size());
// }


// void Motion::moveTo(double x, double y) {
//     RCLCPP_INFO(this->get_logger(), "Moving to target position: (%f, %f)", x, y);
//     // 여기에 모터 제어 코드 추가 (예: Dynamixel에 목표 위치 명령)