#ifndef JET_WALKING_MOTION_HPP
#define JET_WALKING_MOTION_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <pcl/pcl_macros.h>
#include <cmath>
#include <random>
#include "sensor_msgs/msg/imu.hpp"

#undef DEG2RAD  // 기존 정의된 매크로 제거
#define DEG2RAD (M_PI / 180.0)

extern uint8_t dxl_error;
extern int dxl_comm_result;
extern dynamixel::PortHandler *portHandler;
extern dynamixel::PacketHandler *packetHandler;

class Motion : public rclcpp::Node {
public:
    Motion();  // 🔹 생성자 선언
    void executeMotion(const std_msgs::msg::String::SharedPtr msg);        // O

private:
    void accelCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);  // O
    void angVelCallback(const geometry_msgs::msg::Vector3::SharedPtr msg); // O
    void updateInitialState();                                             // O

    void forward();                                                        // X
    void backward();                                                       // X
    void left();                                                           // X
    void right();                                                          // X
    void squat();                                                          // O
    void circle();
    void standby();                                                        // O

    void pelvisToSupport(std::vector<int32_t> &present_positions);         // O
    void pelvisToSwing(std::vector<int32_t> &present_positions);           // O
    void generatePelvisTrajectory(int steps);
    void generateCircleTrajectory(int steps);
    // void getZMPTrajectory(std::vector<Eigen::Vector3d>& zmp_positions, int num_steps, double step_time);


    // void ZMPGenerator(std::vector<Eigen::Vector3d>& zmp_positions, int num_steps, double step_time);

    // void oneStepZMP(unsigned int current_step_number, Eigen::VectorXd& temp_px, Eigen::VectorXd& temp_py);

    // void getFootTrajectory();                                              // X
    // void getPelvisTrajectory();                                            // X
    void executeIK();                                                      // O
    void writePosition(std::vector<int32_t> &goal_positions);              // O
    void generateWalkingPattern();                                         // X
    void moveTo(double x, double y);                                     
    void stopCheck();                                                      // O

    void decideFirstFoot();                                                // O
    void torqueEnable(bool torque_flag);                                   // O

    float forward_distance = 0.2;
    float backward_distance = 0.2;
    float turn_angle = 10.0*DEG2RAD;

    bool is_right;
    bool is_aligned;
    int steps = 0;
    std::vector<int32_t> q_current = std::vector<int32_t>(12, 0);
    std::vector<double> q_desire = std::vector<double>(12, 0);
    std::vector<int32_t> q_desire_int = std::vector<int32_t>(12, 0);
    std::vector<float> pelvis_acc = {0, 0, 0};
    std::vector<float> pelvis_ang_vel = {0, 0, 0};
    Eigen::Isometry3d float_trunk_transform;
    Eigen::Isometry3d pelvis_support;
    Eigen::Isometry3d pelvis_swing;
    Eigen::Isometry3d pelvis_knee_support;
    Eigen::Isometry3d pelvis_knee_swing;
    Eigen::Isometry3d pelvis_support_floor;
    Eigen::Isometry3d pelvis_swing_floor;
    Eigen::Vector3d target_positions;
    float target_angle;
    Eigen::Vector3d zmp_positions;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr ang_vel_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr accel_subscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscriber;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motion_status_pub;
    
    // float hz_ = 50;
    // float t_double1_ = 0.10*hz_; 
    // float t_double2_ = 0.10*hz_;
    // float t_rest_init_ = 0.05*hz_;
    // float t_rest_last_ = 0.05*hz_;
    // float t_total_= 1.2*hz_;
    // float t_temp_ = 3.0*hz_;
    // float t_last_ = t_total_ + t_temp_ ; 
    // float t_start_ = t_temp_ + 1 ;
   
    // float t_start_real_ = t_start_ + t_rest_init_;
};

#endif  // JET_WALKING_MOTION_HPP
