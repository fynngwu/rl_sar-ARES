#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <iostream>
#include <iomanip>
#include <atomic>
#include <cmath>
#include <string>

static std::atomic<int> motor_count{0};
static std::atomic<int> imu_count{0};
static bool got_motor = false;
static bool got_imu = false;
static int print_n = 10;

static float motor_pos[12] = {};
static float motor_vel[12] = {};
static float imu_gyro[3] = {};
static float imu_gravity[3] = {};
static std::string motor_names[12] = {};

static void print_joints()
{
    std::cout << "\n--- Motor Feedback (joint " << motor_count.load() << ") ---" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "  idx  name          pos(rad)    vel(rad/s)" << std::endl;
    std::cout << "  ---  ----------  ----------  ----------" << std::endl;
    for (int i = 0; i < 12; ++i)
    {
        std::cout << "  " << std::setw(3) << i << "  "
                  << std::setw(10) << std::left << motor_names[i] << std::right
                  << "  " << std::setw(10) << motor_pos[i]
                  << "  " << std::setw(10) << motor_vel[i]
                  << std::endl;
    }
}

static void print_imu()
{
    std::cout << "\n--- IMU Data (sample " << imu_count.load() << ") ---" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "  gyro (body, rad/s):  [" << imu_gyro[0] << ", " << imu_gyro[1] << ", " << imu_gyro[2] << "]" << std::endl;
    std::cout << "  gravity (body):      [" << imu_gravity[0] << ", " << imu_gravity[1] << ", " << imu_gravity[2] << "]" << std::endl;
    float norm = std::sqrt(imu_gravity[0]*imu_gravity[0] + imu_gravity[1]*imu_gravity[1] + imu_gravity[2]*imu_gravity[2]);
    std::cout << "  |gravity|:           " << norm << std::endl;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::cout << "=== ARES Driver Feedback Test ===" << std::endl;
    std::cout << "Subscribing to /motor_feedback and /imu/data ..." << std::endl;
    std::cout << "Will print " << print_n << " samples of each, then exit." << std::endl;

    auto node = std::make_shared<rclcpp::Node>("test_driver_feedback");

    auto motor_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/motor_feedback", 10,
        [&](const sensor_msgs::msg::JointState::SharedPtr msg) {
            int n = std::min((int)msg->position.size(), 12);
            for (int i = 0; i < n; ++i) {
                if (i < (int)msg->name.size())
                    motor_names[i] = msg->name[i];
                motor_pos[i] = msg->position[i];
                motor_vel[i] = msg->velocity[i];
            }
            motor_count++;
            if (motor_count <= print_n || motor_count == print_n + 1) {
                print_joints();
                if (motor_count == print_n)
                    std::cout << "(further motor messages suppressed)" << std::endl;
            }
        });

    auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 10,
        [&](const sensor_msgs::msg::Imu::SharedPtr msg) {
            imu_gyro[0] = msg->angular_velocity.x;
            imu_gyro[1] = msg->angular_velocity.y;
            imu_gyro[2] = msg->angular_velocity.z;
            imu_gravity[0] = msg->linear_acceleration.x;
            imu_gravity[1] = msg->linear_acceleration.y;
            imu_gravity[2] = msg->linear_acceleration.z;
            imu_count++;
            if (imu_count <= print_n || imu_count == print_n + 1) {
                print_imu();
                if (imu_count == print_n)
                    std::cout << "(further IMU messages suppressed)" << std::endl;
            }
        });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
        executor.spin_some();

        bool have_both = (motor_count.load() > 0 && imu_count.load() > 0);
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start).count();

        if (have_both && elapsed >= 3) {
            std::cout << "\n========================================" << std::endl;
            std::cout << "Received " << motor_count.load() << " motor, "
                      << imu_count.load() << " IMU messages in " << elapsed << "s" << std::endl;

            if (motor_count.load() >= 3) {
                std::cout << "Motor feedback rate: ~" << motor_count.load() / elapsed << " Hz" << std::endl;
                std::cout << "Joint names received: [";
                for (int i = 0; i < 12; ++i) {
                    if (i > 0) std::cout << ", ";
                    std::cout << motor_names[i];
                }
                std::cout << "]" << std::endl;
            }
            if (imu_count.load() >= 3) {
                std::cout << "IMU rate: ~" << imu_count.load() / elapsed << " Hz" << std::endl;
            }
            std::cout << "========================================" << std::endl;
            break;
        }

        if (elapsed >= 10) {
            std::cerr << "\nTIMEOUT: Did not receive both motor and IMU within 10s" << std::endl;
            std::cerr << "  motor messages: " << motor_count.load() << std::endl;
            std::cerr << "  IMU messages:   " << imu_count.load() << std::endl;
            std::cerr << "Make sure ares_driver_node is running!" << std::endl;
            rclcpp::shutdown();
            return 1;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    rclcpp::shutdown();
    return 0;
}
