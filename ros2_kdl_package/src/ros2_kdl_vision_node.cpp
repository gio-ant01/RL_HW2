#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class VisionControlNode : public rclcpp::Node
{
public:
    VisionControlNode()
    : Node("ros2_kdl_vision_node"), 
      node_handle_(std::shared_ptr<VisionControlNode>(this))
    {
        declare_parameter("cmd_interface", "velocity");
        get_parameter("cmd_interface", cmd_interface_);
        RCLCPP_INFO(get_logger(), "cmd_interface: '%s'", cmd_interface_.c_str());

        if (cmd_interface_ != "velocity") {
            RCLCPP_ERROR(get_logger(), "Vision control requires velocity interface!");
            return;
        }

        declare_parameter("K_vision", 0.5);
        get_parameter("K_vision", K_vision_);
        RCLCPP_INFO(get_logger(), "K_vision = %.3f", K_vision_);

        declare_parameter("lambda", 0.5);
        get_parameter("lambda", lambda_);
        RCLCPP_INFO(get_logger(), "lambda = %.3f", lambda_);

        joint_state_available_ = false;
        aruco_pose_available_ = false;
        first_marker_seen_ = false;
        
        // Camera offset: xyz="0.0 0.0 0.15" from link_7
        camera_offset_position_ << 0.0, 0.0, 0.15;
        
        // Option 1: Use your empirical rotation [0,1,0; 0,0,-1; -1,0,0]
        // This is EE to optical frame
        R_ee_to_optical_ << 0,  1,  0,
                            0,  0, -1,
                           -1,  0,  0;
        
        RCLCPP_INFO(get_logger(), "Using empirical EE→optical rotation:");
        RCLCPP_INFO(get_logger(), "[%.1f, %.1f, %.1f]", R_ee_to_optical_(0,0), R_ee_to_optical_(0,1), R_ee_to_optical_(0,2));
        RCLCPP_INFO(get_logger(), "[%.1f, %.1f, %.1f]", R_ee_to_optical_(1,0), R_ee_to_optical_(1,1), R_ee_to_optical_(1,2));
        RCLCPP_INFO(get_logger(), "[%.1f, %.1f, %.1f]", R_ee_to_optical_(2,0), R_ee_to_optical_(2,1), R_ee_to_optical_(2,2));
        
        // Retrieve robot_description
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
            node_handle_, "robot_state_publisher");
        
        while (!parameters_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(get_logger(), "Interrupted waiting for service.");
                rclcpp::shutdown();
            }
            RCLCPP_INFO(get_logger(), "Waiting for robot_state_publisher...");
        }
        
        auto parameter = parameters_client->get_parameters({"robot_description"});

        KDL::Tree robot_tree;
        if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
            RCLCPP_ERROR(get_logger(), "Failed to parse robot_description!");
            return;
        }
        
        robot_ = std::make_shared<KDLRobot>(robot_tree);  
        
        unsigned int nj = robot_->getNrJnts();
        KDL::JntArray q_min(nj), q_max(nj);
        q_min.data << -2.96, -2.09, -2.96, -2.09, -2.96, -2.09, -2.96;
        q_max.data <<  2.96,  2.09,  2.96,  2.09,  2.96,  2.09,  2.96;
        robot_->setJntLimits(q_min, q_max);
        
        joint_positions_.resize(nj); 
        joint_velocities_.resize(nj);
        joint_velocities_cmd_.resize(nj);

        jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, 
            std::bind(&VisionControlNode::joint_state_callback, this, std::placeholders::_1));

        arucoPoseSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_single/pose", 100,
            std::bind(&VisionControlNode::aruco_pose_callback, this, std::placeholders::_1));

        while(!joint_state_available_){
            RCLCPP_INFO(get_logger(), "Waiting for joint_states...");
            rclcpp::spin_some(node_handle_);
        }

        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
        
        KDL::Frame f_T_ee = KDL::Frame::Identity();
        robot_->addEE(f_T_ee);
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

        controller_ = std::make_shared<KDLController>(*robot_);

        cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), 
            std::bind(&VisionControlNode::control_loop, this));

        RCLCPP_INFO(get_logger(), "==============================================");
        RCLCPP_INFO(get_logger(), "Vision control ready! Waiting for ArUco...");
        RCLCPP_INFO(get_logger(), "==============================================");
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState& sensor_msg)
    {
        joint_state_available_ = true;
        for (unsigned int i = 0; i < sensor_msg.position.size(); i++){
            joint_positions_.data[i] = sensor_msg.position[i];
            joint_velocities_.data[i] = sensor_msg.velocity[i];
        }
    }

    void aruco_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!first_marker_seen_) {
            RCLCPP_INFO(get_logger(), "✓ ArUco marker detected!");
            first_marker_seen_ = true;
        }
        
        aruco_pose_available_ = true;
        
        c_Po_optical_ << msg->pose.position.x, 
                         msg->pose.position.y, 
                         msg->pose.position.z;
    }

    Eigen::MatrixXd computeCameraJacobian(const Eigen::MatrixXd& J_ee, 
                                          const Eigen::Matrix3d& R_ee)
    {
        int n = J_ee.cols();
        Eigen::MatrixXd J_optical(6, n);
        
        // Position offset in world frame
        Eigen::Vector3d r_world = R_ee * camera_offset_position_;
        
        // Skew-symmetric matrix
        Eigen::Matrix3d S_r;
        S_r <<           0, -r_world(2),  r_world(1),
               r_world(2),            0, -r_world(0),
              -r_world(1),  r_world(0),            0;
        
        // v_optical = v_ee - [r]× w_ee
        J_optical.block(0, 0, 3, n) = J_ee.block(0, 0, 3, n) - S_r * J_ee.block(3, 0, 3, n);
        
        // w_optical = w_ee (same angular velocity)
        J_optical.block(3, 0, 3, n) = J_ee.block(3, 0, 3, n);
        
        return J_optical;
    }

    void control_loop()
    {
        if (!aruco_pose_available_) {
            // No marker: send zeros
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data.resize(robot_->getNrJnts(), 0.0);
            cmdPublisher_->publish(cmd_msg);
            return;
        }

        // Update robot
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

        // Get EE Jacobian and pose
        Eigen::MatrixXd J_ee = robot_->getEEJacobian().data;
        KDL::Frame ee_frame = robot_->getEEFrame();
        Eigen::Matrix3d R_ee = toEigen(ee_frame.M);
        
        // Compute optical frame Jacobian
        Eigen::MatrixXd J_optical = computeCameraJacobian(J_ee, R_ee);
        
        // Optical frame rotation in world
        Eigen::Matrix3d R_optical_world = R_ee * R_ee_to_optical_;

        // Desired s: object along +Z of optical frame
        Eigen::Vector3d s_d(0.0, 0.0, 1.0);
        
        // Current s
        double norm_Po = c_Po_optical_.norm();
        Eigen::Vector3d s_current = (norm_Po > 1e-6) ? (c_Po_optical_ / norm_Po) : Eigen::Vector3d(0,0,1);
        
        // Error
        Eigen::Vector3d s_error = s_d - s_current;

        // Call vision controller
        joint_velocities_cmd_.data = controller_->vision_ctrl(
            c_Po_optical_,
            R_optical_world,
            J_optical,
            s_d,
            K_vision_,
            lambda_
        );

        // Publish
        std_msgs::msg::Float64MultiArray cmd_msg;
        cmd_msg.data.resize(joint_velocities_cmd_.data.size());
        for (long int i = 0; i < joint_velocities_cmd_.data.size(); ++i) {
            cmd_msg.data[i] = joint_velocities_cmd_(i);
        }
        cmdPublisher_->publish(cmd_msg);

        // DEBUG LOG (1 Hz)
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, 
            "c_Po=[%.3f,%.3f,%.3f] |Po|=%.3f | s_cur=[%.2f,%.2f,%.2f] s_d=[%.2f,%.2f,%.2f] | err=%.3f | cmd[0]=%.3f",
            c_Po_optical_(0), c_Po_optical_(1), c_Po_optical_(2), norm_Po,
            s_current(0), s_current(1), s_current(2),
            s_d(0), s_d(1), s_d(2),
            s_error.norm(), cmd_msg.data[0]);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arucoPoseSubscriber_;
    rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Node::SharedPtr node_handle_;

    std::shared_ptr<KDLRobot> robot_;
    std::shared_ptr<KDLController> controller_;
    
    KDL::JntArray joint_positions_;
    KDL::JntArray joint_velocities_;
    KDL::JntArray joint_velocities_cmd_;

    Eigen::Vector3d camera_offset_position_;
    Eigen::Matrix3d R_ee_to_optical_;
    Eigen::Vector3d c_Po_optical_;

    std::string cmd_interface_;
    double K_vision_;
    double lambda_;
    
    bool joint_state_available_;
    bool aruco_pose_available_;
    bool first_marker_seen_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionControlNode>());
    rclcpp::shutdown();
    return 0;
}
