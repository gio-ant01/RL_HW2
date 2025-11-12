#include <functional>
#include <memory>
#include <thread>
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "ros2_kdl_package/action/linear_trajectory.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class KDLActionServer : public rclcpp::Node
{
public:
    using LinearTrajectory = ros2_kdl_package::action::LinearTrajectory;
    using GoalHandleLinearTrajectory = rclcpp_action::ServerGoalHandle<LinearTrajectory>;

    explicit KDLActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("ros2_kdl_action_server", options)
    {
        using namespace std::placeholders;

        // Declare cmd_interface parameter (position, velocity)
        declare_parameter("cmd_interface", "velocity");
        get_parameter("cmd_interface", cmd_interface_);
        RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());

        if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort" ))
        {
            RCLCPP_ERROR(get_logger(),"Selected cmd interface is not valid! Use 'position', 'velocity' or 'effort' instead..."); 
            return;
        }

        // Declare trajectory parameters (defaults)
        declare_parameter("total_time", 1.5);
        get_parameter("total_time", total_time_);
        
        declare_parameter("trajectory_len", 150);
        get_parameter("trajectory_len", trajectory_len_);
        
        declare_parameter("Kp", 5);
        get_parameter("Kp", Kp_);

        declare_parameter("ctrl", "velocity_ctrl");
        get_parameter("ctrl", ctrl_type_);
        RCLCPP_INFO(get_logger(),"Current velocity controller type is: '%s'", ctrl_type_.c_str());

        declare_parameter("lambda", 0.5);
        get_parameter("lambda", lambda_);

        joint_state_available_ = false;

        // Retrieve robot_description param
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
            std::shared_ptr<KDLActionServer>(this, [](auto*){}), "robot_state_publisher");
        
        while (!parameters_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                rclcpp::shutdown();
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        auto parameter = parameters_client->get_parameters({"robot_description"});

        // Create KDLrobot structure
        KDL::Tree robot_tree;
        if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
            RCLCPP_ERROR(get_logger(), "Failed to retrieve robot_description param!");
        }
        robot_ = std::make_shared<KDLRobot>(robot_tree);  
        
        // Create joint array
        unsigned int nj = robot_->getNrJnts();
        KDL::JntArray q_min(nj), q_max(nj);
        q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96;
        q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96;     
        robot_->setJntLimits(q_min,q_max);            
        joint_positions_.resize(nj); 
        joint_velocities_.resize(nj); 
        joint_positions_cmd_.resize(nj); 
        joint_velocities_cmd_.resize(nj); 
        joint_efforts_cmd_.resize(nj); 
        joint_efforts_cmd_.data.setZero();

        // Subscriber to jnt states
        jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&KDLActionServer::joint_state_subscriber, this, _1));

        // Wait for the joint_state topic
        while(!joint_state_available_){
            RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
            rclcpp::spin_some(this->get_node_base_interface());
        }

        // Update KDLrobot object
        robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
        KDL::Frame f_T_ee = KDL::Frame::Identity();
        robot_->addEE(f_T_ee);
        robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

        // Compute EE frame
        init_cart_pose_ = robot_->getEEFrame();

        // Compute IK
        KDL::JntArray q(nj);
        robot_->getInverseKinematics(init_cart_pose_, q);

        // Initialize controller
        controller_ = std::make_shared<KDLController>(*robot_);

        // Create cmd publisher based on interface
        if(cmd_interface_ == "position"){
            cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
        }
        else if(cmd_interface_ == "velocity"){
            cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
        }
        else if(cmd_interface_ == "effort"){
            cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
        }

        // Create action server
        this->action_server_ = rclcpp_action::create_server<LinearTrajectory>(
            this,
            "linear_trajectory",
            std::bind(&KDLActionServer::handle_goal, this, _1, _2),
            std::bind(&KDLActionServer::handle_cancel, this, _1),
            std::bind(&KDLActionServer::handle_accepted, this, _1));

        RCLCPP_INFO(this->get_logger(), "Action server ready!");
    }

private:
    rclcpp_action::Server<LinearTrajectory>::SharedPtr action_server_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
    rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;

    std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    KDL::JntArray joint_positions_;
    KDL::JntArray joint_velocities_;
    KDL::JntArray joint_positions_cmd_;
    KDL::JntArray joint_velocities_cmd_;
    KDL::JntArray joint_efforts_cmd_;

    std::shared_ptr<KDLRobot> robot_;
    std::shared_ptr<KDLController> controller_;

    bool joint_state_available_;
    std::string cmd_interface_;
    std::string ctrl_type_;
    KDL::Frame init_cart_pose_;
    
    double total_time_;
    int trajectory_len_;
    int Kp_;
    double lambda_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const LinearTrajectory::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        RCLCPP_INFO(this->get_logger(), "  traj_type: %s, s_type: %s", 
                    goal->traj_type.c_str(), goal->s_type.c_str());
        RCLCPP_INFO(this->get_logger(), "  duration: %.2f, acc_duration: %.2f", 
                    goal->traj_duration, goal->acc_duration);
        RCLCPP_INFO(this->get_logger(), "  end_position: [%.3f, %.3f, %.3f]",
                    goal->end_position_x, goal->end_position_y, goal->end_position_z);
        
        (void)uuid;

        // Validate goal parameters
        if (goal->traj_duration <= 0.0 || goal->acc_duration <= 0.0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid durations!");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (goal->traj_type != "linear" && goal->traj_type != "circular") {
            RCLCPP_ERROR(this->get_logger(), "Invalid trajectory type!");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (goal->s_type != "trapezoidal" && goal->s_type != "cubic") {
            RCLCPP_ERROR(this->get_logger(), "Invalid s type!");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleLinearTrajectory> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleLinearTrajectory> goal_handle)
    {
        using namespace std::placeholders;
        // This needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&KDLActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleLinearTrajectory> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing trajectory...");
        
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<LinearTrajectory::Feedback>();
        auto result = std::make_shared<LinearTrajectory::Result>();

        // Extract goal parameters
        double traj_duration = goal->traj_duration;
        double acc_duration = goal->acc_duration;
        std::string traj_type = goal->traj_type;
        std::string s_type = goal->s_type;
        Eigen::Vector3d end_position(goal->end_position_x, goal->end_position_y, goal->end_position_z);

        // EE's trajectory initial position (offset)
        Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0,0,0.1));

        // Use end_position from goal if non-zero, otherwise use default
        if (end_position.norm() < 1e-6) {
            end_position << init_position[0], -init_position[1], init_position[2];
            RCLCPP_INFO(get_logger(),"Using default end position (mirrored y): [%.3f, %.3f, %.3f]",
                        end_position[0], end_position[1], end_position[2]);
        }

        // Create planner
        KDLPlanner planner;
        double traj_radius = 0.15;
        
        if(traj_type == "linear"){
            planner = KDLPlanner(traj_duration, acc_duration, init_position, end_position);
        } 
        else if(traj_type == "circular")
        {
            planner = KDLPlanner(traj_duration, init_position, traj_radius, acc_duration);
        }

        // Trajectory execution
        int loop_rate = trajectory_len_ / total_time_;
        double dt = 1.0 / loop_rate;
        rclcpp::Rate rate(loop_rate);
        
        double t = 0.0;
        double final_error_norm = 0.0;

        while (t < total_time_ && rclcpp::ok())
        {
            // Check if there is a cancel request
            if (goal_handle->is_canceling()) {
                result->success = false;
                result->final_error_norm = final_error_norm;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                
                // Stop the robot
                stop_robot();
                return;
            }

            // Get trajectory point
            trajectory_point p;
            if(traj_type == "linear"){
                if(s_type == "trapezoidal")
                    p = planner.linear_traj_trapezoidal(t);
                else if(s_type == "cubic")
                    p = planner.linear_traj_cubic(t);
            } 
            else if(traj_type == "circular")
            {
                if(s_type == "trapezoidal")
                    p = planner.circular_traj_trapezoidal(t);
                else if(s_type == "cubic")
                    p = planner.circular_traj_cubic(t);
            }

            // Compute EE frame
            KDL::Frame cartpos = robot_->getEEFrame();

            // Compute errors
            Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data));
            Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));
            
            final_error_norm = error.norm();

            // Publish feedback
            feedback->error_x = error[0];
            feedback->error_y = error[1];
            feedback->error_z = error[2];
            feedback->error_norm = final_error_norm;
            feedback->time_elapsed = t;
            goal_handle->publish_feedback(feedback);

            // Compute commands based on interface
            if(cmd_interface_ == "position"){
                KDL::Frame nextFrame; 
                nextFrame.M = cartpos.M; 
                nextFrame.p = cartpos.p + (toKDL(p.vel) + toKDL(Kp_*error))*dt;
                joint_positions_cmd_ = joint_positions_;
                robot_->getInverseKinematics(nextFrame, joint_positions_cmd_);
                
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_cmd_(i);
                }
            }
            else if(cmd_interface_ == "velocity"){
                if(ctrl_type_ == "velocity_ctrl"){
                    Vector6d cartvel; 
                    cartvel << p.vel + Kp_*error, o_error;
                    joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                }
                else if(ctrl_type_ == "velocity_ctrl_null"){
                    Eigen::Vector3d pos_d = p.pos;
                    Eigen::Vector3d pos_e = Eigen::Vector3d(cartpos.p.data);
                    joint_velocities_cmd_.data = controller_->velocity_ctrl_null(pos_d, pos_e, Kp_, lambda_);
                }
                
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_cmd_(i);
                }
            }
            else if(cmd_interface_ == "effort"){
                joint_efforts_cmd_.data[0] = 0.1*std::sin(2*M_PI*t/total_time_);
                
                for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                    desired_commands_[i] = joint_efforts_cmd_(i);
                }
            }

            // Update robot and publish commands
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

            t += dt;
            rate.sleep();
        }

        // Check if goal is done
        if (rclcpp::ok()) {
            result->success = true;
            result->final_error_norm = final_error_norm;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded! Final error: %.6f", final_error_norm);
            
            // Stop the robot
            stop_robot();
        }
    }

    void stop_robot()
    {
        // Send zero velocities
        std::vector<double> zero_commands(7, 0.0);
        std_msgs::msg::Float64MultiArray cmd_msg;
        cmd_msg.data = zero_commands;
        cmdPublisher_->publish(cmd_msg);
        RCLCPP_INFO(this->get_logger(), "Robot stopped");
    }

    void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg)
    {
        joint_state_available_ = true;
        for (unsigned int i = 0; i < sensor_msg.position.size(); i++){
            joint_positions_.data[i] = sensor_msg.position[i];
            joint_velocities_.data[i] = sensor_msg.velocity[i];
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KDLActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}