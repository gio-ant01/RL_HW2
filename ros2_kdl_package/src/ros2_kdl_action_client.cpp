#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "ros2_kdl_package/action/linear_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace ros2_kdl_package
{
class LinearTrajectoryActionClient : public rclcpp::Node
{
public:
    using LinearTrajectory = ros2_kdl_package::action::LinearTrajectory;
    using GoalHandleLinearTrajectory = rclcpp_action::ClientGoalHandle<LinearTrajectory>;

    explicit LinearTrajectoryActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("linear_trajectory_action_client", options)
    {
        // Declare parameters with default values
        this->declare_parameter("traj_duration", 2.0);
        this->declare_parameter("acc_duration", 0.5);
        this->declare_parameter("end_position_x", 0.5);
        this->declare_parameter("end_position_y", -0.3);
        this->declare_parameter("end_position_z", 0.4);
        this->declare_parameter("traj_type", "linear");
        this->declare_parameter("s_type", "trapezoidal");

        // Get parameters
        this->get_parameter("traj_duration", traj_duration_);
        this->get_parameter("acc_duration", acc_duration_);
        this->get_parameter("end_position_x", end_position_x_);
        this->get_parameter("end_position_y", end_position_y_);
        this->get_parameter("end_position_z", end_position_z_);
        this->get_parameter("traj_type", traj_type_);
        this->get_parameter("s_type", s_type_);

        RCLCPP_INFO(this->get_logger(), "Client parameters:");
        RCLCPP_INFO(this->get_logger(), "  traj_duration: %.2f", traj_duration_);
        RCLCPP_INFO(this->get_logger(), "  acc_duration: %.2f", acc_duration_);
        RCLCPP_INFO(this->get_logger(), "  end_position: [%.3f, %.3f, %.3f]", 
                    end_position_x_, end_position_y_, end_position_z_);
        RCLCPP_INFO(this->get_logger(), "  traj_type: %s", traj_type_.c_str());
        RCLCPP_INFO(this->get_logger(), "  s_type: %s", s_type_.c_str());

        this->client_ptr_ = rclcpp_action::create_client<LinearTrajectory>(
            this,
            "linear_trajectory");

        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&LinearTrajectoryActionClient::send_goal, this));
    }

    void send_goal()
    {
        using namespace std::placeholders;

        this->timer_->cancel();

        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            return;
        }

        auto goal_msg = LinearTrajectory::Goal();
        goal_msg.traj_duration = traj_duration_;
        goal_msg.acc_duration = acc_duration_;
        goal_msg.end_position_x = end_position_x_;
        goal_msg.end_position_y = end_position_y_;
        goal_msg.end_position_z = end_position_z_;
        goal_msg.traj_type = traj_type_;
        goal_msg.s_type = s_type_;

        RCLCPP_INFO(this->get_logger(), "Sending goal to action server...");

        auto send_goal_options = rclcpp_action::Client<LinearTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&LinearTrajectoryActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&LinearTrajectoryActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&LinearTrajectoryActionClient::result_callback, this, _1);
        
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<LinearTrajectory>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Goal parameters
    double traj_duration_;
    double acc_duration_;
    double end_position_x_;
    double end_position_y_;
    double end_position_z_;
    std::string traj_type_;
    std::string s_type_;

    void goal_response_callback(const GoalHandleLinearTrajectory::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        GoalHandleLinearTrajectory::SharedPtr,
        const std::shared_ptr<const LinearTrajectory::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), 
                    "Feedback - Time: %.2fs | Error: [%.4f, %.4f, %.4f] | Norm: %.4f",
                    feedback->time_elapsed,
                    feedback->error_x,
                    feedback->error_y,
                    feedback->error_z,
                    feedback->error_norm);
    }

    void result_callback(const GoalHandleLinearTrajectory::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                rclcpp::shutdown();
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                rclcpp::shutdown();
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                rclcpp::shutdown();
                return;
        }

        RCLCPP_INFO(this->get_logger(), "Result:");
        RCLCPP_INFO(this->get_logger(), "  Success: %s", result.result->success ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  Final error norm: %.6f", result.result->final_error_norm);
        
        rclcpp::shutdown();
    }
};  // class LinearTrajectoryActionClient

}  // namespace ros2_kdl_package

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_kdl_package::LinearTrajectoryActionClient)