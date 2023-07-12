#ifndef SPES_MOVE__MOVE_HPP_
#define SPES_MOVE__MOVE_HPP_

#include "nav2_behaviors/timed_behavior.hpp"
#include "spes_msgs/msg/move_command.hpp"
#include "spes_msgs/msg/move_properties.hpp"
#include "spes_msgs/msg/move_state.hpp"
#include "ruckig/ruckig.hpp"
#include "rclcpp/rclcpp.hpp"

namespace spes_move
{
  class Move : public rclcpp::Node
  {
  public:
    Move(std::string name);
    void update();
    int get_update_rate() { return update_rate_; };

  private:
    void on_command_received(const spes_msgs::msg::MoveCommand::SharedPtr msg);

    bool init_move(const spes_msgs::msg::MoveCommand::SharedPtr msg);
    void init_rotation(double diff_yaw);
    void regulate_rotation(geometry_msgs::msg::Twist *cmd_vel, double diff_yaw);
    void init_translation(double diff_x, double diff_y);
    void regulate_translation(geometry_msgs::msg::Twist *cmd_vel, double diff_x, double diff_y);

    void state_rotating_towards_goal(const tf2::Transform &tf_base_target, const tf2::Transform &tf_odom_base, geometry_msgs::msg::Twist *cmd_vel);
    void state_translating(const tf2::Transform &tf_base_target, geometry_msgs::msg::Twist *cmd_vel);
    void state_rotating_at_goal(const tf2::Transform &tf_base_target, geometry_msgs::msg::Twist *cmd_vel);
    void state_stopping();

    double get_diff_heading(const tf2::Transform &tf_base_target);
    double get_diff_final_orientation(const tf2::Transform& tf_base_target);
    double get_distance(const tf2::Transform &tf_base_target);

    bool update_odom_target_tf();

    int update_rate_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<spes_msgs::msg::MoveCommand>::SharedPtr command_sub_;
    rclcpp::Publisher<spes_msgs::msg::MoveState>::SharedPtr state_pub_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>> collision_checker_;

    spes_msgs::msg::MoveCommand::SharedPtr command_;
    spes_msgs::msg::MoveCommand::SharedPtr default_command_{new spes_msgs::msg::MoveCommand()};

    std::string robot_frame_{"base_link"};
  
    tf2::Transform tf_odom_target_;
    rclcpp::Time end_time_;

    rclcpp::Time debouncing_end_;
    rclcpp::Duration debouncing_duration_{0, 0};
    void debouncing_reset();

    ruckig::Ruckig<1> *rotation_ruckig_{nullptr};
    ruckig::InputParameter<1> rotation_ruckig_input_;
    ruckig::OutputParameter<1> rotation_ruckig_output_;
    double rotation_last_input_;
    double previous_yaw_;
    int multiturn_n_;
    bool use_multiturn_;

    tf2::Transform locked_tf_odom_base_;
    bool lock_tf_odom_base_{false};

    ruckig::Ruckig<1> *translation_ruckig_{nullptr};
    ruckig::InputParameter<1> translation_ruckig_input_;
    ruckig::OutputParameter<1> translation_ruckig_output_;
    double translation_last_input_;

    rclcpp::Duration command_timeout_{0, 0};
    rclcpp::Time last_command_received_;

    uint8_t state_{spes_msgs::msg::MoveState::IDLE};
    uint8_t previous_state_{spes_msgs::msg::MoveState::IDLE};

    // TODO: Remove
    double transform_tolerance_{0.5};
  };

} // namespace spes_move

#endif // SPES_MOVE__MOVE_HPP_
