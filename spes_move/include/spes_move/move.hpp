#ifndef SPES_MOVE__MOVE_HPP_
#define SPES_MOVE__MOVE_HPP_

#include "nav2_behaviors/timed_behavior.hpp"
#include "spes_msgs/msg/move_command.hpp"
#include "spes_msgs/msg/move_properties.hpp"
#include "ruckig/ruckig.hpp"
#include "rclcpp/rclcpp.hpp"

namespace spes_move
{
  enum MoveState
  {
    IDLE,
    INITIALIZE_ROTATION_TOWARDS_GOAL,
    REGULATE_ROTATION_TOWARDS_GOAL,
    INITIALIZE_TRANSLATION,
    REGULATE_TRANSLATION,
    INITIALIZE_ROTATION_AT_GOAL,
    REGULATE_ROTATION_AT_GOAL
  };

  class Move
  {
  public:
    Move(rclcpp::Node::SharedPtr node, double cycle_frequency);
    void update();

  private:
    void on_command_received(const spes_msgs::msg::MoveCommand::SharedPtr msg){};

    bool init_move(const spes_msgs::msg::MoveCommand::SharedPtr msg);
    void init_rotation(double diff_yaw);
    void regulate_rotation(geometry_msgs::msg::Twist *cmd_vel, double diff_yaw);
    void init_translation(double diff_x, double diff_y);
    void regulate_translation(geometry_msgs::msg::Twist *cmd_vel, double diff_x, double diff_y);

    double get_diff_heading(const tf2::Transform &tf_base_target);
    double get_diff_final_orientation(const tf2::Transform& tf_base_target);
    double get_distance(const tf2::Transform &tf_base_target);

    rclcpp::Node::SharedPtr node_;
    double cycle_frequency_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>> collision_checker_;

    spes_msgs::msg::MoveCommand::SharedPtr command_;
    spes_msgs::msg::MoveCommand::SharedPtr default_command_{new spes_msgs::msg::MoveCommand()};

    std::string robot_frame_{"base_link"};
    tf2::Transform tf_odom_target_;

    rclcpp::Duration timeout_{0, 0};
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

    MoveState state_{MoveState::IDLE};

    // TODO: Remove
    double transform_tolerance_{0.5};
  };

} // namespace spes_move

#endif // SPES_MOVE__MOVE_HPP_
