#include "spes_move/move.hpp"

#define sign(x) (((x) > 0) - ((x) < 0))
#define min(x, y) (((x) < (y)) ? (x) : (y))

namespace spes_move
{
  bool Move::init_move(const spes_msgs::msg::MoveCommand::SharedPtr command)
  {
    move_command_ = command;

    global_frame_ = command->header.frame_id;
    odom_frame_ = command->odom_frame;
    ignore_obstacles_ = command->ignore_obstacles;
    timeout_ = command->timeout;
    end_time_ = timeout_ + node_->now();
    linear_properties_ = command->linear_properties;
    angular_properties_ = command->angular_properties;
    reversing_ = command->reversing;

    // Apply defaults
    if (global_frame_ == "")
      global_frame_ = "map";
    if (odom_frame_ == "")
      odom_frame_ = "odom";
    if (linear_properties_.max_velocity == 0.0)
      linear_properties_.max_velocity = default_linear_properties_.max_velocity;
    if (linear_properties_.max_acceleration == 0.0)
      linear_properties_.max_acceleration = default_linear_properties_.max_acceleration;
    if (linear_properties_.kp == 0.0)
      linear_properties_.kp = default_linear_properties_.kp;
    if (linear_properties_.kd == 0.0)
      linear_properties_.kd = default_linear_properties_.kd;
    if (linear_properties_.tolerance == 0.0)
      linear_properties_.tolerance = default_linear_properties_.tolerance;
    if (angular_properties_.max_velocity == 0.0)
      angular_properties_.max_velocity = default_angular_properties_.max_velocity;
    if (angular_properties_.max_acceleration == 0.0)
      angular_properties_.max_acceleration = default_angular_properties_.max_acceleration;
    if (angular_properties_.kp == 0.0)
      angular_properties_.kp = default_angular_properties_.kp;
    if (angular_properties_.kd == 0.0)
      angular_properties_.kd = default_angular_properties_.kd;
    if (angular_properties_.tolerance == 0.0)
      angular_properties_.tolerance = default_angular_properties_.tolerance;

    // Target in the global frame
    tf2::Transform tf_global_target;
    tf_global_target.setOrigin(tf2::Vector3(command->target.x, command->target.y, 0.0));
    tf_global_target.setRotation(tf2::Quaternion(
        tf2::Vector3(0, 0, 1), command->target.theta));

    geometry_msgs::msg::PoseStamped tf_global_odom_message;
    if (!nav2_util::getCurrentPose(
            tf_global_odom_message, *tf_, global_frame_, odom_frame_,
            transform_tolerance_))
    {
      RCLCPP_ERROR(node_->get_logger(), "Initial global_frame -> odom_frame_ is not available.");
      return false;
    }
    tf2::Transform tf_global_odom;
    tf2::convert(tf_global_odom_message.pose, tf_global_odom);
    tf_odom_target_ = tf_global_odom.inverse() * tf_global_target;

    // Reset multiturn
    multiturn_n_ = 0;
    use_multiturn_ = false;
    previous_yaw_ = tf2::getYaw(tf_global_target.getRotation());

    // Kickoff FSM
    lock_tf_odom_base_ = false;
    if (command->rotate_towards_goal) {
      state_ = MoveState::INITIALIZE_ROTATION_TOWARDS_GOAL;
      return true;
    }
    if (command->translate) {
      state_ = MoveState::INITIALIZE_TRANSLATION;
      return true;
    }
    if (command->rotate_at_goal) {
      state_ = MoveState::INITIALIZE_ROTATION_AT_GOAL;
      return true;
    }
    RCLCPP_ERROR(node_->get_logger(), "Invalid MoveCommand, at least one of rotate_towards_goal, translate, or rotate_at_goal must be true.");
    return false;
  }

  void Move::update()
  {
    if (state_ == MoveState::IDLE)
      return;

    // Timeout
    rclcpp::Duration time_remaining = end_time_ - node_->now();
    if (time_remaining.seconds() < 0.0 && timeout_.seconds() > 0.0)
    {
      // stopRobot();
      RCLCPP_WARN(
          node_->get_logger(),
          "Exceeded time allowance before reaching the Move goal - Exiting Move");
      state_ = MoveState::IDLE;
      return;
    }

    // Target in the base frame
    geometry_msgs::msg::PoseStamped tf_odom_base_message;
    if (!nav2_util::getCurrentPose(
            tf_odom_base_message, *tf_, odom_frame_, robot_frame_,
            transform_tolerance_))
    {
      RCLCPP_ERROR(node_->get_logger(), "Initial odom_frame -> base frame is not available.");
      state_ = MoveState::IDLE;
      return;
    }
    tf2::Transform tf_odom_base;
    tf2::convert(tf_odom_base_message.pose, tf_odom_base);
    if (lock_tf_odom_base_)
    {
      tf_odom_base.getOrigin().setX(locked_tf_odom_base_.getOrigin().x());
      tf_odom_base.getOrigin().setY(locked_tf_odom_base_.getOrigin().y());
    }
    const tf2::Transform tf_base_target = tf_odom_base.inverse() * tf_odom_target_;

    const double final_yaw_raw = tf2::getYaw(tf_base_target.getRotation());
    if (use_multiturn_)
    {
      if (final_yaw_raw - previous_yaw_ > M_PI)
        multiturn_n_--;
      else if (final_yaw_raw - previous_yaw_ < -M_PI)
        multiturn_n_++;
    }
    previous_yaw_ = final_yaw_raw;
    const double final_yaw = final_yaw_raw + multiturn_n_ * 2 * M_PI;
    const double diff_x = tf_base_target.getOrigin().x();
    const double diff_y = tf_base_target.getOrigin().y();

    double diff_yaw = 0;
    if (reversing_ == spes_msgs::msg::MoveCommand::REVERSING_AUTO)
    {
      const double diff_yaw_back = atan2(-diff_y, -diff_x);
      const double diff_yaw_forward = atan2(diff_y, diff_x);
      diff_yaw = (abs(diff_yaw_back) < abs(diff_yaw_forward)) ? diff_yaw_back : diff_yaw_forward;
    }
    else if (reversing_ == spes_msgs::msg::MoveCommand::REVERSING_FORCE)
    {
      diff_yaw = atan2(-diff_y, -diff_x);
    }
    else
    {
      diff_yaw = atan2(diff_y, diff_x);
    }

    // FSM
    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
    switch (state_)
    {
    case MoveState::INITIALIZE_ROTATION_TOWARDS_GOAL:
    {
      const double dinstace_to_goal = sqrt(diff_x * diff_x + diff_y * diff_y);
      if (dinstace_to_goal < linear_properties_.tolerance)
      {
        // In case we are already at the goal we skip rotation towards the goal and translation.
        state_ = MoveState::INITIALIZE_ROTATION_AT_GOAL;
      }
      else
      {
        if (dinstace_to_goal < 0.15)
        {
          // When a robot is very close to the goal we cannot use atan2(diff_y, diff_x) as the goal shits during the rotation.
          lock_tf_odom_base_ = true;
          locked_tf_odom_base_ = tf_odom_base;
        }
        init_rotation(diff_yaw);
        regulate_rotation(cmd_vel.get(), diff_yaw);
        state_ = MoveState::REGULATE_ROTATION_TOWARDS_GOAL;
      }
    }
    break;
    case MoveState::REGULATE_ROTATION_TOWARDS_GOAL:
      regulate_rotation(cmd_vel.get(), diff_yaw);
      if (abs(diff_yaw) < angular_properties_.tolerance)
      {
        if (node_->now() >= debouncing_end_)
        {
          // stopRobot();
          lock_tf_odom_base_ = false;
          state_ = MoveState::INITIALIZE_TRANSLATION;
          debouncing_reset();
        }
      }
      else
      {
        debouncing_reset();
      }
      break;
    case MoveState::INITIALIZE_TRANSLATION:
      init_translation(diff_x, diff_y);
      state_ = MoveState::REGULATE_TRANSLATION;
      break;
    case MoveState::REGULATE_TRANSLATION:
      regulate_translation(cmd_vel.get(), diff_x, diff_y);
      if (abs(diff_x) < linear_properties_.tolerance)
      {
        if (node_->now() >= debouncing_end_)
        {
          // stopRobot();
          if (move_command_->rotate_at_goal) {
            state_ = MoveState::INITIALIZE_ROTATION_AT_GOAL;
            debouncing_reset();
          } else {
            state_ = MoveState::IDLE;
            return;
          }
        }
      }
      else
      {
        debouncing_reset();
      }
      break;
    case MoveState::INITIALIZE_ROTATION_AT_GOAL:
      init_rotation(final_yaw);
      regulate_rotation(cmd_vel.get(), final_yaw);
      state_ = MoveState::REGULATE_ROTATION_AT_GOAL;
      break;
    case MoveState::REGULATE_ROTATION_AT_GOAL:
      regulate_rotation(cmd_vel.get(), final_yaw);
      if (abs(final_yaw) < angular_properties_.tolerance)
      {
        if (node_->now() >= debouncing_end_)
        {
          // stopRobot();
          debouncing_reset();
          state_ = MoveState::IDLE;
          return;
        }
      }
      else
      {
        debouncing_reset();
      }
      break;
    }

    // Stop if there is a collision
    // if (!ignore_obstacles_)
    // {
    //   geometry_msgs::msg::PoseStamped current_pose;
    //   if (!nav2_util::getCurrentPose(
    //           current_pose, *tf_, global_frame_, robot_frame_,
    //           transform_tolerance_))
    //   {
    //     RCLCPP_ERROR(node_->get_logger(), "Current robot pose is not available.");
    //     state_ = MoveState::IDLE;
    //     return;
    //   }

    //   geometry_msgs::msg::Pose2D pose2d;
    //   pose2d.x = current_pose.pose.position.x;
    //   pose2d.y = current_pose.pose.position.y;
    //   pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

    //   const double sim_position_change = sign(cmd_vel->linear.x) * simulate_ahead_distance_;
    //   pose2d.x += sim_position_change * cos(pose2d.theta);
    //   pose2d.y += sim_position_change * sin(pose2d.theta);
    //   if (!collision_checker_->isCollisionFree(pose2d))
    //   {
    //     // stopRobot();
    //     RCLCPP_WARN(node_->get_logger(), "Collision Ahead - Exiting Move");
    //     state_ = MoveState::IDLE;
    //     return;
    //   }
    // }

    cmd_vel_pub_->publish(std::move(cmd_vel));
  }

  void Move::init_rotation(double diff_yaw)
  {
    if (rotation_ruckig_ != nullptr)
      delete rotation_ruckig_;

    rotation_ruckig_ = new ruckig::Ruckig<1>{1.0 / cycle_frequency_};
    rotation_ruckig_input_.max_velocity = {angular_properties_.max_velocity};
    rotation_ruckig_input_.max_acceleration = {angular_properties_.max_acceleration};
    rotation_ruckig_input_.max_jerk = {99999999999.0};
    rotation_ruckig_input_.target_position = {0};
    rotation_ruckig_input_.current_position = {diff_yaw};
    rotation_ruckig_input_.control_interface = ruckig::ControlInterface::Position;
    rotation_ruckig_output_.new_position = {diff_yaw};
    rotation_ruckig_output_.new_velocity = {0.0};
    rotation_ruckig_output_.new_acceleration = {0.0};
    rotation_ruckig_output_.pass_to_input(rotation_ruckig_input_);
    rotation_last_input_ = rotation_ruckig_output_.new_position[0];
  }

  void Move::regulate_rotation(geometry_msgs::msg::Twist *cmd_vel, double diff_yaw)
  {
    if (rotation_ruckig_->update(rotation_ruckig_input_, rotation_ruckig_output_) != ruckig::Finished)
      rotation_ruckig_output_.pass_to_input(rotation_ruckig_input_);
    const double error_yaw = diff_yaw - rotation_ruckig_output_.new_position[0];
    const double d_input = rotation_ruckig_output_.new_position[0] - rotation_last_input_;
    rotation_last_input_ = rotation_ruckig_output_.new_position[0];
    cmd_vel->angular.z = angular_properties_.kp * error_yaw - angular_properties_.kd * d_input;
  }

  void Move::init_translation(double diff_x, double diff_y)
  {
    if (translation_ruckig_ != nullptr)
      delete translation_ruckig_;

    translation_ruckig_ = new ruckig::Ruckig<1>{1.0 / cycle_frequency_};
    translation_ruckig_input_.max_velocity = {linear_properties_.max_velocity};
    translation_ruckig_input_.max_acceleration = {linear_properties_.max_acceleration};
    translation_ruckig_input_.max_jerk = {99999999999.0};
    translation_ruckig_input_.target_position = {0};
    translation_ruckig_input_.current_position = {diff_x};
    translation_ruckig_input_.control_interface = ruckig::ControlInterface::Position;
    translation_ruckig_output_.new_position = {diff_x};
    translation_ruckig_output_.new_velocity = {0.0};
    translation_ruckig_output_.new_acceleration = {0.0};
    translation_ruckig_output_.pass_to_input(translation_ruckig_input_);
    translation_last_input_ = translation_ruckig_output_.new_position[0];
  }

  void Move::regulate_translation(geometry_msgs::msg::Twist *cmd_vel, double diff_x, double diff_y)
  {
    if (translation_ruckig_->update(translation_ruckig_input_, translation_ruckig_output_) != ruckig::Finished)
      translation_ruckig_output_.pass_to_input(translation_ruckig_input_);
    const double error_x = diff_x - translation_ruckig_output_.new_position[0];
    const double d_input = translation_ruckig_output_.new_position[0] - translation_last_input_;
    translation_last_input_ = translation_ruckig_output_.new_position[0];
    cmd_vel->linear.x = linear_properties_.kp * error_x - linear_properties_.kd * d_input;
    cmd_vel->angular.z = diff_y * cmd_vel->linear.x * 1.0;
  }

  Move::Move(rclcpp::Node::SharedPtr node, double cycle_frequency)
  {
    node_ = node;
    cycle_frequency_ = cycle_frequency;

    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    tf_ =
      std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_);

    node->declare_parameter("simulate_ahead_distance", rclcpp::ParameterValue(0.2));
    node->get_parameter("simulate_ahead_distance", simulate_ahead_distance_);

    double debouncing_duration;
    node->declare_parameter("debouncing_duration", rclcpp::ParameterValue(0.05));
    node->get_parameter("debouncing_duration", debouncing_duration);
    debouncing_duration_ = rclcpp::Duration::from_seconds(debouncing_duration);

    // Linear
    node->declare_parameter("linear.kp", rclcpp::ParameterValue(15.0));
    node->get_parameter("linear.kp", default_linear_properties_.kp);

    node->declare_parameter("linear.kd", rclcpp::ParameterValue(0.0));
    node->get_parameter("linear.kd", default_linear_properties_.kd);

    node->declare_parameter("linear.max_velocity", rclcpp::ParameterValue(0.5));
    node->get_parameter("linear.max_velocity", default_linear_properties_.max_velocity);

    node->declare_parameter("linear.max_acceleration", rclcpp::ParameterValue(0.5));
    node->get_parameter("linear.max_acceleration", default_linear_properties_.max_acceleration);

    node->declare_parameter("linear.tolerance", rclcpp::ParameterValue(0.01));
    node->get_parameter("linear.tolerance", default_linear_properties_.tolerance);

    // Angular
    node->declare_parameter("angular.kp", rclcpp::ParameterValue(15.0));
    node->get_parameter("angular.kp", default_angular_properties_.kp);

    node->declare_parameter("angular.kd", rclcpp::ParameterValue(0.0));
    node->get_parameter("angular.kd", default_angular_properties_.kd);

    node->declare_parameter("angular.max_velocity", rclcpp::ParameterValue(0.5));
    node->get_parameter("angular.max_velocity", default_angular_properties_.max_velocity);

    node->declare_parameter("angular.max_acceleration", rclcpp::ParameterValue(0.5));
    node->get_parameter("angular.max_acceleration", default_angular_properties_.max_acceleration);

    node->declare_parameter("angular.tolerance", rclcpp::ParameterValue(0.03));
    node->get_parameter("angular.tolerance", default_angular_properties_.tolerance);
  }

  void Move::debouncing_reset()
  {
    debouncing_end_ = node_->now() + debouncing_duration_;
  }
};