#include "spes_move/move.hpp"

#define sign(x) (((x) > 0) - ((x) < 0))
#define min(x, y) (((x) < (y)) ? (x) : (y))

namespace spes_move
{
  void Move::on_command_received(const spes_msgs::msg::MoveCommand::SharedPtr msg)
  {
    if (state_ == spes_msgs::msg::MoveState::IDLE)
    {
      init_move(msg);
      return;
    }

    update_odom_target_tf();
  }

  bool Move::update_odom_target_tf()
  {
    tf2::Transform tf_global_target;
    tf_global_target.setOrigin(tf2::Vector3(command_->target.x, command_->target.y, 0.0));
    tf_global_target.setRotation(tf2::Quaternion(
        tf2::Vector3(0, 0, 1), command_->target.theta));

    geometry_msgs::msg::TransformStamped tf_global_odom_message;
    try
    {
      tf_global_odom_message = tf_->lookupTransform(command_->header.frame_id, command_->odom_frame, command_->header.stamp);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_ERROR(get_logger(), "Initial global_frame -> command_->odom_frame is not available.");
      return false;
    }

    tf2::Transform tf_global_odom;
    tf2::convert(tf_global_odom_message.transform, tf_global_odom);
    tf_odom_target_ = tf_global_odom.inverse() * tf_global_target;
    target_updated_ = true;

    // Reset multiturn
    multiturn_n_ = 0;
    use_multiturn_ = false;
    previous_yaw_ = tf2::getYaw(tf_global_target.getRotation());
    return true;
  }

  bool Move::init_move(const spes_msgs::msg::MoveCommand::SharedPtr command)
  {
    command_ = command;
    end_time_ = command_->timeout + now();

    // Apply defaults
    if (command_->header.frame_id == "")
      command_->header.frame_id = "map";
    if (command_->odom_frame == "")
      command_->odom_frame = "odom";
    if (command_->linear_properties.max_velocity == 0.0)
      command_->linear_properties.max_velocity = default_command_->linear_properties.max_velocity;
    if (command_->linear_properties.max_acceleration == 0.0)
      command_->linear_properties.max_acceleration = default_command_->linear_properties.max_acceleration;
    if (command_->linear_properties.kp == 0.0)
      command_->linear_properties.kp = default_command_->linear_properties.kp;
    if (command_->linear_properties.kd == 0.0)
      command_->linear_properties.kd = default_command_->linear_properties.kd;
    if (command_->linear_properties.tolerance == 0.0)
      command_->linear_properties.tolerance = default_command_->linear_properties.tolerance;
    if (command_->angular_properties.max_velocity == 0.0)
      command_->angular_properties.max_velocity = default_command_->angular_properties.max_velocity;
    if (command_->angular_properties.max_acceleration == 0.0)
      command_->angular_properties.max_acceleration = default_command_->angular_properties.max_acceleration;
    if (command_->angular_properties.kp == 0.0)
      command_->angular_properties.kp = default_command_->angular_properties.kp;
    if (command_->angular_properties.kd == 0.0)
      command_->angular_properties.kd = default_command_->angular_properties.kd;
    if (command_->angular_properties.tolerance == 0.0)
      command_->angular_properties.tolerance = default_command_->angular_properties.tolerance;

    update_odom_target_tf();

    // Kickoff FSM
    lock_tf_odom_base_ = false;
    if (command->rotate_towards_goal)
    {
      state_ = spes_msgs::msg::MoveState::ROTATING_TOWARDS_GOAL;
      return true;
    }
    if (command->translate)
    {
      state_ = spes_msgs::msg::MoveState::TRANSLATING;
      return true;
    }
    if (command->rotate_at_goal)
    {
      state_ = spes_msgs::msg::MoveState::ROTATING_AT_GOAL;
      return true;
    }
    RCLCPP_ERROR(get_logger(), "Invalid MoveCommand, at least one of rotate_towards_goal, translate, or rotate_at_goal must be true.");
    return false;
  }

  double Move::get_diff_final_orientation(const tf2::Transform &tf_base_target)
  {
    const double final_yaw_raw = tf2::getYaw(tf_base_target.getRotation());
    if (use_multiturn_)
    {
      if (final_yaw_raw - previous_yaw_ > M_PI)
        multiturn_n_--;
      else if (final_yaw_raw - previous_yaw_ < -M_PI)
        multiturn_n_++;
    }
    previous_yaw_ = final_yaw_raw;
    return final_yaw_raw + multiturn_n_ * 2 * M_PI;
  }

  double Move::get_diff_heading(const tf2::Transform &tf_base_target)
  {
    if (command_->reversing == spes_msgs::msg::MoveCommand::REVERSING_AUTO)
    {
      const double diff_yaw_back = atan2(-tf_base_target.getOrigin().y(), -tf_base_target.getOrigin().x());
      const double diff_yaw_forward = atan2(tf_base_target.getOrigin().y(), tf_base_target.getOrigin().x());
      return (abs(diff_yaw_back) < abs(diff_yaw_forward)) ? diff_yaw_back : diff_yaw_forward;
    }
    if (command_->reversing == spes_msgs::msg::MoveCommand::REVERSING_FORCE)
      return atan2(-tf_base_target.getOrigin().y(), -tf_base_target.getOrigin().x());
    return atan2(tf_base_target.getOrigin().y(), tf_base_target.getOrigin().x());
  }

  double Move::get_distance(const tf2::Transform &tf_base_target)
  {
    return sqrt(tf_base_target.getOrigin().x() * tf_base_target.getOrigin().x() + tf_base_target.getOrigin().y() * tf_base_target.getOrigin().y());
  }

  void Move::state_rotating_towards_goal(const tf2::Transform &tf_base_target, const tf2::Transform &tf_odom_base, geometry_msgs::msg::Twist *cmd_vel)
  {
    const bool should_init = (state_ != previous_state_);
    const double diff_yaw = get_diff_heading(tf_base_target);

    if (should_init)
    {
      debouncing_reset();
      const double distance_to_goal = get_distance(tf_base_target);
      if (distance_to_goal < command_->linear_properties.tolerance)
      {
        // In case we are already at the goal we skip rotation towards the goal and translation.
        state_ = spes_msgs::msg::MoveState::ROTATING_AT_GOAL;
        return;
      }
      else
      {
        // TODO: Parametrize this threshold
        if (distance_to_goal < 0.15)
        {
          // When a robot is very close to the goal we cannot use atan2(diff_y, diff_x) as the goal shifts during the rotation.
          lock_tf_odom_base_ = true;
          locked_tf_odom_base_ = tf_odom_base;
        }
        init_rotation(diff_yaw);
      }
    }

    regulate_rotation(cmd_vel, diff_yaw);
    if (abs(diff_yaw) < command_->angular_properties.tolerance)
    {
      if (now() >= debouncing_end_)
      {
        // stopRobot();
        lock_tf_odom_base_ = false;
        state_ = spes_msgs::msg::MoveState::TRANSLATING;
        debouncing_reset();
      }
      return;
    }
    debouncing_reset();
  }

  void Move::state_translating(const tf2::Transform &tf_base_target, geometry_msgs::msg::Twist *cmd_vel)
  {
    const bool should_init = (state_ != previous_state_);
    if (should_init) {
      debouncing_reset();
      init_translation(tf_base_target.getOrigin().x(), tf_base_target.getOrigin().y());
    }

    regulate_translation(cmd_vel, tf_base_target.getOrigin().x(), tf_base_target.getOrigin().y());
    if (abs(tf_base_target.getOrigin().x()) < command_->linear_properties.tolerance)
    {
      if (now() >= debouncing_end_)
      {
        // stopRobot();
        if (command_->rotate_at_goal)
        {
          state_ = spes_msgs::msg::MoveState::ROTATING_AT_GOAL;
          debouncing_reset();
        }
        else
        {
          state_ = spes_msgs::msg::MoveState::IDLE;
          return;
        }
      }
      return;
    }
    debouncing_reset();
  }

  void Move::state_rotating_at_goal(const tf2::Transform &tf_base_target, geometry_msgs::msg::Twist *cmd_vel)
  {
    const bool should_init = (state_ != previous_state_);
    const double final_yaw = get_diff_final_orientation(tf_base_target);

    if (should_init) {
      debouncing_reset();
      init_rotation(final_yaw);
    }

    regulate_rotation(cmd_vel, final_yaw);
    if (abs(final_yaw) < command_->angular_properties.tolerance)
    {
      if (now() >= debouncing_end_)
      {
        // stopRobot();
        debouncing_reset();
        state_ = spes_msgs::msg::MoveState::IDLE;
      }
      return;
    }
    debouncing_reset();
  }

  void Move::update()
  {
    if (state_ == spes_msgs::msg::MoveState::IDLE)
    {
      previous_state_ = state_;
      return;
    }

    const uint8_t previous_state = state_;

    // Timeout
    rclcpp::Duration time_remaining = end_time_ - now();
    if (time_remaining.seconds() < 0.0 && rclcpp::Duration(command_->timeout).seconds() > 0.0)
    {
      // stopRobot();
      RCLCPP_WARN(
          get_logger(),
          "Exceeded time allowance before reaching the Move goal - Exiting Move");
      state_ = spes_msgs::msg::MoveState::IDLE;
      return;
    }

    // Target in the base frame
    geometry_msgs::msg::PoseStamped tf_odom_base_message;
    if (!nav2_util::getCurrentPose(
            tf_odom_base_message, *tf_, command_->odom_frame, robot_frame_,
            transform_tolerance_))
    {
      RCLCPP_ERROR(get_logger(), "Initial odom_frame -> base frame is not available.");
      state_ = spes_msgs::msg::MoveState::IDLE;
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

    RCLCPP_INFO(get_logger(), "Current state: %lf", get_diff_final_orientation(tf_base_target));

    // FSM
    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
    switch (state_)
    {
    case spes_msgs::msg::MoveState::ROTATING_TOWARDS_GOAL:
      state_rotating_towards_goal(tf_base_target, tf_odom_base, cmd_vel.get());
      break;
    case spes_msgs::msg::MoveState::TRANSLATING:
      state_translating(tf_base_target, cmd_vel.get());
      break;
    case spes_msgs::msg::MoveState::ROTATING_AT_GOAL:
      state_rotating_at_goal(tf_base_target, cmd_vel.get());
      break;
    }

    // Stop if there is a collision
    // if (!command_->ignore_obstacles)
    // {
    //   geometry_msgs::msg::PoseStamped current_pose;
    //   if (!nav2_util::getCurrentPose(
    //           current_pose, *tf_, command_->header.frame_id, robot_frame_,
    //           transform_tolerance_))
    //   {
    //     RCLCPP_ERROR(get_logger(), "Current robot pose is not available.");
    //     state_ = spes_msgs::msg::MoveState::IDLE;
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
    //     RCLCPP_WARN(get_logger(), "Collision Ahead - Exiting Move");
    //     state_ = spes_msgs::msg::MoveState::IDLE;
    //     return;
    //   }
    // }

    cmd_vel_pub_->publish(std::move(cmd_vel));

    spes_msgs::msg::MoveState state_msg;
    state_msg.state = state_;
    state_msg.distance_xy = get_distance(tf_base_target);
    state_msg.distance_x = tf_base_target.getOrigin().x();
    state_msg.distance_yaw = get_diff_final_orientation(tf_base_target);
    state_pub_->publish(state_msg);

    previous_state_ = previous_state;
  }

  void Move::init_rotation(double diff_yaw)
  {
    if (rotation_ruckig_ != nullptr)
      delete rotation_ruckig_;

    rotation_ruckig_ = new ruckig::Ruckig<1>{1.0 / update_rate_};
    rotation_ruckig_input_.max_velocity = {command_->angular_properties.max_velocity};
    rotation_ruckig_input_.max_acceleration = {command_->angular_properties.max_acceleration};
    rotation_ruckig_input_.max_jerk = {99999999999.0};
    rotation_ruckig_input_.target_position = {0};
    rotation_ruckig_input_.current_position = {diff_yaw};
    rotation_ruckig_input_.control_interface = ruckig::ControlInterface::Position;
    rotation_ruckig_->update(rotation_ruckig_input_, rotation_ruckig_output_);

    last_error_yaw_ = 0;
  }

  void Move::regulate_rotation(geometry_msgs::msg::Twist *cmd_vel, double diff_yaw)
  {
    if (target_updated_) {
      // TODO: This will produce minor jitter when a goal is updated.
      rotation_ruckig_input_.current_position[0] = diff_yaw - last_error_yaw_;
      target_updated_ = false;
    }

    const double previous_input = rotation_ruckig_output_.new_position[0];
    const bool is_trajectory_finished = (rotation_ruckig_->update(rotation_ruckig_input_, rotation_ruckig_output_) == ruckig::Finished);
    last_error_yaw_ = diff_yaw - rotation_ruckig_output_.new_position[0];
    const double d_input = rotation_ruckig_output_.new_position[0] - previous_input;
    cmd_vel->angular.z = command_->angular_properties.kp * last_error_yaw_ - command_->angular_properties.kd * d_input;
  
    if (!is_trajectory_finished)
      rotation_ruckig_output_.pass_to_input(rotation_ruckig_input_);
  }

  void Move::init_translation(double diff_x, double diff_y)
  {
    if (translation_ruckig_ != nullptr)
      delete translation_ruckig_;

    translation_ruckig_ = new ruckig::Ruckig<1>{1.0 / update_rate_};
    translation_ruckig_input_.max_velocity = {command_->linear_properties.max_velocity};
    translation_ruckig_input_.max_acceleration = {command_->linear_properties.max_acceleration};
    translation_ruckig_input_.max_jerk = {99999999999.0};
    translation_ruckig_input_.target_position = {0};
    translation_ruckig_input_.current_position = {diff_x};
    translation_ruckig_input_.control_interface = ruckig::ControlInterface::Position;
    translation_ruckig_->update(translation_ruckig_input_, translation_ruckig_output_);

    last_error_x_ = 0;
    last_error_y_ = 0;
  }

  void Move::regulate_translation(geometry_msgs::msg::Twist *cmd_vel, double diff_x, double diff_y)
  {
    if (target_updated_) {
      // TODO: This will produce minor jitter when a goal is updated.
      translation_ruckig_input_.current_position[0] = diff_x - last_error_x_;
      target_updated_ = false;
    }

    const double previous_input = translation_ruckig_output_.new_position[0];
    const bool is_trajectory_finished = (translation_ruckig_->update(translation_ruckig_input_, translation_ruckig_output_) == ruckig::Finished);
    last_error_x_ = diff_x - translation_ruckig_output_.new_position[0];
    const double d_input = translation_ruckig_output_.new_position[0] - previous_input;
    cmd_vel->linear.x = command_->linear_properties.kp * last_error_x_ - command_->linear_properties.kd * d_input;

    // TODO: Parameterize this + add kd
    cmd_vel->angular.z = diff_y * cmd_vel->linear.x * 1.0;

    if (!is_trajectory_finished)
      translation_ruckig_output_.pass_to_input(translation_ruckig_input_);
  }

  Move::Move(std::string name) : Node(name)
  {
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    command_sub_ = create_subscription<spes_msgs::msg::MoveCommand>(
        "~/command", 1, std::bind(&Move::on_command_received, this, std::placeholders::_1));
    state_pub_ = create_publisher<spes_msgs::msg::MoveState>("~/state", 1);

    tf_ =
        std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_);

    // Read parameters
    declare_parameter("update_rate", rclcpp::ParameterValue(50));
    get_parameter("update_rate", update_rate_);

    double command_timeout;
    declare_parameter("command_timeout", rclcpp::ParameterValue(0.5));
    get_parameter("command_timeout", command_timeout);
    command_timeout_ = rclcpp::Duration::from_seconds(command_timeout);

    double debouncing_duration;
    declare_parameter("debouncing_duration", rclcpp::ParameterValue(0.05));
    get_parameter("debouncing_duration", debouncing_duration);
    debouncing_duration_ = rclcpp::Duration::from_seconds(debouncing_duration);

    // Linear
    declare_parameter("linear.kp", rclcpp::ParameterValue(3.0));
    get_parameter("linear.kp", default_command_->linear_properties.kp);

    declare_parameter("linear.kd", rclcpp::ParameterValue(0.0));
    get_parameter("linear.kd", default_command_->linear_properties.kd);

    declare_parameter("linear.max_velocity", rclcpp::ParameterValue(0.1));
    get_parameter("linear.max_velocity", default_command_->linear_properties.max_velocity);

    declare_parameter("linear.max_acceleration", rclcpp::ParameterValue(1.5));
    get_parameter("linear.max_acceleration", default_command_->linear_properties.max_acceleration);

    declare_parameter("linear.tolerance", rclcpp::ParameterValue(0.01));
    get_parameter("linear.tolerance", default_command_->linear_properties.tolerance);

    // Angular
    declare_parameter("angular.kp", rclcpp::ParameterValue(5.0));
    get_parameter("angular.kp", default_command_->angular_properties.kp);

    declare_parameter("angular.kd", rclcpp::ParameterValue(0.0));
    get_parameter("angular.kd", default_command_->angular_properties.kd);

    declare_parameter("angular.max_velocity", rclcpp::ParameterValue(0.5));
    get_parameter("angular.max_velocity", default_command_->angular_properties.max_velocity);

    declare_parameter("angular.max_acceleration", rclcpp::ParameterValue(0.1));
    get_parameter("angular.max_acceleration", default_command_->angular_properties.max_acceleration);

    declare_parameter("angular.tolerance", rclcpp::ParameterValue(0.03));
    get_parameter("angular.tolerance", default_command_->angular_properties.tolerance);
  }

  void Move::debouncing_reset()
  {
    debouncing_end_ = now() + debouncing_duration_;
  }
};