#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <vector>

struct Control {
  // controller gains
  float Kp;
  float Ki;
  float Kd;

  // controller output limits
  float output_min;
  float output_max;

  // integrator state
  float integrator;
  float previous_error;

  // integrator limits
  float integrator_min;
  float integrator_max;

  // differentiator state
  float differentiator;
  float previous_measurement;

  // sample time
  float dt;

  float get_proportional_gain(float error) {
    // calculate proportional gain
    return Kp * error;
  }

  float get_integral_gain(float error) {
    // calculate integral gain
    float value = integrator + (0.5 * Ki * (error + previous_error) * dt);
    // anti-wind-up via integrator clamping
    if (value > integrator_max) {
      integrator = integrator_max;
    } else if (value < integrator_min) {
      integrator = integrator_min;
    } else {
      integrator = value;
    }
    previous_error = error;
    return integrator;
  }

  float get_differential_gain(float measurement) {
    // calculate differential gain (derivative on measurement)
    float value = (measurement - previous_measurement) / dt;
    previous_measurement = measurement;
    return Kd * value;
  }

  float get_total_gain(float error, float measurement) {
    // get proportional, integral and differential gain
    float p = get_proportional_gain(error);
    float i = get_integral_gain(error);
    float d = get_differential_gain(measurement);

    // calculate total gain
    float output = p + i + d;

    // output with limits
    if (output > output_max) {
      return output_max;
    } else if (output < output_min) {
      return output_min;
    } else {
      return output;
    }
  }
};

class TurnControllerNode : public rclcpp::Node {
private:
  // member variables
  double px;
  double py;
  double yaw;

  std::vector<std::vector<float>> waypoints;
  Control pid_angle;

  // callback groups
  rclcpp::CallbackGroup::SharedPtr callback_g1;
  rclcpp::CallbackGroup::SharedPtr callback_g2;

  // ros objects
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel;
  rclcpp::TimerBase::SharedPtr timer_control;

  // member method
  void subscriber_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // reading current position from /odom topic
    this->px = msg->pose.pose.position.x;
    this->py = msg->pose.pose.position.y;

    // reading current orientation from /odom topic
    double x = msg->pose.pose.orientation.x;
    double y = msg->pose.pose.orientation.y;
    double z = msg->pose.pose.orientation.z;
    double w = msg->pose.pose.orientation.w;

    // convert quaternion into euler angles
    this->yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  }

  void timer_control_callback() {
    // start waypoint execution
    int counter = 1;
    for (auto waypoint : waypoints) {
      RCLCPP_INFO(this->get_logger(), "Executing Waypoint : %d", counter);
      this->turn_robot(waypoint[1], waypoint[2], waypoint[0]);
      counter++;
    }

    // halt the execution
    this->timer_control->cancel();
  }

  void turn_robot(float x, float y, float phi) {
    // define message
    auto message = geometry_msgs::msg::Twist();

    // reset struct state variable
    pid_angle.integrator = 0.0;
    pid_angle.previous_error = 0.0;

    // loop till target done
    while (rclcpp::ok()) {
      float dx = x - px;
      float dy = y - py;
      float error = std::atan2(dy, dx) - yaw;
      float measurement = yaw;

      // fix : smallest difference between two angles around a point
      float error_prime = std::atan2(std::sin(error), std::cos(error));

      // get proportional, integral and differential gain
      float p = pid_angle.get_proportional_gain(error_prime);
      float i = pid_angle.get_integral_gain(error_prime);
      float d = pid_angle.get_differential_gain(measurement);

      // log error and gain
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                           "%f : [%f, %f, %f] %f", error_prime, p, i, d, yaw);

      // calculate total gain
      float output = p + i + d;

      // output with limits
      if (output > pid_angle.output_max) {
        output = pid_angle.output_max;
      } else if (output < pid_angle.output_min) {
        output = pid_angle.output_min;
      } else {
        output = output;
      }

      if (std::fabs(error) > 0.01) {
        // publish velocity
        message.angular.z = output;
        this->publisher_cmd_vel->publish(message);
      } else {
        break;
      }
    }
    // publish velocity
    message.angular.z = 0.0;
    this->publisher_cmd_vel->publish(message);

    // halt thread for few seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // suppress unused variables warnings
    (void)phi;
  }

public:
  // constructor
  TurnControllerNode(std::vector<std::vector<float>> target_waypoints,
                     Control &pid_angle)
      : Node("turn_controller") {
    // initialize member variables
    this->waypoints = target_waypoints;
    this->pid_angle = pid_angle;

    // callback groups objects
    callback_g1 =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    callback_g2 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // ros node options
    rclcpp::SubscriptionOptions sub_callback_g1;
    sub_callback_g1.callback_group = callback_g1;

    // ros objects
    this->subscriber_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&TurnControllerNode::subscriber_odom_callback, this,
                  std::placeholders::_1),
        sub_callback_g1);
    this->publisher_cmd_vel =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    this->timer_control = this->create_wall_timer(
        std::chrono::seconds(1 / 1),
        std::bind(&TurnControllerNode::timer_control_callback, this),
        callback_g2);
  }
};

int main(int argc, char *argv[]) {
  // initialize ros
  rclcpp::init(argc, argv);

  // waypoints for robot motion
  std::vector<std::vector<float>> waypoints;

  // proportional integral derivative control for angle
  Control pid_angle{0.8, 0.01, 0.1, -1.0, 1.0, 0.0,
                    0.0, -1.0, 1.0, 0.0,  0.0, 0.0001};

  if (std::atoi(argv[1]) == 2) {
    // real lab paramters
    waypoints = {
        {0.0, 1.0, -0.25},
        {0.0, 0.6, -1.45},
    };
    pid_angle.output_min = -0.7;
    pid_angle.output_max = 0.7;
  } else {
    // simulation paramters
    waypoints = {
        {0.0, 1, -1},
        {0.0, 1, 0},
        {0.0, 1, 1},
    };
  }

  // initialize executor and node
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<TurnControllerNode>(waypoints, pid_angle);

  // add node to executor and spin
  executor.add_node(node);
  executor.spin();

  // shutdown
  rclcpp::shutdown();
  return 0;
}