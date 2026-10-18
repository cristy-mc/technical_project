#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"


class Fra2moControllerNode : public rclcpp::Node
{
public:

  Fra2moControllerNode()
  : Node("fra2mo_controller_node"),
    x_(0.0),
    y_(0.0),
    yaw_(0.0),
    odom_received_(false),
    target_received_(false)
  {
    // ==========================================================
    // PUBLISHER VELOCITA'
    // ==========================================================

    cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10
      );


    // ==========================================================
    // ODOMETRIA
    // ==========================================================

    odom_sub_ =
      this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom",
        10,
        std::bind(
          &Fra2moControllerNode::odomCallback,
          this,
          std::placeholders::_1
        )
      );


    // ==========================================================
    // TIMER DI CONTROLLO
    // ==========================================================

    timer_ =
      this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(
          &Fra2moControllerNode::controlLoop,
          this
        )
      );


    RCLCPP_INFO(
      this->get_logger(),
      "============================================"
    );

    RCLCPP_INFO(
      this->get_logger(),
      "FRA2MO CONTROLLER AVVIATO"
    );

    RCLCPP_INFO(
      this->get_logger(),
      "Comando: /cmd_vel"
    );

    RCLCPP_INFO(
      this->get_logger(),
      "Odometria: /odom"
    );

    RCLCPP_INFO(
      this->get_logger(),
      "============================================"
    );
  }


  void setTarget(
    double x,
    double y)
  {
    target_x_ = x;
    target_y_ = y;
    target_received_ = true;


    RCLCPP_INFO(
      this->get_logger(),
      "Nuovo target rover: [%.2f %.2f]",
      target_x_,
      target_y_
    );
  }


private:

  // ==========================================================
  // ODOM CALLBACK
  // ==========================================================

  void odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    x_ =
      msg->pose.pose.position.x;

    y_ =
      msg->pose.pose.position.y;


    const auto & q =
      msg->pose.pose.orientation;


    // quaternion -> yaw
    yaw_ =
      std::atan2(
        2.0 *
          (q.w * q.z + q.x * q.y),
        1.0 -
          2.0 *
            (q.y * q.y + q.z * q.z)
      );


    odom_received_ = true;
  }


  // ==========================================================
  // NORMALIZE ANGLE
  // ==========================================================

  double normalizeAngle(
    double angle)
  {
    while (angle > M_PI)
      angle -= 2.0 * M_PI;

    while (angle < -M_PI)
      angle += 2.0 * M_PI;

    return angle;
  }


  // ==========================================================
  // CONTROL LOOP
  // ==========================================================

  void controlLoop()
  {
    geometry_msgs::msg::Twist cmd;


    // ----------------------------------------------------------
    // Aspettiamo odometria
    // ----------------------------------------------------------

    if (!odom_received_)
    {
      cmd_vel_pub_->publish(cmd);
      return;
    }


    // ----------------------------------------------------------
    // Nessun target
    // ----------------------------------------------------------

    if (!target_received_)
    {
      cmd_vel_pub_->publish(cmd);
      return;
    }


    // ----------------------------------------------------------
    // Errore posizione
    // ----------------------------------------------------------

    const double dx =
      target_x_ - x_;

    const double dy =
      target_y_ - y_;


    const double distance =
      std::sqrt(
        dx * dx +
        dy * dy
      );


    // ----------------------------------------------------------
    // Arrivato
    // ----------------------------------------------------------

    if (distance < 0.10)
    {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;

      cmd_vel_pub_->publish(cmd);

      target_received_ = false;


      RCLCPP_INFO(
        this->get_logger(),
        "TARGET RAGGIUNTO: [%.2f %.2f]",
        x_,
        y_
      );


      return;
    }


    // ----------------------------------------------------------
    // Direzione target
    // ----------------------------------------------------------

    const double desired_yaw =
      std::atan2(
        dy,
        dx
      );


    const double yaw_error =
      normalizeAngle(
        desired_yaw - yaw_
      );


    // ----------------------------------------------------------
    // Controllo angolare
    // ----------------------------------------------------------

    constexpr double K_ANGULAR = 1.5;

    constexpr double MAX_ANGULAR_SPEED = 1.0;


    cmd.angular.z =
      K_ANGULAR * yaw_error;


    cmd.angular.z =
      std::clamp(
        cmd.angular.z,
        -MAX_ANGULAR_SPEED,
        MAX_ANGULAR_SPEED
      );


    // ----------------------------------------------------------
    // Controllo lineare
    //
    // Avanziamo solamente quando siamo abbastanza orientati.
    // ----------------------------------------------------------

    constexpr double K_LINEAR = 0.6;

    constexpr double MAX_LINEAR_SPEED = 0.25;


    if (std::fabs(yaw_error) < 0.4)
    {
      cmd.linear.x =
        K_LINEAR * distance;


      cmd.linear.x =
        std::clamp(
          cmd.linear.x,
          0.0,
          MAX_LINEAR_SPEED
        );
    }
    else
    {
      cmd.linear.x = 0.0;
    }


    cmd_vel_pub_->publish(cmd);
  }


  // ==========================================================
  // MEMBERS
  // ==========================================================

  rclcpp::Publisher<
    geometry_msgs::msg::Twist
  >::SharedPtr cmd_vel_pub_;


  rclcpp::Subscription<
    nav_msgs::msg::Odometry
  >::SharedPtr odom_sub_;


  rclcpp::TimerBase::SharedPtr timer_;


  double x_;
  double y_;
  double yaw_;


  double target_x_ = 0.0;
  double target_y_ = 0.0;


  bool odom_received_;
  bool target_received_;
};


// ================================================================
// MAIN
// ================================================================

int main(
  int argc,
  char ** argv)
{
  rclcpp::init(argc, argv);


  auto node =
    std::make_shared<Fra2moControllerNode>();


  // --------------------------------------------------------------
  // Primo target di test
  // --------------------------------------------------------------

  node->setTarget(
    1.0,
    0.0
  );


  rclcpp::spin(node);


  rclcpp::shutdown();

  return 0;
}
