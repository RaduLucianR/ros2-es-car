#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "es_car_inter/msg/drive.hpp"                                            // CHANGE

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher(double a, double s)
  : Node("minimal_publisher"), count_(0)
  {
    this->angle = a;
    this->speed = s;
    ang_ = this->create_publisher<es_car_inter::msg::Drive>("steering_angle", 10);  // CHANGE
    spd_ = this->create_publisher<es_car_inter::msg::Drive>("speed", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = es_car_inter::msg::Drive();                                   // CHANGE
    message.angle = this->angle;
    message.speed = this->speed;                                                     // CHANGE
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.angle << " " << message.speed << "'");    // CHANGE
    ang_->publish(message);
    spd_->publish(message); // This is redundant. This node publishes the same message to 2 different topics. It should just publish to 1 topic.
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<es_car_inter::msg::Drive>::SharedPtr ang_;
  rclcpp::Publisher<es_car_inter::msg::Drive>::SharedPtr spd_;

  size_t count_;
  double angle;
  double speed;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>(std::stod(argv[1]), std::stod(argv[2])));
  rclcpp::shutdown();
  return 0;
}
