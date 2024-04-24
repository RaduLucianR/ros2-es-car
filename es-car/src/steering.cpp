#include "carFront.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Steering : public rclcpp::Node
{
  public:
    Steering()
    : Node("steering")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "steering_angle", 10, std::bind(&Steering::topic_callback, this, std::placeholders::_1));
    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Steering %s degrees", msg->data.c_str());
      setSteering(std::stod(msg->data)); 
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  carFront_init();
  rclcpp::spin(std::make_shared<Steering>());
  rclcpp::shutdown();
  return 0;
}
