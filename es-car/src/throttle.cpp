#include "carBack.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Throttle : public rclcpp::Node
{
  public:
    Throttle()
    : Node("throttle")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "speed", 10, std::bind(&Throttle::topic_callback, this, std::placeholders::_1));
    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Setting speed to %s", msg->data.c_str());
      setEsc(std::stod(msg->data)); 
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  carBack_init();
  rclcpp::spin(std::make_shared<Throttle>());
  rclcpp::shutdown();
  return 0;
}
