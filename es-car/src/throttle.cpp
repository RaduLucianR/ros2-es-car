#include "carBack.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Throttle : public rclcpp::Node
{
  public:
    Throttle()
    : Node("throttle")
    {
      carBack_init(1, 1); // Chip select 1, i.e. GPIO 17 on a RPi 4+ B aka 6th pin on the left-most ping column
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "speed", 10, std::bind(&Throttle::topic_callback, this, std::placeholders::_1));
    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Setting speed to %s [min must be ~395]", msg->data.c_str());
      setEsc(std::stod(msg->data)); 
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};


// Main can be commented out and the node can be put in a separate .hpp
// Problem still is that carBack_init() would be called every time a new thing using that SPI is needed
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
//  carBack_init(1, 1);
  rclcpp::spin(std::make_shared<Throttle>());
  rclcpp::shutdown();
  return 0;
}
