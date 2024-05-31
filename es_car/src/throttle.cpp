#include "carBack.hpp"
#include "rclcpp/rclcpp.hpp"
#include "es_car_inter/msg/drive.hpp"

class Throttle : public rclcpp::Node
{
  public:
    Throttle()
    : Node("throttle")
    {
      carBack_init(1, 1); // Chip select 1, i.e. GPIO 17 on a RPi 4+ B aka 6th pin on the left-most pin column
      subscription_ = this->create_subscription<es_car_inter::msg::Drive>(
      "speed", 10, std::bind(&Throttle::topic_callback, this, std::placeholders::_1));
    }

  private:
    void topic_callback(const es_car_inter::msg::Drive & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Setting speed to %f [min must be ~395]", msg.speed);
      setEsc(msg.speed); 
    }

    rclcpp::Subscription<es_car_inter::msg::Drive>::SharedPtr subscription_;
};


// Main can be commented out and the node can be put in a separate .hpp
// Problem still is that carBack_init() would be called every time a new thing using that SPI is needed
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Throttle>());
  rclcpp::shutdown();
  return 0;
}
