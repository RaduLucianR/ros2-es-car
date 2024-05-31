#include "carFront.hpp"
#include "rclcpp/rclcpp.hpp"
#include "es_car_inter/msg/drive.hpp"

class Steering : public rclcpp::Node
{
  public:
    Steering()
    : Node("steering")
    {
      carFront_init(0, 0);
      subscription_ = this->create_subscription<es_car_inter::msg::Drive>(
      "steering_angle", 10, std::bind(&Steering::topic_callback, this, std::placeholders::_1));
    }

  private:
    void topic_callback(const es_car_inter::msg::Drive & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Steering %f degrees", msg.angle);
      setSteering(msg.angle); 
    }

    rclcpp::Subscription<es_car_inter::msg::Drive>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Steering>());
  rclcpp::shutdown();
  return 0;
}
