#include "carFront.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class CarStartup : public rclcpp::Node
{
  public:
    CarStartup()
    : Node("carstartup")
    {
	carFront_init(0, 0);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CarStartup>());
  rclcpp::shutdown();
  return 0;
}
