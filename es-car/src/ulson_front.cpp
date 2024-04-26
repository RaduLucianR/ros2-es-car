#include <chrono>
#include "carFront.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class UltrasonicFront : public rclcpp::Node
{
  public:
    UltrasonicFront()
    : Node("ultrasonic_front")
    {
	publisher1_ = this->create_publisher<std_msgs::msg::Float64>("ultrasonic_fl", 10);
	publisher2_ = this->create_publisher<std_msgs::msg::Float64>("ultrasonic_fr", 10);
        timer1_ = this->create_wall_timer(
        500ms, std::bind(&UltrasonicFront::timer_callback1, this));
 	timer2_ = this->create_wall_timer(
        500ms, std::bind(&UltrasonicFront::timer_callback2, this));
    }

  private:
    rclcpp::TimerBase::SharedPtr timer1_;
    rclcpp::TimerBase::SharedPtr timer2_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher1_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher2_;

    void timer_callback1()
    {
        std_msgs::msg::Float64 message;
	double dist[2];

	dist[0] = 0;
	dist[1] = 0;

	getDistance(dist); 
	message.data = dist[0];

    	RCLCPP_INFO(this->get_logger(), "Left: %lf m", dist[0]);
    	publisher1_->publish(message);
    }

    void timer_callback2()
    {
        std_msgs::msg::Float64 message;
        double dist[2];

        dist[0] = 0;
        dist[1] = 0;

        getDistance(dist); 
        message.data = dist[1];

        RCLCPP_INFO(this->get_logger(), "Right: %lf m", dist[1]);
        publisher2_->publish(message);
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  carFront_init();
  rclcpp::spin(std::make_shared<UltrasonicFront>());
  rclcpp::shutdown();
  return 0;
}

