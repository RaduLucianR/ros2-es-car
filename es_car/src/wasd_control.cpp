#include <iostream>
#include <termios.h>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "es_car_inter/msg/drive.hpp"

double angle = 0.0;
double speed = 0.0;

class KeyboardNode : public rclcpp::Node {
public:
    KeyboardNode() : Node("keyboard_node") {
        ang_ = this->create_publisher<es_car_inter::msg::Drive>("steering_angle", 10);
	spd_ = this->create_publisher<es_car_inter::msg::Drive>("speed", 10);
        setRawMode(true);
        publishKeyPresses();
        setRawMode(false);
    }

    ~KeyboardNode() {
        setRawMode(false);
    }

private:
    void setRawMode(bool enable) {
        static struct termios oldt;
        struct termios newt;

        if (enable) {
            tcgetattr(STDIN_FILENO, &oldt);  // Save old settings
            newt = oldt;
            newt.c_lflag &= ~(ICANON | ECHO);  // Disable buffering and echo
            tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // Apply new settings
        } else {
            tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // Restore old settings
        }
    }

    void publishKeyPresses() {
        char input;
        while (rclcpp::ok()) {
            input = std::cin.get();
            es_car_inter::msg::Drive message = es_car_inter::msg::Drive();
            message.angle = 0.0;
            message.speed = 0.0;

            if (input == 'w')
            {
               if (speed < 138.0) {
                 speed = 138.0;
               } else if (speed + 1.0 < 180) {
                 speed += 1.0;
               } else {
                 RCLCPP_INFO(this->get_logger(), "Maximum forward speed reached!");
               }

               message.speed = speed;
               spd_->publish(message);
            } else if (input == 's') {
                 speed = 130.0;
                 message.speed = speed;
                 spd_->publish(message);
            } else if (input == 'b') {
                 speed = 80.0;
                 message.speed = speed;
                 spd_->publish(message);
            } else if (input == 'x') {
               if (speed == 130.0) {
                 speed = 110.0;
               } else if (speed < 130.0 && speed - 1.0 > 80.0) {
                 speed -= 1.0;
               } else {
                 RCLCPP_INFO(this->get_logger(), "Maximum backward speed reached!");
               }

               message.speed = speed;
               spd_->publish(message);
            } else if (input == 'a') {
               angle += 20;
               message.angle = angle;
               ang_->publish(message);
            } else if (input == 'd') {
               angle -= 20;
               message.angle = angle;
               ang_->publish(message);
            } else {
               RCLCPP_INFO(this->get_logger(), "You pressed a wrong key!");
            }
        }
    }

    rclcpp::Publisher<es_car_inter::msg::Drive>::SharedPtr ang_;
    rclcpp::Publisher<es_car_inter::msg::Drive>::SharedPtr spd_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardNode>());
    rclcpp::shutdown();
    return 0;
}
