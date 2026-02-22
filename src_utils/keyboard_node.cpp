
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <array>
#include <vector>
#include <chrono>
#include <algorithm>


using namespace std::chrono_literals;


class TermRaw {
public:
    TermRaw() {
        tcgetattr(STDIN_FILENO, &orig_);
        termios raw = orig_;
        raw.c_lflag &= ~(ICANON | ECHO);   // the charactor 1 by 1 & no echo
        raw.c_cc[VMIN]  = 0;               // non-blocking
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);

        orig_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, orig_flags_ | O_NONBLOCK);
    }
    ~TermRaw() {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_);
        fcntl(STDIN_FILENO, F_SETFL, orig_flags_);
    }

private:
    termios orig_{};
    int orig_flags_{0};
};



int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("keyboard_joy_node");
    auto pub = node-> create_publisher<sensor_msgs::msg::Joy>("/joy", rclcpp::SensorDataQoS());

    TermRaw guard;

    std::vector<float> axes(2, 0.0f);   // axes[0]=x, axes[1]=y
    std::vector<int32_t> buttons(1, 0); // buttons[0]=space

    rclcpp::WallRate rate(50);

    while (rclcpp::ok())
    {
        axes[0] = 0.0f;
        axes[1] = 0.0f;
        buttons[0] = 0;

        while (1)
        {
            char c;
            ssize_t n = ::read(STDIN_FILENO, &c, 1);
            if (n <= 0) break;
            switch (c) {
                case 'a': case 'A': 
                    axes[0] = -1.0f;
                    break;
                case 'd': case 'D': 
                    axes[0] = +1.0f;
                    break;
                case 'w': case 'W': 
                    axes[1] = +1.0f;
                    break;
                case 's': case 'S': 
                    axes[0] = -1.0f;
                    break;
                case ' ':
                    buttons[0] = 1;
                    break;
            }
        }

        sensor_msgs::msg::Joy msg;
        msg.header.stamp = node->now();
        msg.axes = axes;
        msg.buttons = buttons;
        pub->publish(msg);

        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}


// class KeyboardJoyNode : public rclcpp::Node
// {
// public: 
//     KeyboardNode(): Node("keyboard_joy_node")
//     {
//         deadman_timeout_ms_ = this -> declare_parameter<int>("deadman_timeout_ms", 300);
//         publidh_hz_ = this -> declare_parameter<int>("publish_hz", 60);
//         // topic_ = this -> declare_parameter<std::string>("topic", "/keyboard_joy")

//         axes_.assign(8, 0.0f);
//     }
// }














