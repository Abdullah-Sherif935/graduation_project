#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <termios.h>   // إعدادات الـ Serial
#include <fcntl.h>     // open()
#include <unistd.h>    // close(), write()
#include <string.h>    // strerror()
#include <chrono>
#include <string>
#include <sstream>

using namespace std::chrono_literals;

class HubMotorNode : public rclcpp::Node
{
public:
    HubMotorNode()
    : Node("hub_motor_node"), serial_fd_(-1), last_r_(0), last_l_(0)
    {
        // 1) بارامترات الـ port والـ baud
        this->declare_parameter<std::string>("port", "/dev/ttyACM0");
        this->declare_parameter<int>("baudrate", 115200);

        std::string port;
        int baudrate;
        this->get_parameter("port", port);
        this->get_parameter("baudrate", baudrate);

        RCLCPP_INFO(this->get_logger(), "Trying to open serial port: %s at %d baud",
                    port.c_str(), baudrate);

        // 2) فتح الـ serial port
        serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s (error: %s)",
                         port.c_str(), strerror(errno));
        } else {
            if (!configure_port(baudrate)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to configure serial port");
                close(serial_fd_);
                serial_fd_ = -1;
            } else {
                RCLCPP_INFO(this->get_logger(), "Serial port opened and configured successfully!");
            }
        }

        // 3) Subscriber على /wheel_cmd (String: "R L")
        wheel_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/wheel_cmd",
            10,
            std::bind(&HubMotorNode::wheel_cmd_callback, this, std::placeholders::_1)
        );

        // 4) تايمر يبعت آخر قيمة استلمناها كل 100ms
        timer_ = this->create_wall_timer(
            100ms,
            std::bind(&HubMotorNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(),
                    "HubMotorNode is up. Publish to /wheel_cmd like: \"100 100\".");
    }

    ~HubMotorNode() override
    {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
            RCLCPP_INFO(this->get_logger(), "Serial port closed");
        }
    }

private:
    int serial_fd_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr wheel_cmd_sub_;

    int last_r_;
    int last_l_;
    std::string last_cmd_str_;   // النص اللي هيتبعت على السيريال (مع \n)

    bool configure_port(int baudrate)
    {
        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "tcgetattr failed: %s", strerror(errno));
            return false;
        }

        cfmakeraw(&tty);

        speed_t speed;
        switch (baudrate) {
            case 9600:   speed = B9600; break;
            case 19200:  speed = B19200; break;
            case 57600:  speed = B57600; break;
            case 115200:
            default:     speed = B115200; break;
        }

        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 5;   // 0.5 ثانية

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "tcsetattr failed: %s", strerror(errno));
            return false;
        }

        return true;
    }

    // callback: استلام أمر من /wheel_cmd
    void wheel_cmd_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        // نتوقع string بالشكل "R L"
        std::stringstream ss(msg->data);
        int r = 0, l = 0;
        if (!(ss >> r >> l)) {
            RCLCPP_WARN(this->get_logger(),
                        "Invalid /wheel_cmd format. Expected: \"R L\" (e.g. \"100 100\"), got: '%s'",
                        msg->data.c_str());
            return;
        }

        last_r_ = r;
        last_l_ = l;

        // نخزن string جاهز للإرسال، مضاف له '\n'
        last_cmd_str_ = std::to_string(last_r_) + " " + std::to_string(last_l_) + "\n";

        RCLCPP_INFO(this->get_logger(), "Updated wheel command to: R=%d L=%d", last_r_, last_l_);
    }

    void timer_callback()
    {
        if (serial_fd_ < 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Serial port not open");
            return;
        }

        if (last_cmd_str_.empty()) {
            // لو لسه ما جاش أي أمر، نخليها 0 0
            last_cmd_str_ = "0 0\n";
        }

        ssize_t written = write(serial_fd_, last_cmd_str_.c_str(), last_cmd_str_.size());
        if (written < 0) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "Failed to write to serial: %s", strerror(errno));
        } else {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Sent to hoverboard: %s", last_cmd_str_.c_str());
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HubMotorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
