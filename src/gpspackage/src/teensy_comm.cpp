#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <glob.h>
#include <cmath>
#include <string>
#include <vector>
#include <mutex>

int serial_fd = -1;
std::mutex serial_mutex;

// ---------------- ACM PORT TARAMA ----------------
std::vector<std::string> findAcmPorts()
{
    glob_t g{};
    std::vector<std::string> ports;
    if (glob("/dev/ttyACM*", 0, nullptr, &g) == 0) {
        for (size_t i = 0; i < g.gl_pathc; i++)
            ports.emplace_back(g.gl_pathv[i]);
    }
    globfree(&g);
    return ports;
}

// ---------------- SERIAL PORT AÇMA ----------------
bool openSerial(const std::string &port)
{
    serial_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0) return false;

    struct termios tty{};
    if (tcgetattr(serial_fd, &tty) != 0) return false;

    // Teensy sabit hızı: 4800 Baud
    cfsetospeed(&tty, B4800);
    cfsetispeed(&tty, B4800);

    // stty raw ile aynı etkiyi yapar - tüm özel karakter işlemlerini kapatır
    // Binary veri gönderirken \x00 gibi byte'ların kaybolmasını önler
    cfmakeraw(&tty);

    // Baud rate'i cfmakeraw sonra tekrar set et (cfmakeraw sıfırlayabilir)
    cfsetospeed(&tty, B4800);
    cfsetispeed(&tty, B4800);

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;

    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) return false;
    return true;
}

// ---------------- SERIAL YAZMA (Teensy için 1 Byte — bufferSize=1) ----------------
bool sendByteToTeensy(uint8_t value)
{
    std::lock_guard<std::mutex> lock(serial_mutex);
    if (serial_fd < 0) return false;

    ssize_t written = write(serial_fd, &value, 1);
    return (written == 1);
}

void sendRepeated(uint8_t value)
{
    for (int i = 0; i < 3; i++)
    {
        sendByteToTeensy(value);
        usleep(10000); // 10 ms bekleme
    }
}

class SerialCommNode : public rclcpp::Node
{
public:
    SerialCommNode() : Node("serial_comm_node")
    {
        auto ports = findAcmPorts();
        bool connected = false;
        for (const auto &port : ports) {
            if (openSerial(port)) {
                RCLCPP_INFO(this->get_logger(), "Teensy bulundu: %s", port.c_str());
                connected = true;
                break;
            }
            RCLCPP_WARN(this->get_logger(), "%s acilamadi, sonraki deneniyor...", port.c_str());
        }
        if (!connected) {
            RCLCPP_FATAL(this->get_logger(), "Hicbir ACM portu acilamadi! Kablo bagli mi?");
            return;
        }

        steering_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "steering_angle_topic", 10, std::bind(&SerialCommNode::steeringCallback, this, std::placeholders::_1));

        speed_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "speed_on", std::bind(&SerialCommNode::speedService, this, std::placeholders::_1, std::placeholders::_2));

        brake_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "brake_on", std::bind(&SerialCommNode::brakeService, this, std::placeholders::_1, std::placeholders::_2));

        steering_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&SerialCommNode::steeringTimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Serial Comm Node (4800 Baud, RAW Mode) HAZIR");
    }

    ~SerialCommNode() { if (serial_fd >= 0) close(serial_fd); }

private:
    void steeringCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        last_steering_ = msg->data;
        steering_updated_ = true;
    }

    void steeringTimerCallback() {
        if (!steering_updated_) return;
        sendByteToTeensy(static_cast<uint8_t>(last_steering_));
        steering_updated_ = false;
    }

    void speedService(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        sendRepeated(201);
        res->success = true;
        RCLCPP_WARN(this->get_logger(), "GAZ KOMUTU GÖNDERİLDİ (201)");
    }

    void brakeService(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        sendRepeated(202);
        res->success = true;
        RCLCPP_WARN(this->get_logger(), "FREN KOMUTU GÖNDERİLDİ (202)");
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr steering_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr speed_srv_, brake_srv_;
    rclcpp::TimerBase::SharedPtr steering_timer_;
    int last_steering_{90};
    bool steering_updated_{false};
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialCommNode>());
    rclcpp::shutdown();
    return 0;
}
