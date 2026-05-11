#include <memory>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

// ============================================================
// AYARLANABILIR PARAMETRELER
// ============================================================
static constexpr double WHEELBASE        = 1.484;
static constexpr double LOOKAHEAD        = 4.0;
static constexpr double ARRIVAL_DIST     = 2.0;
static constexpr double STEER_MULTIPLIER = 1.0;
static constexpr int    STEER_MIN        = 35;
static constexpr int    STEER_MAX        = 130;
static constexpr int    STEER_CENTER     = 90;

// WGS84
static constexpr double EARTH_R = 6378137.0;
static constexpr double DEG2RAD = M_PI / 180.0;

class PurePursuit : public rclcpp::Node
{
public:
    PurePursuit()
    : Node("pure_pursuit_node"),
      current_target_index_(0),
      is_moving_(false),
      initialized_(false),
      has_gps_(false),
      has_yaw_(false),
      current_lat_(0.0),
      current_lon_(0.0),
      current_yaw_(0.0)
    {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/mavros/local_position/odom", rclcpp::SensorDataQoS(),
            std::bind(&PurePursuit::odom_callback, this, std::placeholders::_1));

        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/mavros/global_position/global", rclcpp::SensorDataQoS(),
            std::bind(&PurePursuit::gps_callback, this, std::placeholders::_1));

        steering_pub_ = this->create_publisher<std_msgs::msg::Int32>("steering_angle_topic", 10);
        speed_client_  = this->create_client<std_srvs::srv::Trigger>("speed_on");
        brake_client_  = this->create_client<std_srvs::srv::Trigger>("brake_on");

        speed_timer_ = this->create_wall_timer(
            500ms, std::bind(&PurePursuit::speed_timer_cb, this));

        load_waypoints("/home/muhaliugur/gpsurus/src/gpspackage/config/waypoints_gps.csv");
        RCLCPP_INFO(this->get_logger(), "SISTEM AKTIF. RTK GPS + MAVROS YAW BEKLENIYOR...");
        RCLCPP_INFO(this->get_logger(), "Lookahead: %.1fm | Varis esigi: %.1fm", LOOKAHEAD, ARRIVAL_DIST);
    }

private:

    void gps_to_enu(double lat1, double lon1, double lat2, double lon2,
                    double &dx, double &dy, double &dist)
    {
        double dlat = (lat2 - lat1) * DEG2RAD;
        double dlon = (lon2 - lon1) * DEG2RAD;
        dx   = dlon * EARTH_R * std::cos(lat1 * DEG2RAD);
        dy   = dlat * EARTH_R;
        dist = std::hypot(dx, dy);
    }

    double angle_diff(double a, double b)
    {
        double d = a - b;
        while (d > M_PI)  d -= 2.0 * M_PI;
        while (d < -M_PI) d += 2.0 * M_PI;
        return d;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
                             msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
        double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
                                    msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
        double raw_yaw = std::atan2(siny, cosy);

        // ArduRover + MAVROS yaw 180 derece ters geliyor — düzeltiyoruz
        current_yaw_ = raw_yaw + M_PI;
        while (current_yaw_ > M_PI)  current_yaw_ -= 2.0 * M_PI;
        while (current_yaw_ < -M_PI) current_yaw_ += 2.0 * M_PI;

        has_yaw_ = true;
    }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        if (msg->latitude == 0.0 && msg->longitude == 0.0) return;
        if (msg->status.status < 0) return;

        current_lat_ = msg->latitude;
        current_lon_ = msg->longitude;
        has_gps_ = true;

        if (has_yaw_) {
            compute_steering();
        }
    }

    void initialize()
    {
        // En yakın waypoint'i bul
        double min_dist = std::numeric_limits<double>::max();
        size_t closest = 0;
        for (size_t i = 0; i < waypoints_.size(); i++) {
            double dx, dy, d;
            gps_to_enu(current_lat_, current_lon_,
                       waypoints_[i][0], waypoints_[i][1],
                       dx, dy, d);
            if (d < min_dist) {
                min_dist = d;
                closest = i;
            }
        }

        // İleri ve geri yön açılarını hesapla
        double forward_angle  = current_yaw_;
        double backward_angle = current_yaw_;

        if (closest + 1 < waypoints_.size()) {
            double dx, dy, d;
            gps_to_enu(waypoints_[closest][0], waypoints_[closest][1],
                       waypoints_[closest + 1][0], waypoints_[closest + 1][1],
                       dx, dy, d);
            forward_angle = std::atan2(dy, dx);
        }

        if (closest > 0) {
            double dx, dy, d;
            gps_to_enu(waypoints_[closest][0], waypoints_[closest][1],
                       waypoints_[closest - 1][0], waypoints_[closest - 1][1],
                       dx, dy, d);
            backward_angle = std::atan2(dy, dx);
        }

        double forward_diff  = std::abs(angle_diff(current_yaw_, forward_angle));
        double backward_diff = std::abs(angle_diff(current_yaw_, backward_angle));

        if (backward_diff < forward_diff) {
            std::reverse(waypoints_.begin(), waypoints_.end());
            closest = waypoints_.size() - 1 - closest;
            RCLCPP_WARN(this->get_logger(),
                "TERS YON TESPIT EDILDI! Waypoint listesi otomatik tersine cevrildi.");
        } else {
            RCLCPP_INFO(this->get_logger(), "YON DOGRU. Liste oldugu gibi kullaniliyor.");
        }

        current_target_index_ = closest;
        initialized_ = true;

        RCLCPP_INFO(this->get_logger(),
                    "BASLANGIC: Hedef=%zu | Mesafe=%.2fm | Yaw=%.1f deg",
                    closest, min_dist, current_yaw_ * 180.0 / M_PI);
    }

    void compute_steering()
    {
        if (waypoints_.empty()) return;

        if (!initialized_) {
            initialize();
        }

        // Geçilen waypoint'leri atla
        while (current_target_index_ < waypoints_.size()) {
            double dx, dy, d;
            gps_to_enu(current_lat_, current_lon_,
                       waypoints_[current_target_index_][0],
                       waypoints_[current_target_index_][1],
                       dx, dy, d);
            if (d > 2.0) break;
            RCLCPP_INFO(this->get_logger(), ">>> Hedef %zu gecildi (%.2fm)!", current_target_index_, d);
            current_target_index_++;
        }

        // Tüm hedefler bitti mi?
        if (current_target_index_ >= waypoints_.size()) {
            if (is_moving_) {
                call_brake();
                is_moving_ = false;
                RCLCPP_INFO(this->get_logger(), "VARIS! TUM HEDEFLER TAMAMLANDI. FREN.");
            }
            return;
        }

        // Son hedefe yakın mıyız?
        {
            double dx, dy, d;
            gps_to_enu(current_lat_, current_lon_,
                       waypoints_.back()[0], waypoints_.back()[1],
                       dx, dy, d);
            if (current_target_index_ >= waypoints_.size() - 1 && d < ARRIVAL_DIST) {
                if (is_moving_) {
                    call_brake();
                    is_moving_ = false;
                    RCLCPP_INFO(this->get_logger(), "VARIS! Son hedefe %.2fm kaldi. FREN.", d);
                }
                return;
            }
        }

        is_moving_ = true;

        // Lookahead hedefini bul
        size_t lookahead_idx = waypoints_.size() - 1;
        for (size_t i = current_target_index_; i < waypoints_.size(); i++) {
            double dx, dy, ld;
            gps_to_enu(current_lat_, current_lon_,
                       waypoints_[i][0], waypoints_[i][1],
                       dx, dy, ld);
            if (ld >= LOOKAHEAD) {
                lookahead_idx = i;
                break;
            }
        }

        // Hedefe yön vektörü
        double tx, ty, td;
        gps_to_enu(current_lat_, current_lon_,
                   waypoints_[lookahead_idx][0],
                   waypoints_[lookahead_idx][1],
                   tx, ty, td);

        double target_angle = std::atan2(ty, tx);
        double alpha = angle_diff(target_angle, current_yaw_);

        double ld_use  = std::max(td, LOOKAHEAD);
        double delta   = std::atan((2.0 * WHEELBASE * std::sin(alpha)) / ld_use);
        double steer_f = STEER_CENTER - (delta * 180.0 / M_PI * STEER_MULTIPLIER);
        int steer      = std::clamp(static_cast<int>(steer_f), STEER_MIN, STEER_MAX);

        std_msgs::msg::Int32 s_msg;
        s_msg.data = steer;
        steering_pub_->publish(s_msg);

        RCLCPP_INFO(this->get_logger(),
                    "Ref:%zu Hedef:%zu | Dist:%.2fm | Yaw:%.1f | Target:%.1f | Alpha:%.1f | Steer:%d",
                    current_target_index_, lookahead_idx,
                    td,
                    current_yaw_  * 180.0 / M_PI,
                    target_angle  * 180.0 / M_PI,
                    alpha         * 180.0 / M_PI,
                    steer);
    }

    void speed_timer_cb()
    {
        if (!is_moving_) return;
        if (!speed_client_->service_is_ready()) return;
        speed_client_->async_send_request(
            std::make_shared<std_srvs::srv::Trigger::Request>());
    }

    void load_waypoints(const std::string& path)
    {
        std::ifstream file(path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "WAYPOINT DOSYASI ACILAMADI: %s", path.c_str());
            return;
        }
        std::string line;
        bool first_line = true;
        while (std::getline(file, line)) {
            if (line.empty()) continue;
            if (first_line) {
                first_line = false;
                if (std::isalpha(line[0])) continue;
            }
            std::replace(line.begin(), line.end(), ',', ' ');
            std::istringstream ss(line);
            double lat, lon;
            if (!(ss >> lat >> lon)) continue;
            waypoints_.push_back({lat, lon});
        }
        if (waypoints_.empty())
            RCLCPP_ERROR(this->get_logger(), "WAYPOINT DOSYASI BOS!");
        else
            RCLCPP_INFO(this->get_logger(), "WAYPOINT YUKLENDI: %zu nokta", waypoints_.size());
    }

    void call_brake()
    {
        if (!brake_client_->service_is_ready()) return;
        brake_client_->async_send_request(
            std::make_shared<std_srvs::srv::Trigger::Request>());
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr            steering_pub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr             speed_client_, brake_client_;
    rclcpp::TimerBase::SharedPtr                                  speed_timer_;

    std::vector<std::vector<double>> waypoints_;
    size_t  current_target_index_;
    bool    is_moving_;
    bool    initialized_;
    bool    has_gps_;
    bool    has_yaw_;
    double  current_lat_;
    double  current_lon_;
    double  current_yaw_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}
