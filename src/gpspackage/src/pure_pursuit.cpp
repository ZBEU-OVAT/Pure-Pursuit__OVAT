#include <memory>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

// ============================================================
// AYARLANABILIR PARAMETRELER
// ============================================================
static constexpr double WHEELBASE        = 1.484;
static constexpr double LOOKAHEAD        = 3.0;
static constexpr double ARRIVAL_DIST     = 1.5;
static constexpr double STEER_MULTIPLIER = 1.2;
static constexpr int    STEER_MIN        = 25;
static constexpr int    STEER_MAX        = 165;
static constexpr int    STEER_CENTER     = 100;

// Durma noktası: -1 = kapalı, 0-N = hangi WP indeksinde dursun
static constexpr int    STOP_WP_INDEX    = 15;
static constexpr double STOP_DURATION_S  = 7.0;

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
      has_gps_yaw_(false),
      current_lat_(0.0),
      current_lon_(0.0),
      current_yaw_(0.0),
      gps_status_(0),
      waiting_(false),
      stop_triggered_(false)
    {
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/mavros/global_position/global", rclcpp::SensorDataQoS(),
            std::bind(&PurePursuit::gps_callback, this, std::placeholders::_1));

        steering_pub_ = this->create_publisher<std_msgs::msg::Int32>("steering_angle_topic", 10);
        speed_client_  = this->create_client<std_srvs::srv::Trigger>("speed_on");
        brake_client_  = this->create_client<std_srvs::srv::Trigger>("brake_on");

        speed_timer_ = this->create_wall_timer(
            500ms, std::bind(&PurePursuit::speed_timer_cb, this));

        load_waypoints("/home/muhaliugur/gpsurus/src/gpspackage/config/waypoints_gps.csv");
        RCLCPP_INFO(this->get_logger(), "SISTEM AKTIF. GPS BEKLENIYOR...");
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

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        if (msg->latitude == 0.0 && msg->longitude == 0.0) return;
        if (msg->status.status < 0) return;

        if (has_gps_) {
            double dx, dy, dist;
            gps_to_enu(current_lat_, current_lon_, msg->latitude, msg->longitude, dx, dy, dist);
            if (dist > 0.3) {
                current_yaw_ = std::atan2(dy, dx);
                has_gps_yaw_ = true;
            }
        }

        current_lat_ = msg->latitude;
        current_lon_ = msg->longitude;
        gps_status_  = msg->status.status;
        has_gps_ = true;

        compute_steering();
    }

    void initialize()
    {
        double min_any = std::numeric_limits<double>::max();
        size_t best_any = 0;

        for (size_t i = 0; i < waypoints_.size(); i++) {
            double dx, dy, d;
            gps_to_enu(current_lat_, current_lon_, waypoints_[i][0], waypoints_[i][1], dx, dy, d);
            if (d < min_any) { min_any = d; best_any = i; }
        }

        current_target_index_ = best_any;
        initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "BASLANGIC: WP %zu (%.2fm)", current_target_index_, min_any);
    }

    void compute_steering()
    {
        if (waypoints_.empty()) return;
        if (!initialized_) initialize();

        if (waiting_) {
            call_brake();
            double elapsed = (this->now() - wait_start_).seconds();
            if (elapsed < STOP_DURATION_S) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "BEKLIYOR: %.1f / %.0f sn", elapsed, STOP_DURATION_S);
                return;
            }
            waiting_ = false;
            RCLCPP_INFO(this->get_logger(), "Bekleme bitti, devam ediliyor.");
        }

        while (current_target_index_ < waypoints_.size()) {
            double dx, dy, d;
            gps_to_enu(current_lat_, current_lon_, waypoints_[current_target_index_][0], waypoints_[current_target_index_][1], dx, dy, d);
            if (d > ARRIVAL_DIST) break;
            current_target_index_++;
        }

        if (current_target_index_ >= waypoints_.size()) {
            if (is_moving_) {
                call_brake();
                is_moving_ = false;
                RCLCPP_INFO(this->get_logger(), "VARIS! Tum WP tamamlandi.");
            }
            return;
        }

        double final_dx, final_dy, final_d;
        gps_to_enu(current_lat_, current_lon_, waypoints_.back()[0], waypoints_.back()[1], final_dx, final_dy, final_d);
        if (final_d < ARRIVAL_DIST) {
            if (is_moving_) {
                call_brake();
                is_moving_ = false;
                RCLCPP_INFO(this->get_logger(), "VARIS! Son hedefe %.2fm kaldi. FREN.", final_d);
            }
            return;
        }

        if (STOP_WP_INDEX >= 0 && !stop_triggered_ && (int)current_target_index_ >= STOP_WP_INDEX) {
            stop_triggered_ = true;
            is_moving_ = false;
            waiting_ = true;
            wait_start_ = this->now();
            call_brake();
            RCLCPP_INFO(this->get_logger(),
                "DURUYOR: WP %d seviyesinde %.0f sn bekleniyor...", STOP_WP_INDEX, STOP_DURATION_S);
            return;
        }

        is_moving_ = true;

        if (!has_gps_yaw_) {
            std_msgs::msg::Int32 s_msg;
            s_msg.data = STEER_CENTER;
            steering_pub_->publish(s_msg);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "GPS_YAW bekleniyor, duz gidiliyor...");
            return;
        }

        size_t lookahead_idx = current_target_index_;
        for (size_t i = current_target_index_; i < waypoints_.size(); i++) {
            double dx, dy, ld;
            gps_to_enu(current_lat_, current_lon_, waypoints_[i][0], waypoints_[i][1], dx, dy, ld);
            if (ld >= LOOKAHEAD) {
                lookahead_idx = i;
                break;
            }
        }

        double tx, ty, td;
        gps_to_enu(current_lat_, current_lon_, waypoints_[lookahead_idx][0], waypoints_[lookahead_idx][1], tx, ty, td);

        double target_angle = std::atan2(ty, tx);
        double alpha = angle_diff(target_angle, current_yaw_);

        int skip_count = 0;
        while (std::abs(alpha) > (60.0 * DEG2RAD) && current_target_index_ + 1 < waypoints_.size() && skip_count < 2) {
            RCLCPP_WARN(this->get_logger(), "WP %zu geride kaldi, atlaniyor!", current_target_index_);
            current_target_index_++;
            gps_to_enu(current_lat_, current_lon_, waypoints_[current_target_index_][0], waypoints_[current_target_index_][1], tx, ty, td);
            target_angle = std::atan2(ty, tx);
            alpha = angle_diff(target_angle, current_yaw_);
            skip_count++;
        }

        double delta   = std::atan((2.0 * WHEELBASE * std::sin(alpha)) / LOOKAHEAD);
        double steer_f = STEER_CENTER - (delta * 180.0 / M_PI * STEER_MULTIPLIER);
        int steer      = std::clamp(static_cast<int>(steer_f), STEER_MIN, STEER_MAX);

        std_msgs::msg::Int32 s_msg;
        s_msg.data = steer;
        steering_pub_->publish(s_msg);

        RCLCPP_INFO(this->get_logger(),
                    "[STEER: %d] Mesafe: %.2fm | WP: %zu/%zu | Yaw: %.1f | Alpha: %.1f | GPS:%d [GPS_YAW]",
                    steer, final_d, current_target_index_, waypoints_.size(),
                    current_yaw_ * 180.0 / M_PI, alpha * 180.0 / M_PI, gps_status_);
    }

    void speed_timer_cb()
    {
        if (waiting_) {
            call_brake();
            return;
        }
        if (!is_moving_) return;
        if (!speed_client_->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "speed_on servisi hazir degil! teensy_comm calisiyor mu?");
            return;
        }
        speed_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    }

    void load_waypoints(const std::string& path)
    {
        std::ifstream file(path);
        if (!file.is_open()) return;
        std::string line;
        bool first_line = true;
        while (std::getline(file, line)) {
            if (line.empty()) continue;
            if (first_line) { first_line = false; if (std::isalpha(line[0])) continue; }
            std::replace(line.begin(), line.end(), ',', ' ');
            std::istringstream ss(line);
            double lat, lon;
            if (!(ss >> lat >> lon)) continue;
            waypoints_.push_back({lat, lon});
        }
        RCLCPP_INFO(this->get_logger(), "%zu waypoint yuklendi.", waypoints_.size());
    }

    void call_brake()
    {
        if (!brake_client_->service_is_ready()) return;
        brake_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr           steering_pub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr            speed_client_, brake_client_;
    rclcpp::TimerBase::SharedPtr                                 speed_timer_;

    std::vector<std::vector<double>> waypoints_;
    size_t  current_target_index_;
    bool    is_moving_, initialized_, has_gps_, has_gps_yaw_;
    double  current_lat_, current_lon_, current_yaw_;
    int     gps_status_;
    bool    waiting_;
    bool    stop_triggered_;
    rclcpp::Time wait_start_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}
