#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <vector>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <chrono>

class RPLidarTcpNode : public rclcpp::Node
{
public:
    RPLidarTcpNode() : Node("rplidar_tcp_node"),
        measurement_counter_(0), last_valid_angle_(0.0), has_last_angle_(false)
    {
        // Параметры
        this->declare_parameter<std::string>("frame_id", "laser");
        this->declare_parameter<double>("scan_frequency", 10.0);
        this->declare_parameter<double>("range_min", 0.05);
        this->declare_parameter<double>("range_max", 8.0);
        this->declare_parameter<double>("angle_offset", 0.0);
        this->declare_parameter<bool>("flip_scan", true);
        this->declare_parameter<bool>("enable_sor_filter", true);
        this->declare_parameter<int>("sor_k_neighbors", 3);
        this->declare_parameter<double>("sor_std_dev_multiplier", 0.1);

        frame_id_ = this->get_parameter("frame_id").as_string();
        scan_frequency_ = this->get_parameter("scan_frequency").as_double();
        range_min_ = this->get_parameter("range_min").as_double();
        range_max_ = this->get_parameter("range_max").as_double();
        angle_offset_ = this->get_parameter("angle_offset").as_double();
        flip_scan_ = this->get_parameter("flip_scan").as_bool();
        enable_sor_filter_ = this->get_parameter("enable_sor_filter").as_bool();
        sor_k_neighbors_ = this->get_parameter("sor_k_neighbors").as_int();
        sor_std_dev_multiplier_ = this->get_parameter("sor_std_dev_multiplier").as_double();

        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
        sub_raw_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/rplidar/raw", 100,
            std::bind(&RPLidarTcpNode::rawCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "RPLidar Node: subscrisssbed to /rplidar/raw");
    }

private:
    struct RPLidarMeasurement {
        bool new_scan;
        uint8_t quality;
        double angle;
        double distance;
        double x, y, z;
    };

    // Members
    std::string frame_id_;
    double scan_frequency_;
    double range_min_, range_max_;
    double angle_offset_;
    bool flip_scan_;
    bool enable_sor_filter_;
    int sor_k_neighbors_;
    double sor_std_dev_multiplier_;

    std::vector<uint8_t> buffer_;
    std::vector<RPLidarMeasurement> current_scan_;
    std::chrono::steady_clock::time_point last_scan_time_{std::chrono::steady_clock::now()};
    int measurement_counter_;
    double last_valid_angle_;
    bool has_last_angle_;
    std::chrono::steady_clock::time_point last_angle_time_;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_raw_;

    // ===== RAW callback =====
    void rawCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
        if (msg->data.empty()) return;
        buffer_.insert(buffer_.end(), msg->data.begin(), msg->data.end());
        processRPLidarDataImproved(buffer_);
    }

    // ===== Parser =====
    void processRPLidarDataImproved(std::vector<uint8_t>& buffer)
    {
        const size_t packet_size = 5;

        while (buffer.size() >= packet_size) {
            RPLidarMeasurement measurement;
            bool parsed_successfully = false;

            if (parseRPLidarPacketImproved(&buffer[0], measurement)) {
                parsed_successfully = true;
            } else {
                bool found_sync = false;
                for (size_t i = 1; i < std::min(buffer.size() - packet_size + 1, static_cast<size_t>(20)); i++) {
                    if (parseRPLidarPacketImproved(&buffer[i], measurement)) {
                        buffer.erase(buffer.begin(), buffer.begin() + i);
                        parsed_successfully = true;
                        found_sync = true;
                        break;
                    }
                }
                if (!found_sync) {
                    buffer.erase(buffer.begin());
                    continue;
                }
            }

            if (parsed_successfully) {
                if (!isAngleJumpReasonable(measurement.angle)) {
                    buffer.erase(buffer.begin(), buffer.begin() + packet_size);
                    continue;
                }
                updateLastValidAngle(measurement.angle);

                bool is_valid_measurement = (measurement.distance > 5.0 &&
                                             measurement.distance >= range_min_ * 1000 &&
                                             measurement.distance <= 5500.0 &&
                                             measurement.distance != 128.0 &&
                                             measurement.quality >= 5);

                if (is_valid_measurement) {
                    double angle_rad = measurement.angle * M_PI / 180.0;
                    double distance_m = measurement.distance / 1000.0;
                    measurement.x = distance_m * cos(angle_rad);
                    measurement.y = distance_m * sin(angle_rad);
                    measurement.z = 0.0;
                    current_scan_.push_back(measurement);
                }

                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_scan_time_);
                if (elapsed.count() >= 100 || current_scan_.size() >= 300) {
                    if (current_scan_.size() >= 20) {
                        publishScan();
                        current_scan_.clear();
                        last_scan_time_ = now;
                    } else {
                        // очистка и ожидание ещё данных
                        current_scan_.clear();
                        last_scan_time_ = now;
                    }
                }

                buffer.erase(buffer.begin(), buffer.begin() + packet_size);
            }

            if (buffer.size() > 100000) {
                RCLCPP_WARN(this->get_logger(), "Buffer huge, clear");
                buffer.clear();
                break;
            }
        }
    }

    bool parseRPLidarPacketImproved(const uint8_t* data, RPLidarMeasurement& m)
    {
        uint8_t b0 = data[0], b1 = data[1], b2 = data[2], b3 = data[3], b4 = data[4];
        bool start_flag = (b0 & 0x01) != 0;
        bool inv_start  = ((b0 >> 1) & 0x01) != 0;
        if (start_flag == inv_start) return false;
        if ((b1 & 0x01) == 0) return false;

        if (b0 == b1 && b1 == b2 && b2 == b3 && b3 == b4) return false;
        if ((b0 == 0xFF && b1 == 0xFF) || (b0 == 0x00 && b1 == 0x00 && b2 == 0x00)) return false;
        if (b1 > 0xFE || b2 > 0xFE) return false;

        uint16_t raw_word = (b1 << 8) | b2;
        if (raw_word == 0xFFFF || raw_word == 0x0000) return false;

        m.quality = (b0 >> 2) & 0x3F;
        if (m.quality < 3) return false;

        uint16_t angle_raw = ((b1 >> 1) & 0x7F) | ((b2 & 0xFF) << 7);
        if (angle_raw > 23040) return false;

        double ang = (double)angle_raw / 64.0;
        while (ang >= 360.0) ang -= 360.0;
        while (ang < 0.0) ang += 360.0;
        m.angle = ang;

        uint16_t distance_raw = b3 | (b4 << 8);
        m.distance = (double)distance_raw / 4.0;
        if (m.distance > 8000.0 || m.distance < 0.0) return false;

        m.new_scan = start_flag;
        return true;
    }

    bool isAngleJumpReasonable(double new_angle) {
        if (!has_last_angle_) return true;
        auto now = std::chrono::steady_clock::now();
        auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_angle_time_);
        if (dt.count() > 1000) { has_last_angle_ = false; return true; }

        double diff = new_angle - last_valid_angle_;
        if (diff > 180.0) diff -= 360.0;
        else if (diff < -180.0) diff += 360.0;

        return std::abs(diff) <= 120.0;
    }
    void updateLastValidAngle(double angle) {
        last_valid_angle_ = angle;
        has_last_angle_ = true;
        last_angle_time_ = std::chrono::steady_clock::now();
    }

    double euclideanDistance3D(const RPLidarMeasurement& a, const RPLidarMeasurement& b) {
        double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    std::vector<size_t> findKNearestNeighbors(const std::vector<RPLidarMeasurement>& points, size_t qi, int k)
    {
        struct DI { double d; size_t i; bool operator<(const DI& o) const { return d < o.d; } };
        std::vector<DI> dists; dists.reserve(points.size());
        for (size_t i = 0; i < points.size(); i++) {
            if (i == qi) continue;
            dists.push_back({euclideanDistance3D(points[qi], points[i]), i});
        }
        int use = std::min(k, (int)dists.size());
        std::partial_sort(dists.begin(), dists.begin() + use, dists.end());
        std::vector<size_t> idx; idx.reserve(use);
        for (int i = 0; i < use; i++) idx.push_back(dists[i].i);
        return idx;
    }

    void applySORFilter(std::vector<RPLidarMeasurement>& scan)
    {
        if (!enable_sor_filter_ || scan.size() < (size_t)sor_k_neighbors_) return;

        std::vector<double> mean_d; mean_d.reserve(scan.size());
        for (size_t i = 0; i < scan.size(); i++) {
            auto neigh = findKNearestNeighbors(scan, i, sor_k_neighbors_);
            double sum = 0.0;
            for (auto j : neigh) sum += euclideanDistance3D(scan[i], scan[j]);
            mean_d.push_back(neigh.empty() ? 0.0 : sum / neigh.size());
        }
        double mu = std::accumulate(mean_d.begin(), mean_d.end(), 0.0) / mean_d.size();
        double var = 0.0; for (auto v: mean_d) { double dv = v - mu; var += dv*dv; }
        double sd = std::sqrt(var / mean_d.size());
        double thr = mu + sor_std_dev_multiplier_ * sd;

        std::vector<RPLidarMeasurement> filtered;
        filtered.reserve(scan.size());
        for (size_t i = 0; i < scan.size(); i++) {
            if (mean_d[i] < thr) filtered.push_back(scan[i]);
        }
        scan.swap(filtered);
    }

    void publishScan()
    {
        if (current_scan_.empty()) return;

        applySORFilter(current_scan_);
        if (current_scan_.empty()) return;

        auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
        scan_msg->header.stamp = this->now();
        scan_msg->header.frame_id = frame_id_;
        scan_msg->angle_min = -M_PI;
        scan_msg->angle_max = M_PI;
        scan_msg->angle_increment = 2.0 * M_PI / 360.0;
        scan_msg->time_increment = 1.0 / (scan_frequency_ * 360.0);
        scan_msg->scan_time = 1.0 / scan_frequency_;
        scan_msg->range_min = range_min_;
        scan_msg->range_max = range_max_;
        scan_msg->ranges.resize(360, std::numeric_limits<float>::infinity());
        scan_msg->intensities.resize(360, 0.0);

        for (const auto& m : current_scan_) {
            double ang = m.angle + angle_offset_;
            if (flip_scan_) ang = 360.0 - ang;
            while (ang < 0) ang += 360.0;
            while (ang >= 360.0) ang -= 360.0;

            int idx;
            if (ang <= 180.0) {
                idx = (int)round(ang) + 180;
                if (idx >= 360) idx = 0;
            } else {
                idx = (int)round(ang) - 180;
            }
            if (idx >= 0 && idx < 360) {
                scan_msg->ranges[idx] = m.distance / 1000.0;
                scan_msg->intensities[idx] = m.quality;
            }
        }
        scan_pub_->publish(*scan_msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RPLidarTcpNode>());
    rclcpp::shutdown();
    return 0;
}