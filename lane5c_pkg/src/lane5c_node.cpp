#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <cmath>
#include <algorithm>
#include <vector>
#include <map>
#include <limits>

class PID {
public:
    PID(double p, double i, double d, double setpoint = 0.0)
        : kp_(p), ki_(i), kd_(d), setpoint_(setpoint), prev_error_(0.0), integral_(0.0) {}

    void setSetpoint(double setpoint) {
        setpoint_ = setpoint;
    }

    double operator()(double value) {
        double error = setpoint_ - value;
        integral_ += error;
        double derivative = error - prev_error_;
        prev_error_ = error;
        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

    double getSetpoint() const { 
        return setpoint_; 
    }

private:
    double kp_, ki_, kd_;
    double setpoint_;
    double prev_error_;
    double integral_;
};

class ImageSubscriber : public rclcpp::Node {
public:
    ImageSubscriber(PID pid, const std::map<std::string, double>& cfg)
        : Node("image_subscriber"), pid_st_(pid), cfg_(cfg), target_pixel_(-1), throttle_(0.2) {

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image", 10,
            std::bind(&ImageSubscriber::callback, this, std::placeholders::_1));

        steering_pub_ = this->create_publisher<std_msgs::msg::Float32>("/steering", 10);
        throttle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/throttle", 10);
    }

private:
    void callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat original_img = cv_ptr->image;
        cv::Mat binary = filter_colors(original_img);
        cv::Mat img_mask = region_of_interest(binary);
        cv::Mat result = selective_erosion(img_mask);
        cv::Mat canny_img = apply_canny(result);
        std::vector<cv::Vec4i> lines = houghLines(canny_img);

        if (!lines.empty()) {
            auto distributed_lines = separateLine(lines, original_img);
            auto regression_result = regression(distributed_lines, original_img);
            
            if (regression_result.empty()) {
                publish_controls(-0.24, 0.0);
                return;
            }
            
            auto center_fitx = compute_intersection(regression_result, original_img);
            if (center_fitx >= 0) {
                auto [steering, throttle] = predicDir(center_fitx, original_img.cols);
                publish_controls(steering, throttle);
            } else {
                publish_controls(0.0, 0.0);
            }
        } else {
            publish_controls(0.0, 0.0);
        }
    }

    void publish_controls(double steering, double throttle) {
        std_msgs::msg::Float32 msg_s;
        msg_s.data = std::clamp(steering, -1.0, 1.0);
        steering_pub_->publish(msg_s);

        std_msgs::msg::Float32 msg_t;
        msg_t.data = std::clamp(throttle_, 0.0, 0.5);
        throttle_pub_->publish(msg_t);
    }

    std::pair<double, double> predicDir(int center_fitx, int image_width) {
        if (target_pixel_ < 0) {
            target_pixel_ = center_fitx;
            RCLCPP_INFO(this->get_logger(), "Automatically chosen line position = %d", target_pixel_);
        }

        if (std::abs(pid_st_.getSetpoint() - target_pixel_) > 1e-6) {
            pid_st_.setSetpoint(target_pixel_);
        }

        double steering = pid_st_(center_fitx);
        steering -= 0.24;  // 오프셋 적용

        if (std::abs(center_fitx - target_pixel_) > cfg_.at("target_threshold")) {
            if (throttle_ > cfg_.at("throttle_min"))
                throttle_ -= cfg_.at("delta_th");
            if (throttle_ < cfg_.at("throttle_min"))
                throttle_ = cfg_.at("throttle_min");
        } else {
            if (throttle_ < cfg_.at("throttle_max"))
                throttle_ += cfg_.at("delta_th");
            if (throttle_ > cfg_.at("throttle_max"))
                throttle_ = cfg_.at("throttle_max");
        }

        RCLCPP_INFO(this->get_logger(), "[PID CONTROL] steering: %.3f, throttle: %.3f, center_fitx: %d, target: %d", 
                    steering, throttle_, center_fitx, target_pixel_);
        return {steering, throttle_};
    }

    // --- 유틸리티 함수들 ---
    cv::Mat filter_colors(const cv::Mat& img) {
        cv::Mat hsv;
        cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
        cv::Mat mask;
        cv::inRange(hsv, cv::Scalar(0, 0, 240), cv::Scalar(180, 15, 255), mask);
        return mask;
    }

    cv::Mat region_of_interest(const cv::Mat& binary) {
        int height = binary.rows;
        int width = binary.cols;
        cv::Mat mask = cv::Mat::zeros(binary.size(), binary.type());

        std::vector<cv::Point> polygon = {
            cv::Point(static_cast<int>(width * 0.1), height),
            cv::Point(static_cast<int>(width * 0.2), static_cast<int>(height * 0.1)),
            cv::Point(static_cast<int>(width * 0.8), static_cast<int>(height * 0.1)),
            cv::Point(static_cast<int>(width * 0.9), height)
        };
        std::vector<std::vector<cv::Point>> pts = {polygon};
        cv::fillPoly(mask, pts, 255);
        
        cv::Mat masked;
        cv::bitwise_and(binary, mask, masked);
        return masked;
    }

    cv::Mat selective_erosion(const cv::Mat& img_mask) {
        cv::Mat result = img_mask.clone();
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(img_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        
        for (const auto& cnt : contours) {
            cv::Rect rect = cv::boundingRect(cnt);
            if (rect.width > 10) {
                cv::Mat roi = img_mask(rect);
                cv::Mat eroded;
                cv::erode(roi, eroded, kernel, cv::Point(-1, -1), 1);
                eroded.copyTo(result(rect));
            }
        }
        return result;
    }

    cv::Mat apply_canny(const cv::Mat& img) {
        cv::Mat canny;
        cv::Canny(img, canny, 100, 200);
        return canny;
    }

    std::vector<cv::Vec4i> houghLines(const cv::Mat& edge_img) {
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(edge_img, lines, 1, CV_PI / 180, 30, 10, 5);
        return lines;
    }

    std::vector<std::vector<cv::Vec4i>> separateLine(const std::vector<cv::Vec4i>& lines, const cv::Mat& img) {
        std::vector<cv::Vec4i> left, right;
        int x_center = img.cols / 2;
        
        for (const auto& line : lines) {
            int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
            if (x1 < x_center && x2 < x_center) {
                left.push_back(line);
            } else if (x1 > x_center && x2 > x_center) {
                right.push_back(line);
            }
        }
        return {right, left};
    }

    std::vector<cv::Point> regression(const std::vector<std::vector<cv::Vec4i>>& distr_lines, const cv::Mat& img) {
        std::vector<cv::Point> points;
        
        for (const auto& group : distr_lines) {
            for (const auto& line : group) {
                points.emplace_back(line[0], line[1]);
                points.emplace_back(line[2], line[3]);
            }
        }
        return points;
    }

    int compute_intersection(const std::vector<cv::Point>& points, const cv::Mat& img) {
        if (points.size() < 6) {
            RCLCPP_WARN(this->get_logger(), "Not enough points for intersection computation");
            return -1;
        }
        
        // 점들을 왼쪽과 오른쪽으로 분리
        std::vector<cv::Point> left, right;
        int x_center = img.cols / 2;
        
        for (const auto& point : points) {
            if (point.x < x_center) {
                left.push_back(point);
            } else {
                right.push_back(point);
            }
        }
        
        if (left.size() < 2 || right.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Not enough points in left or right lane");
            return -1;
        }
        
        cv::Vec4f left_fit, right_fit;
        cv::fitLine(left, left_fit, cv::DIST_L2, 0, 0.01, 0.01);
        cv::fitLine(right, right_fit, cv::DIST_L2, 0, 0.01, 0.01);

        float target_y = img.rows * 0.6f;
        
        // 수직선 처리 (기울기가 무한대인 경우)
        float lx, rx;
        if (std::abs(left_fit[1]) < 1e-6) {
            lx = left_fit[2];
        } else {
            lx = left_fit[2] + ((target_y - left_fit[3]) * left_fit[0] / left_fit[1]);
        }
        
        if (std::abs(right_fit[1]) < 1e-6) {
            rx = right_fit[2];
        } else {
            rx = right_fit[2] + ((target_y - right_fit[3]) * right_fit[0] / right_fit[1]);
        }
        
        int center_x = static_cast<int>((lx + rx) / 2);
        
        RCLCPP_INFO(this->get_logger(), "Left X: %.1f, Right X: %.1f, Center X: %d", lx, rx, center_x);
        
        return center_x;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;
    PID pid_st_;
    std::map<std::string, double> cfg_;
    int target_pixel_;
    double throttle_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    PID pid(0.5, 0.01, 0.05);
    std::map<std::string, double> cfg = {
        {"PID_P", 0.5},
        {"PID_I", 0.01},
        {"PID_D", 0.05},
        {"target_threshold", 20},
        {"throttle_min", 0.1},
        {"throttle_max", 0.3},
        {"delta_th", 0.02}
    };
    
    auto node = std::make_shared<ImageSubscriber>(pid, cfg);
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}