// Minimal ROS1 C++ line follower experiment.
//
// This file is intentionally separate from the Web panel. It publishes
// /cmd_vel directly, so run it only while supervised and after stopping Web
// line-follow/navigation/patrol control.

#include <algorithm>
#include <cmath>
#include <iostream>
#include <locale>
#include <sstream>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>

using namespace cv;
using namespace std;

class LineFollower
{
public:
    LineFollower() : private_nh_("~"), it_(nh_), lost_frames_(0)
    {
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        try {
            locale::global(locale("zh_CN.UTF-8"));
        } catch (const std::exception &e) {
            ROS_WARN("中文 locale 不可用，继续使用默认 locale: %s", e.what());
        }

        readParameters();

        adaptive_Kp_ = Kp_;
        adaptive_Ki_ = Ki_;
        adaptive_Kd_ = Kd_;

        printParameters();

        image_sub_ = it_.subscribe(image_topic_, 1, &LineFollower::imageCb, this);
        image_pub_ = it_.advertise("/img_follow", 1);
    }

    ~LineFollower()
    {
        stop();
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher cmd_vel_pub_;

    int lost_frames_;
    int max_lost_frames_;
    int last_target_cx_;
    int straight_hold_remaining_;
    int branch_lock_remaining_;
    int branch_route_memory_remaining_;
    int branch_route_sign_;
    int branch_exit_hold_remaining_;
    int white_gap_remaining_;
    geometry_msgs::Twist last_twist_;

    double max_linear_speed_;
    double max_angular_speed_;
    double Kp_;
    double Ki_;
    double Kd_;
    double adaptive_Kp_;
    double adaptive_Ki_;
    double adaptive_Kd_;
    double deadzone_;
    double integral_limit_;
    double error_threshold_;
    double soft_limit_lower_;
    double soft_limit_upper_;
    double filter_coefficient_;
    double roi_top_ratio_;
    double near_weight_;
    double turn_slowdown_;
    double min_curve_speed_;
    double lookahead_weight_;
    double curve_slowdown_;
    double max_curve_delta_;
    double straight_hold_near_error_;
    double straight_hold_curve_delta_;
    double straight_hold_speed_;
    double branch_split_delta_;
    double branch_approach_speed_;
    double branch_target_weight_;
    double branch_max_angular_scale_;
    double white_gap_speed_;
    double yellow_guard_center_ratio_;
    double yellow_guard_height_ratio_;
    double yellow_avoid_angular_;
    bool pid_debug_output_;
    bool img_debug_output_;
    bool yellow_guard_enabled_;
    bool straight_hold_enabled_;
    bool branch_lock_enabled_;
    bool branch_approach_hold_;
    bool white_gap_enabled_;
    string end_audio_msg_;
    string image_topic_;
    string branch_choice_;
    int line_threshold_;
    int min_contour_area_;
    int close_kernel_size_;
    int close_kernel_width_;
    int close_kernel_height_;
    int layer_count_;
    int lookahead_layer_;
    int sliding_min_width_;
    int straight_hold_frames_;
    int branch_commit_frames_;
    int branch_route_memory_frames_;
    int branch_exit_hold_frames_;
    int white_h_min_;
    int white_h_max_;
    int white_s_max_;
    int white_v_min_;
    int white_min_pixels_;
    int white_gap_frames_;
    int yellow_h_min_;
    int yellow_h_max_;
    int yellow_s_min_;
    int yellow_s_max_;
    int yellow_v_min_;
    int yellow_v_max_;
    int yellow_min_pixels_;

    struct SlidingWindowResult
    {
        bool found;
        bool near_seen;
        int near_cx;
        int near_cy;
        int lookahead_cx;
        int lookahead_cy;
        int target_cx;
        int target_cy;
        int layers_found;
        double curve_delta;
        double confidence;
        int far_left_cx;
        int far_right_cx;
        int near_width;
        bool branch_split;

        SlidingWindowResult() :
            found(false),
            near_seen(false),
            near_cx(-1),
            near_cy(-1),
            lookahead_cx(-1),
            lookahead_cy(-1),
            target_cx(-1),
            target_cy(-1),
            layers_found(0),
            curve_delta(0.0),
            confidence(0.0),
            far_left_cx(-1),
            far_right_cx(-1),
            near_width(0),
            branch_split(false)
        {
        }
    };

    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge异常: %s", e.what());
            return;
        }

        Mat frame = cv_ptr->image;
        followLine(frame);
    }

    bool chooseTargetContour(
        const vector<vector<Point> > &contours,
        int image_width,
        int &best_cx,
        int &best_cy)
    {
        bool found = false;
        double best_score = 0.0;
        const double center_x = image_width / 2.0;

        for (size_t i = 0; i < contours.size(); i++) {
            const double area = contourArea(contours[i]);
            if (area < min_contour_area_) {
                continue;
            }

            Moments m = moments(contours[i]);
            if (m.m00 <= 0.0) {
                continue;
            }

            Rect box = boundingRect(contours[i]);
            int cx = static_cast<int>(m.m10 / m.m00);
            int cy = static_cast<int>(m.m01 / m.m00);
            double score = 0.0;

            if (branch_choice_ == "right") {
                score = static_cast<double>(box.x + box.width);
            } else if (branch_choice_ == "largest") {
                score = area;
            } else if (branch_choice_ == "center") {
                score = -std::abs(cx - center_x);
            } else {
                score = -static_cast<double>(box.x);
            }

            if (!found || score > best_score) {
                found = true;
                best_score = score;
                best_cx = cx;
                best_cy = cy;
            }
        }

        return found;
    }

    bool chooseBandTarget(
        const Mat &band,
        int y_offset,
        int expected_cx,
        int image_width,
        int &target_cx,
        int &target_cy)
    {
        vector<vector<Point> > contours;
        findContours(band.clone(), contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        bool found = false;
        double best_score = 0.0;
        const double center_x = image_width / 2.0;
        const double min_band_area = std::max(8.0, static_cast<double>(min_contour_area_) / std::max(1, layer_count_));

        for (size_t i = 0; i < contours.size(); i++) {
            const double area = contourArea(contours[i]);
            if (area < min_band_area) {
                continue;
            }

            Moments m = moments(contours[i]);
            if (m.m00 <= 0.0) {
                continue;
            }

            Rect box = boundingRect(contours[i]);
            int cx = static_cast<int>(m.m10 / m.m00);
            int cy = y_offset + static_cast<int>(m.m01 / m.m00);
            double score = 0.0;

            if (expected_cx >= 0) {
                score = -std::abs(cx - expected_cx);
            } else if (branch_choice_ == "right") {
                score = static_cast<double>(box.x + box.width);
            } else if (branch_choice_ == "largest") {
                score = area;
            } else if (branch_choice_ == "center") {
                score = -std::abs(cx - center_x);
            } else {
                score = -static_cast<double>(box.x);
            }

            if (!found || score > best_score) {
                found = true;
                best_score = score;
                target_cx = cx;
                target_cy = cy;
            }
        }

        return found;
    }

    bool chooseBandWindow(
        const Mat &band,
        int y_offset,
        int expected_cx,
        int image_width,
        int &target_cx,
        int &target_cy,
        int &target_width)
    {
        bool found = false;
        double best_score = 0.0;
        const int column_threshold = std::max(1, band.rows / 5);
        const int min_width = std::max(2, sliding_min_width_);
        const double center_x = image_width / 2.0;
        int run_start = -1;

        for (int x = 0; x <= image_width; x++) {
            bool active = false;
            if (x < image_width) {
                active = countNonZero(band.col(x)) >= column_threshold;
            }

            if (active && run_start < 0) {
                run_start = x;
            } else if ((!active || x == image_width) && run_start >= 0) {
                const int run_end = x - 1;
                const int width = run_end - run_start + 1;
                if (width >= min_width) {
                    const int cx = (run_start + run_end) / 2;
                    const int cy = y_offset + band.rows / 2;
                    double score = 0.0;

                    if (expected_cx >= 0) {
                        score = -std::abs(cx - expected_cx) + width * 0.02;
                    } else if (branch_choice_ == "right") {
                        score = static_cast<double>(run_end);
                    } else if (branch_choice_ == "largest") {
                        score = static_cast<double>(width);
                    } else if (branch_choice_ == "center") {
                        score = -std::abs(cx - center_x);
                    } else {
                        score = -static_cast<double>(run_start);
                    }

                    if (!found || score > best_score) {
                        found = true;
                        best_score = score;
                        target_cx = cx;
                        target_cy = cy;
                        target_width = width;
                    }
                }
                run_start = -1;
            }
        }

        return found;
    }

    bool findBandExtremes(
        const Mat &band,
        int image_width,
        int &left_cx,
        int &right_cx)
    {
        const int column_threshold = std::max(1, band.rows / 5);
        const int min_width = std::max(2, sliding_min_width_);
        bool found = false;
        int run_start = -1;
        left_cx = -1;
        right_cx = -1;

        for (int x = 0; x <= image_width; x++) {
            bool active = false;
            if (x < image_width) {
                active = countNonZero(band.col(x)) >= column_threshold;
            }

            if (active && run_start < 0) {
                run_start = x;
            } else if ((!active || x == image_width) && run_start >= 0) {
                const int run_end = x - 1;
                const int width = run_end - run_start + 1;
                if (width >= min_width) {
                    const int cx = (run_start + run_end) / 2;
                    if (!found) {
                        left_cx = cx;
                    }
                    right_cx = cx;
                    found = true;
                }
                run_start = -1;
            }
        }

        return found && left_cx >= 0 && right_cx >= 0;
    }

    bool detectBranchSplit(const SlidingWindowResult &path)
    {
        return branch_lock_enabled_ &&
            path.branch_split &&
            path.far_left_cx >= 0 &&
            path.far_right_cx >= 0 &&
            std::abs(path.far_right_cx - path.far_left_cx) >= branch_split_delta_;
    }

    int selectCommittedBranchTarget(const SlidingWindowResult &path)
    {
        if (branch_choice_ == "right") {
            return path.far_right_cx;
        }
        if (branch_choice_ == "center") {
            return (path.far_left_cx + path.far_right_cx) / 2;
        }
        if (branch_choice_ == "largest") {
            return path.lookahead_cx;
        }
        return path.far_left_cx;
    }

    int branchChoiceSign() const
    {
        if (branch_choice_ == "right") {
            return 1;
        }
        if (branch_choice_ == "left") {
            return -1;
        }
        return 0;
    }

    void startBranchRouteMemory()
    {
        branch_route_sign_ = branchChoiceSign();
        if (branch_route_sign_ == 0) {
            branch_route_memory_remaining_ = 0;
            return;
        }
        branch_route_memory_remaining_ = std::max(branch_route_memory_remaining_, branch_route_memory_frames_);
        branch_exit_hold_remaining_ = 0;
    }

    int blendBranchTarget(const SlidingWindowResult &path, int committed_cx) const
    {
        if (!path.near_seen || committed_cx < 0) {
            return committed_cx;
        }
        const double weight = std::max(0.0, std::min(1.0, branch_target_weight_));
        return static_cast<int>((1.0 - weight) * path.near_cx + weight * committed_cx);
    }

    SlidingWindowResult computeSlidingWindowCenter(const Mat &mask, int image_width)
    {
        SlidingWindowResult result;
        const int rows = mask.rows;
        const int bands = std::max(1, std::min(layer_count_, rows));
        const int band_height = std::max(1, rows / bands);
        int expected_cx = (last_target_cx_ >= 0) ? last_target_cx_ : image_width / 2;

        vector<Point> centers;

        for (int layer = 0; layer < bands; layer++) {
            int y1 = std::max(0, rows - (layer + 1) * band_height);
            int y2 = (layer == 0) ? rows : std::min(rows, rows - layer * band_height);
            if (y2 <= y1) {
                continue;
            }

            Mat band = mask(Rect(0, y1, image_width, y2 - y1));
            int cx = -1;
            int cy = -1;
            int width = 0;
            int left_cx = -1;
            int right_cx = -1;
            findBandExtremes(band, image_width, left_cx, right_cx);
            if (left_cx >= 0 && right_cx >= 0 && right_cx - left_cx >= branch_split_delta_) {
                result.branch_split = true;
                result.far_left_cx = left_cx;
                result.far_right_cx = right_cx;
            }
            if (!chooseBandWindow(band, y1, expected_cx, image_width, cx, cy, width)) {
                continue;
            }

            if (!result.near_seen) {
                result.near_seen = true;
                result.near_cx = cx;
                result.near_cy = cy;
                result.near_width = width;
            }

            if (static_cast<int>(centers.size()) <= lookahead_layer_) {
                result.lookahead_cx = cx;
                result.lookahead_cy = cy;
            }

            centers.push_back(Point(cx, cy));
            expected_cx = cx;
        }

        result.layers_found = static_cast<int>(centers.size());
        if (centers.empty()) {
            return result;
        }

        if (result.lookahead_cx < 0) {
            result.lookahead_cx = centers.back().x;
            result.lookahead_cy = centers.back().y;
        }

        const double near_x = result.near_seen ? result.near_cx : centers.front().x;
        const double lookahead_x = result.lookahead_cx;
        result.target_cx = static_cast<int>((1.0 - lookahead_weight_) * near_x + lookahead_weight_ * lookahead_x);
        result.target_cy = static_cast<int>((1.0 - lookahead_weight_) * result.near_cy + lookahead_weight_ * result.lookahead_cy);
        result.curve_delta = lookahead_x - near_x;
        result.confidence = std::min(1.0, static_cast<double>(result.layers_found) / std::max(1, bands));
        result.found = result.layers_found >= 1;
        last_target_cx_ = result.target_cx;
        return result;
    }

    bool computeLayeredCenter(const Mat &mask, int image_width, int &best_cx, int &best_cy)
    {
        SlidingWindowResult result = computeSlidingWindowCenter(mask, image_width);
        if (!result.found) {
            return false;
        }
        best_cx = result.target_cx;
        best_cy = result.target_cy;
        return true;
    }

    bool detectWhiteCrosswalk(const Mat &hsv)
    {
        if (!white_gap_enabled_ || hsv.empty()) {
            return false;
        }

        Mat white_mask;
        Scalar lower_white(white_h_min_, 0, white_v_min_);
        Scalar upper_white(white_h_max_, white_s_max_, 255);
        inRange(hsv, lower_white, upper_white, white_mask);

        const int width = white_mask.cols;
        const int height = white_mask.rows;
        const int y1 = std::max(0, height * 2 / 3);
        Mat lower = white_mask(Rect(0, y1, width, height - y1));

        vector<vector<Point> > contours;
        findContours(lower, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        for (size_t i = 0; i < contours.size(); i++) {
            Rect box = boundingRect(contours[i]);
            const int pixels = static_cast<int>(contourArea(contours[i]));
            if (pixels >= white_min_pixels_ && box.width > width * 0.35 && box.height < lower.rows * 0.65) {
                return true;
            }
        }
        return false;
    }

    bool detectYellowGuard(const Mat &hsv, int &avoid_direction)
    {
        if (!yellow_guard_enabled_ || hsv.empty()) {
            return false;
        }

        Mat yellow_mask;
        Scalar lower_yellow(yellow_h_min_, yellow_s_min_, yellow_v_min_);
        Scalar upper_yellow(yellow_h_max_, yellow_s_max_, yellow_v_max_);
        inRange(hsv, lower_yellow, upper_yellow, yellow_mask);

        const int width = yellow_mask.cols;
        const int height = yellow_mask.rows;
        const int guard_height = std::max(1, static_cast<int>(height * yellow_guard_height_ratio_));
        const int y1 = std::max(0, height - guard_height);
        const int guard_width = std::max(1, static_cast<int>(width * yellow_guard_center_ratio_));
        const int x1 = std::max(0, (width - guard_width) / 2);
        const int x2 = std::min(width, x1 + guard_width);
        const int mid_x = width / 2;

        Mat guard = yellow_mask(Rect(x1, y1, x2 - x1, height - y1));
        const int total_pixels = countNonZero(guard);
        if (total_pixels < yellow_min_pixels_) {
            return false;
        }

        const int left_x2 = std::max(x1, std::min(mid_x, x2));
        const int right_x1 = std::min(x2, std::max(mid_x, x1));
        int left_pixels = 0;
        int right_pixels = 0;
        if (left_x2 > x1) {
            left_pixels = countNonZero(yellow_mask(Rect(x1, y1, left_x2 - x1, height - y1)));
        }
        if (x2 > right_x1) {
            right_pixels = countNonZero(yellow_mask(Rect(right_x1, y1, x2 - right_x1, height - y1)));
        }

        if (right_pixels > left_pixels * 1.2) {
            avoid_direction = 1;
        } else if (left_pixels > right_pixels * 1.2) {
            avoid_direction = -1;
        } else {
            avoid_direction = 0;
        }
        return true;
    }

    bool shouldHoldStraight(const SlidingWindowResult &path, int center_x, bool yellow_guard)
    {
        if (!straight_hold_enabled_ || yellow_guard || !path.near_seen) {
            straight_hold_remaining_ = 0;
            return false;
        }

        const double near_error = std::abs(path.near_cx - center_x);
        const double curve_delta = std::abs(path.curve_delta);
        if (near_error <= straight_hold_near_error_ && curve_delta >= straight_hold_curve_delta_) {
            if (straight_hold_remaining_ <= 0) {
                straight_hold_remaining_ = straight_hold_frames_;
            }
        } else if (near_error > straight_hold_near_error_ * 1.4) {
            straight_hold_remaining_ = 0;
        }

        if (straight_hold_remaining_ > 0) {
            straight_hold_remaining_--;
            return true;
        }
        return false;
    }

    bool shouldHoldBranchExit(const SlidingWindowResult &path, int center_x, bool yellow_guard)
    {
        if (yellow_guard || branch_route_memory_remaining_ <= 0 || !path.near_seen) {
            branch_exit_hold_remaining_ = 0;
            return false;
        }

        const double near_error = std::abs(path.near_cx - center_x);
        const bool far_branch_or_curve = detectBranchSplit(path) ||
            std::abs(path.curve_delta) >= straight_hold_curve_delta_;
        if (near_error <= straight_hold_near_error_ && far_branch_or_curve) {
            if (branch_exit_hold_remaining_ <= 0) {
                branch_exit_hold_remaining_ = branch_exit_hold_frames_;
            }
        } else if (near_error > straight_hold_near_error_ * 1.6) {
            branch_exit_hold_remaining_ = 0;
        }

        if (branch_exit_hold_remaining_ > 0) {
            branch_exit_hold_remaining_--;
            return true;
        }
        return false;
    }

    void followLine(Mat &image)
    {
        int height = image.rows;
        int width = image.cols;
        if (height <= 0 || width <= 0) {
            stop();
            return;
        }

        int start_row = static_cast<int>(height * roi_top_ratio_);
        start_row = std::max(0, std::min(height - 1, start_row));
        Rect roi_rect(0, start_row, width, height - start_row);
        Mat roi = image(roi_rect);

        Mat hsv;
        cvtColor(roi, hsv, COLOR_BGR2HSV);

        Mat mask;
        Scalar lower_black(0, 0, 0);
        Scalar upper_black(180, 255, line_threshold_);
        inRange(hsv, lower_black, upper_black, mask);

        int kernel_width = std::max(1, close_kernel_width_);
        int kernel_height = std::max(1, close_kernel_height_);
        if (kernel_width % 2 == 0) {
            kernel_width += 1;
        }
        if (kernel_height % 2 == 0) {
            kernel_height += 1;
        }
        Mat kernel = getStructuringElement(MORPH_RECT, Size(kernel_width, kernel_height));
        morphologyEx(mask, mask, MORPH_CLOSE, kernel);

        SlidingWindowResult path = computeSlidingWindowCenter(mask, width);
        int best_cx = path.target_cx;
        int best_cy = path.target_cy;
        bool found_line = path.found;
        int yellow_avoid_direction = 0;
        bool yellow_guard = detectYellowGuard(hsv, yellow_avoid_direction);

        Mat debug_img;
        cvtColor(mask, debug_img, COLOR_GRAY2BGR);
        bool white_gap = detectWhiteCrosswalk(hsv);
        if (yellow_guard) {
            rectangle(debug_img, Rect(width / 4, debug_img.rows * 3 / 4, width / 2, debug_img.rows / 4), Scalar(0, 255, 255), 2);
        }
        if (white_gap) {
            rectangle(debug_img, Rect(width / 5, debug_img.rows * 3 / 4, width * 3 / 5, debug_img.rows / 8), Scalar(255, 255, 255), 2);
        }

        if (found_line) {
            lost_frames_ = 0;
            if (white_gap) {
                white_gap_remaining_ = white_gap_frames_;
            }
            circle(debug_img, Point(best_cx, best_cy), 10, Scalar(0, 0, 255), -1);
            circle(debug_img, Point(path.near_cx, path.near_cy), 6, Scalar(255, 0, 0), -1);
            circle(debug_img, Point(path.lookahead_cx, path.lookahead_cy), 6, Scalar(0, 255, 0), -1);
            line(debug_img, Point(path.near_cx, path.near_cy), Point(path.lookahead_cx, path.lookahead_cy), Scalar(255, 255, 255), 2);

            int center_x = width / 2;
            bool straight_hold = shouldHoldStraight(path, center_x, yellow_guard);
            bool branch_exit_hold = false;
            bool branch_approach = false;
            bool branch_committed = false;
            bool branch_route_memory = false;
            const bool branch_split = detectBranchSplit(path);
            if (branch_split && path.near_seen &&
                std::abs(path.near_cx - center_x) <= straight_hold_near_error_ &&
                branch_route_memory_remaining_ <= 0) {
                branch_approach = branch_approach_hold_;
                if (branch_lock_remaining_ <= 0 && !branch_approach) {
                    branch_lock_remaining_ = branch_commit_frames_;
                }
            } else if (branch_split && branch_lock_remaining_ <= 0 && branch_route_memory_remaining_ <= 0) {
                branch_lock_remaining_ = branch_commit_frames_;
                startBranchRouteMemory();
            }
            if (branch_lock_remaining_ > 0 && branch_split) {
                startBranchRouteMemory();
                const int committed_cx = selectCommittedBranchTarget(path);
                if (committed_cx >= 0) {
                    best_cx = blendBranchTarget(path, committed_cx);
                    best_cy = path.lookahead_cy;
                    branch_committed = true;
                    branch_lock_remaining_--;
                    printUTF8("BRANCH_COMMITTED");
                }
            } else if (branch_route_memory_remaining_ > 0 && branch_split) {
                const int committed_cx = selectCommittedBranchTarget(path);
                if (committed_cx >= 0) {
                    best_cx = blendBranchTarget(path, committed_cx);
                    best_cy = path.lookahead_cy;
                    branch_route_memory = true;
                    printUTF8("BRANCH_ROUTE_MEMORY");
                }
            }
            if (branch_approach && !branch_committed) {
                best_cx = path.near_cx;
                best_cy = path.near_cy;
                rectangle(debug_img, Rect(width / 3, debug_img.rows / 2, width / 3, debug_img.rows / 6), Scalar(0, 128, 255), 2);
                printUTF8("BRANCH_APPROACH");
            }
            branch_exit_hold = shouldHoldBranchExit(path, center_x, yellow_guard);
            if (branch_exit_hold) {
                best_cx = path.near_cx;
                best_cy = path.near_cy;
                rectangle(debug_img, Rect(width / 3, debug_img.rows / 3, width / 3, debug_img.rows / 6), Scalar(255, 128, 0), 2);
                printUTF8("BRANCH_EXIT_HOLD");
            }
            if (straight_hold) {
                best_cx = path.near_cx;
                best_cy = path.near_cy;
                rectangle(debug_img, Rect(width / 3, debug_img.rows * 2 / 3, width / 3, debug_img.rows / 6), Scalar(255, 255, 0), 2);
                printUTF8("STRAIGHT_HOLD");
            }
            if (branch_route_memory_remaining_ > 0) {
                branch_route_memory_remaining_--;
            }
            int error = best_cx - center_x;

            static double last_error = 0.0;
            static double integral = 0.0;
            static double last_filtered_error = 0.0;

            double filtered_error = filter_coefficient_ * error + (1.0 - filter_coefficient_) * last_filtered_error;
            last_filtered_error = filtered_error;

            if (std::abs(filtered_error) < deadzone_) {
                filtered_error = 0.0;
            } else {
                filtered_error = (filtered_error > 0.0) ? filtered_error - deadzone_ : filtered_error + deadzone_;
            }

            adaptPIDParameters(filtered_error);

            if (std::abs(filtered_error) < error_threshold_) {
                integral += filtered_error;
            } else {
                integral = 0.0;
            }
            integral = std::max(-integral_limit_, std::min(integral_limit_, integral));

            double derivative = filtered_error - last_error;
            double steering_angle = adaptive_Kp_ * filtered_error + adaptive_Ki_ * integral + adaptive_Kd_ * derivative;
            steering_angle = std::max(soft_limit_lower_, std::min(soft_limit_upper_, steering_angle));

            geometry_msgs::Twist twist;
            twist.angular.z = -steering_angle;
            twist.angular.z = std::max(-max_angular_speed_, std::min(max_angular_speed_, twist.angular.z));
            if (branch_committed || branch_route_memory) {
                const double branch_limit = max_angular_speed_ * branch_max_angular_scale_;
                twist.angular.z = std::max(-branch_limit, std::min(branch_limit, twist.angular.z));
            }
            if (yellow_guard) {
                twist.linear.x = 0.0;
                twist.angular.z = yellow_avoid_direction * std::min(max_angular_speed_, yellow_avoid_angular_);
            } else if (branch_approach) {
                twist.linear.x = std::min(max_linear_speed_, branch_approach_speed_);
            } else if (straight_hold || branch_exit_hold) {
                twist.linear.x = std::min(max_linear_speed_, straight_hold_speed_);
            } else {
                const double angular_ratio = (max_angular_speed_ > 0.0) ?
                    std::min(1.0, std::abs(twist.angular.z) / max_angular_speed_) : 0.0;
                const double curve_ratio = std::min(1.0, std::abs(path.curve_delta) / std::max(1.0, max_curve_delta_));
                const double slowdown = std::max(turn_slowdown_ * angular_ratio, curve_slowdown_ * curve_ratio);
                const double speed_scale = std::max(0.0, 1.0 - slowdown);
                twist.linear.x = max_linear_speed_ * speed_scale;
                if (twist.linear.x > 0.0) {
                    twist.linear.x = std::max(min_curve_speed_, twist.linear.x);
                }
            }

            last_twist_ = twist;
            cmd_vel_pub_.publish(twist);

            if (pid_debug_output_) {
                printSpeedInfo(twist.linear.x, twist.angular.z);
            }
            last_error = filtered_error;
        } else {
            lost_frames_++;
            if (lost_frames_ < max_lost_frames_) {
                if (white_gap_remaining_ > 0) {
                    last_twist_.linear.x = std::min(max_linear_speed_, white_gap_speed_);
                    white_gap_remaining_--;
                }
                cmd_vel_pub_.publish(last_twist_);
                if (pid_debug_output_) {
                    printUTF8(white_gap_remaining_ > 0 ? "WHITE_GAP" : "遇到虚线/反光，保持惯性...");
                }
            } else {
                last_target_cx_ = -1;
                straight_hold_remaining_ = 0;
                branch_lock_remaining_ = 0;
                branch_route_memory_remaining_ = 0;
                branch_route_sign_ = 0;
                branch_exit_hold_remaining_ = 0;
                white_gap_remaining_ = 0;
                stop();
            }
        }

        if (img_debug_output_) {
            sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(
                std_msgs::Header(),
                sensor_msgs::image_encodings::BGR8,
                debug_img).toImageMsg();
            image_pub_.publish(output_msg);
        }
    }

    void adaptPIDParameters(double error)
    {
        if (std::abs(error) > error_threshold_ * 2.0) {
            adaptive_Kp_ = Kp_ * 1.5;
            adaptive_Ki_ = Ki_ * 0.5;
            adaptive_Kd_ = Kd_ * 2.0;
        } else {
            adaptive_Kp_ = Kp_;
            adaptive_Ki_ = Ki_;
            adaptive_Kd_ = Kd_;
        }
    }

    void stop()
    {
        geometry_msgs::Twist twist;
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
        cmd_vel_pub_.publish(twist);
        if (pid_debug_output_) {
            printUTF8("丢失路线，已停止");
        }
    }

    void readParameters()
    {
        private_nh_.param("max_linear_speed", max_linear_speed_, 0.06);
        private_nh_.param("max_angular_speed", max_angular_speed_, 0.30);
        private_nh_.param("Kp", Kp_, 0.005);
        private_nh_.param("Ki", Ki_, 0.0);
        private_nh_.param("Kd", Kd_, 0.002);
        private_nh_.param("deadzone", deadzone_, 5.0);
        private_nh_.param("integral_limit", integral_limit_, 1.0);
        private_nh_.param("error_threshold", error_threshold_, 50.0);
        private_nh_.param("soft_limit_lower", soft_limit_lower_, -1.5);
        private_nh_.param("soft_limit_upper", soft_limit_upper_, 1.5);
        private_nh_.param("filter_coefficient", filter_coefficient_, 0.8);
        private_nh_.param("roi_top_ratio", roi_top_ratio_, 0.50);
        private_nh_.param("near_weight", near_weight_, 0.75);
        private_nh_.param("turn_slowdown", turn_slowdown_, 0.70);
        private_nh_.param("min_curve_speed", min_curve_speed_, 0.0);
        private_nh_.param("lookahead_weight", lookahead_weight_, 0.35);
        private_nh_.param("curve_slowdown", curve_slowdown_, 0.35);
        private_nh_.param("max_curve_delta", max_curve_delta_, 180.0);
        private_nh_.param("straight_hold_near_error", straight_hold_near_error_, 35.0);
        private_nh_.param("straight_hold_curve_delta", straight_hold_curve_delta_, 90.0);
        private_nh_.param("straight_hold_speed", straight_hold_speed_, 0.025);
        private_nh_.param("branch_split_delta", branch_split_delta_, 120.0);
        private_nh_.param("branch_approach_speed", branch_approach_speed_, 0.022);
        private_nh_.param("branch_target_weight", branch_target_weight_, 0.45);
        private_nh_.param("branch_max_angular_scale", branch_max_angular_scale_, 0.70);
        private_nh_.param("white_gap_speed", white_gap_speed_, 0.020);
        private_nh_.param("yellow_guard_center_ratio", yellow_guard_center_ratio_, 0.58);
        private_nh_.param("yellow_guard_height_ratio", yellow_guard_height_ratio_, 0.18);
        private_nh_.param("yellow_avoid_angular", yellow_avoid_angular_, 0.16);
        private_nh_.param("pid_debug_output", pid_debug_output_, true);
        private_nh_.param("img_debug_output", img_debug_output_, true);
        private_nh_.param("yellow_guard_enabled", yellow_guard_enabled_, true);
        private_nh_.param("straight_hold_enabled", straight_hold_enabled_, true);
        private_nh_.param("branch_lock_enabled", branch_lock_enabled_, true);
        private_nh_.param("branch_approach_hold", branch_approach_hold_, true);
        private_nh_.param("white_gap_enabled", white_gap_enabled_, true);
        private_nh_.param("line_threshold", line_threshold_, 120);
        private_nh_.param("image_topic", image_topic_, string("/camera/image_raw"));
        private_nh_.param("branch_choice", branch_choice_, string("left"));
        private_nh_.param("max_lost_frames", max_lost_frames_, 10);
        private_nh_.param("min_contour_area", min_contour_area_, 200);
        private_nh_.param("close_kernel_size", close_kernel_size_, 25);
        private_nh_.param("close_kernel_width", close_kernel_width_, 13);
        private_nh_.param("close_kernel_height", close_kernel_height_, 61);
        private_nh_.param("layer_count", layer_count_, 12);
        private_nh_.param("lookahead_layer", lookahead_layer_, 4);
        private_nh_.param("sliding_min_width", sliding_min_width_, 18);
        private_nh_.param("straight_hold_frames", straight_hold_frames_, 4);
        private_nh_.param("branch_commit_frames", branch_commit_frames_, 8);
        private_nh_.param("branch_route_memory_frames", branch_route_memory_frames_, 45);
        private_nh_.param("branch_exit_hold_frames", branch_exit_hold_frames_, 8);
        private_nh_.param("white_h_min", white_h_min_, 0);
        private_nh_.param("white_h_max", white_h_max_, 179);
        private_nh_.param("white_s_max", white_s_max_, 55);
        private_nh_.param("white_v_min", white_v_min_, 205);
        private_nh_.param("white_min_pixels", white_min_pixels_, 500);
        private_nh_.param("white_gap_frames", white_gap_frames_, 8);
        private_nh_.param("yellow_h_min", yellow_h_min_, 15);
        private_nh_.param("yellow_h_max", yellow_h_max_, 45);
        private_nh_.param("yellow_s_min", yellow_s_min_, 55);
        private_nh_.param("yellow_s_max", yellow_s_max_, 255);
        private_nh_.param("yellow_v_min", yellow_v_min_, 60);
        private_nh_.param("yellow_v_max", yellow_v_max_, 255);
        private_nh_.param("yellow_min_pixels", yellow_min_pixels_, 80);

        if (branch_choice_ != "left" && branch_choice_ != "right" &&
            branch_choice_ != "largest" && branch_choice_ != "center") {
            ROS_WARN("未知 branch_choice=%s，回退到 left", branch_choice_.c_str());
            branch_choice_ = "left";
        }

        max_linear_speed_ = std::max(0.0, std::min(0.10, max_linear_speed_));
        max_angular_speed_ = std::max(0.0, std::min(0.60, max_angular_speed_));
        line_threshold_ = std::max(0, std::min(255, line_threshold_));
        max_lost_frames_ = std::max(0, std::min(60, max_lost_frames_));
        min_contour_area_ = std::max(1, min_contour_area_);
        close_kernel_size_ = std::max(1, std::min(99, close_kernel_size_));
        close_kernel_width_ = std::max(1, std::min(99, close_kernel_width_));
        close_kernel_height_ = std::max(1, std::min(151, close_kernel_height_));
        layer_count_ = std::max(1, std::min(20, layer_count_));
        roi_top_ratio_ = std::max(0.25, std::min(0.80, roi_top_ratio_));
        near_weight_ = std::max(0.20, std::min(0.95, near_weight_));
        turn_slowdown_ = std::max(0.0, std::min(1.0, turn_slowdown_));
        min_curve_speed_ = std::max(0.0, std::min(max_linear_speed_, min_curve_speed_));
        lookahead_weight_ = std::max(0.0, std::min(0.80, lookahead_weight_));
        curve_slowdown_ = std::max(0.0, std::min(1.0, curve_slowdown_));
        max_curve_delta_ = std::max(20.0, std::min(1000.0, max_curve_delta_));
        straight_hold_near_error_ = std::max(0.0, std::min(300.0, straight_hold_near_error_));
        straight_hold_curve_delta_ = std::max(0.0, std::min(1000.0, straight_hold_curve_delta_));
        straight_hold_speed_ = std::max(0.0, std::min(max_linear_speed_, straight_hold_speed_));
        branch_split_delta_ = std::max(20.0, std::min(1000.0, branch_split_delta_));
        branch_approach_speed_ = std::max(0.0, std::min(max_linear_speed_, branch_approach_speed_));
        branch_target_weight_ = std::max(0.05, std::min(1.0, branch_target_weight_));
        branch_max_angular_scale_ = std::max(0.20, std::min(1.0, branch_max_angular_scale_));
        white_gap_speed_ = std::max(0.0, std::min(max_linear_speed_, white_gap_speed_));
        yellow_guard_center_ratio_ = std::max(0.20, std::min(1.0, yellow_guard_center_ratio_));
        yellow_guard_height_ratio_ = std::max(0.05, std::min(0.50, yellow_guard_height_ratio_));
        yellow_avoid_angular_ = std::max(0.0, std::min(0.60, yellow_avoid_angular_));
        yellow_h_min_ = std::max(0, std::min(179, yellow_h_min_));
        yellow_h_max_ = std::max(0, std::min(179, yellow_h_max_));
        yellow_s_min_ = std::max(0, std::min(255, yellow_s_min_));
        yellow_s_max_ = std::max(0, std::min(255, yellow_s_max_));
        yellow_v_min_ = std::max(0, std::min(255, yellow_v_min_));
        yellow_v_max_ = std::max(0, std::min(255, yellow_v_max_));
        yellow_min_pixels_ = std::max(1, std::min(10000, yellow_min_pixels_));
        lookahead_layer_ = std::max(0, std::min(layer_count_ - 1, lookahead_layer_));
        sliding_min_width_ = std::max(1, std::min(200, sliding_min_width_));
        straight_hold_frames_ = std::max(0, std::min(30, straight_hold_frames_));
        branch_commit_frames_ = std::max(0, std::min(60, branch_commit_frames_));
        branch_route_memory_frames_ = std::max(0, std::min(120, branch_route_memory_frames_));
        branch_exit_hold_frames_ = std::max(0, std::min(40, branch_exit_hold_frames_));
        white_h_min_ = std::max(0, std::min(179, white_h_min_));
        white_h_max_ = std::max(0, std::min(179, white_h_max_));
        white_s_max_ = std::max(0, std::min(255, white_s_max_));
        white_v_min_ = std::max(0, std::min(255, white_v_min_));
        white_min_pixels_ = std::max(1, std::min(10000, white_min_pixels_));
        white_gap_frames_ = std::max(0, std::min(60, white_gap_frames_));
        last_target_cx_ = -1;
        straight_hold_remaining_ = 0;
        branch_lock_remaining_ = 0;
        branch_route_memory_remaining_ = 0;
        branch_route_sign_ = 0;
        branch_exit_hold_remaining_ = 0;
        white_gap_remaining_ = 0;
    }

    void printParameters()
    {
        std::ostringstream oss;
        oss << "初始化完成：image_topic=" << image_topic_
            << " branch_choice=" << branch_choice_
            << " line_threshold=" << line_threshold_
            << " max_lost_frames=" << max_lost_frames_
            << " roi_top_ratio=" << roi_top_ratio_
            << " close_kernel=" << close_kernel_width_ << "x" << close_kernel_height_
            << " lookahead_layer=" << lookahead_layer_
            << " max_linear=" << max_linear_speed_
            << " max_angular=" << max_angular_speed_;
        printUTF8(oss.str());
    }

    void printUTF8(const string &message)
    {
        ROS_INFO("%s", message.c_str());
    }

    void printSpeedInfo(double linear_x, double angular_z)
    {
        ROS_INFO("cmd_vel linear=%.3f angular=%.3f", linear_x, angular_z);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "line_follower");
    LineFollower follower;
    ros::spin();
    return 0;
}
