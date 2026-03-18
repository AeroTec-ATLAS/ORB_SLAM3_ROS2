#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <sophus/se3.hpp>

#include "System.h"   // ORB_SLAM3

class StereoSlamNode : public rclcpp::Node
{
public:
    using ImageMsg              = sensor_msgs::msg::Image;
    using approximate_sync_policy =
        message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg>;

    StereoSlamNode(ORB_SLAM3::System* pSLAM,
                   const std::string &strSettingsFile,
                   const std::string &strDoRectify);
    ~StereoSlamNode();

private:
    // ── Core SLAM callback ────────────────────────────────────────────────────
    void GrabStereo(const ImageMsg::SharedPtr msgLeft,
                    const ImageMsg::SharedPtr msgRight);

    // ── Publishers ────────────────────────────────────────────────────────────
    void PublishPose(const Sophus::SE3f &Twc,
                     const std_msgs::msg::Header &header);
    void PublishOdometry(const Sophus::SE3f &Twc,
                         const std_msgs::msg::Header &header);
    void PublishTF(const Sophus::SE3f &Twc,
                   const std_msgs::msg::Header &header);
    void PublishMapPoints(const std_msgs::msg::Header &header);
    void PublishTrackingState(int state);

    // ── Failure handling ──────────────────────────────────────────────────────
    void OnTrackingLost();
    void AttemptReinitialization();
    void ResetCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
              std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    // ── Diagnostics timer ─────────────────────────────────────────────────────
    void DiagnosticsCallback();

    // ── Helpers ───────────────────────────────────────────────────────────────
    sensor_msgs::msg::PointCloud2 MapPointsToPointCloud2(
        const std::vector<ORB_SLAM3::MapPoint*> &map_points,
        const std_msgs::msg::Header &header);

    // ── SLAM system ──────────────────────────────────────────────────────────
    ORB_SLAM3::System* m_SLAM;
    bool doRectify{false};

    // Rectification maps
    cv::Mat M1l, M2l, M1r, M2r;

    // Image buffers
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    cv_bridge::CvImageConstPtr cv_ptrRight;

    // ── Subscriptions & sync ─────────────────────────────────────────────────
    std::shared_ptr<message_filters::Subscriber<ImageMsg>> left_sub;
    std::shared_ptr<message_filters::Subscriber<ImageMsg>> right_sub;
    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>> syncApproximate;

    // ── Publishers ────────────────────────────────────────────────────────────
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr         odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   map_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr            state_pub_;

    // ── TF ───────────────────────────────────────────────────────────────────
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ── Service ──────────────────────────────────────────────────────────────
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;

    // ── Diagnostics timer ────────────────────────────────────────────────────
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;

    // ── State tracking ───────────────────────────────────────────────────────
    int  last_tracking_state_{-1};
    int  lost_frame_count_{0};
    int  total_frames_{0};
    int  published_frames_{0};

    // After this many consecutive lost frames → attempt reinit
    static constexpr int kLostFrameThreshold = 30;
};