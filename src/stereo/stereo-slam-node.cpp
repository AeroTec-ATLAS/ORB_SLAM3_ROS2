#include "stereo-slam-node.hpp"
#include <opencv2/core/core.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

StereoSlamNode::StereoSlamNode(ORB_SLAM3::System* pSLAM,
                               const string &strSettingsFile,
                               const string &strDoRectify)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    stringstream ss(strDoRectify);
    ss >> boolalpha >> doRectify;

    if (doRectify) {
        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if (!fsSettings.isOpened()) {
            cerr << "ERROR: Wrong path to settings" << endl;
            assert(0);
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"]  >> K_l;
        fsSettings["RIGHT.K"] >> K_r;
        fsSettings["LEFT.P"]  >> P_l;
        fsSettings["RIGHT.P"] >> P_r;
        fsSettings["LEFT.R"]  >> R_l;
        fsSettings["RIGHT.R"] >> R_r;
        fsSettings["LEFT.D"]  >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() ||
            R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0) {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0,3).colRange(0,3),
                                    cv::Size(cols_l, rows_l), CV_32F, M1l, M2l);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0,3).colRange(0,3),
                                    cv::Size(cols_r, rows_r), CV_32F, M1r, M2r);
    }

    left_sub  = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "camera/left");
    right_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "camera/right");

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(
        approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);

    // Publishers
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("orb_slam3/camera_pose", 10);
    map_pub_  = this->create_publisher<sensor_msgs::msg::PointCloud2>("orb_slam3/map_points", 10);
}

StereoSlamNode::~StereoSlamNode()
{
    m_SLAM->Shutdown();
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void StereoSlamNode::GrabStereo(const ImageMsg::SharedPtr msgLeft,
                                 const ImageMsg::SharedPtr msgRight)
{
    // Convert ROS images to OpenCV
    try { cv_ptrLeft  = cv_bridge::toCvShare(msgLeft);  }
    catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (left): %s", e.what());
        return;
    }
    try { cv_ptrRight = cv_bridge::toCvShare(msgRight); }
    catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (right): %s", e.what());
        return;
    }

    // Run SLAM — single call, capture the returned pose
    Sophus::SE3f Tcw;
    if (doRectify) {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,  imLeft,  M1l, M2l, cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image, imRight, M1r, M2r, cv::INTER_LINEAR);
        Tcw = m_SLAM->TrackStereo(imLeft, imRight,
                                  Utility::StampToSec(msgLeft->header.stamp));
    } else {
        Tcw = m_SLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image,
                                  Utility::StampToSec(msgLeft->header.stamp));
    }

    std_msgs::msg::Header header;
    header.stamp    = msgLeft->header.stamp;
    header.frame_id = "map";

    // ── Publish pose ─────────────────────────────────────────────────────────
    // Tcw is world-to-camera; invert to get camera-in-world (Twc)
    if (!Tcw.translation().isZero()) {
        Sophus::SE3f    Twc = Tcw.inverse();
        Eigen::Vector3f t   = Twc.translation();
        Eigen::Quaternionf q = Twc.unit_quaternion();

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header       = header;
        pose_msg.pose.position.x    = t.x();
        pose_msg.pose.position.y    = t.y();
        pose_msg.pose.position.z    = t.z();
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();
        pose_pub_->publish(pose_msg);
    }

    // ── Publish map points ────────────────────────────────────────────────────
    // GetAllMapPoints() returns every point in the active map
    std::vector<ORB_SLAM3::MapPoint*> map_points = m_SLAM->GetTrackedMapPoints();
    if (!map_points.empty()) {
        map_pub_->publish(MapPointsToPointCloud2(map_points, header));
    }
}

// ── Helper: pack map points into PointCloud2 ─────────────────────────────────
sensor_msgs::msg::PointCloud2 StereoSlamNode::MapPointsToPointCloud2(
    const std::vector<ORB_SLAM3::MapPoint*> &map_points,
    const std_msgs::msg::Header &header)
{
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header      = header;
    cloud.height      = 1;
    cloud.is_dense    = false;
    cloud.is_bigendian = false;
    cloud.point_step  = 12;  // 3 × float32

    cloud.fields.resize(3);
    cloud.fields[0].name     = "x"; cloud.fields[0].offset = 0;
    cloud.fields[1].name     = "y"; cloud.fields[1].offset = 4;
    cloud.fields[2].name     = "z"; cloud.fields[2].offset = 8;
    for (auto &f : cloud.fields) {
        f.datatype = sensor_msgs::msg::PointField::FLOAT32;
        f.count    = 1;
    }

    std::vector<float> data;
    data.reserve(map_points.size() * 3);

    for (auto* mp : map_points) {
        if (!mp || mp->isBad()) continue;
        Eigen::Vector3f pos = mp->GetWorldPos();
        data.push_back(pos.x());
        data.push_back(pos.y());
        data.push_back(pos.z());
    }

    cloud.width    = data.size() / 3;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.data.resize(cloud.row_step);
    memcpy(cloud.data.data(), data.data(), cloud.data.size());

    return cloud;
}