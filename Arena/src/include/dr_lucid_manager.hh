#include <map>
#include <vector>

#include "dr_lucid.hh"

namespace dr
{
/**
 * @brief manager for lucid camera
 */
class LucidManager{
  public:
    // if the active cameras has certain ability
    bool color_enable_;
    bool depth_enable_;
    bool gray_enable_;
    bool overlay_enable_;

    // overlap config, first two are for color camera, last two are the position relationship between color and depth
    cv::Mat cameraMatrix_;
	  cv::Mat distCoeffs_;
	  cv::Mat rotationVector_;
	  cv::Mat translationVector_;
    
    LucidManager();
    ~LucidManager();

    // ROS intefaces
    bool init(const std::string &conf_file_path);
    bool start();
    bool acquire_data(cv::Mat &color_image, cv::Mat &ir_image, cv::Mat &depth_image, pcl::PointCloud<pcl::PointXYZ> &ptcloud, cv::Mat &xyz_image);
    bool stop();
    bool reset();
    // bool get_rgb_camera_info(dr::CameraInfo& camera_info);
    // bool get_depth_camera_info(dr::CameraInfo& camera_info);
    bool overlay_color_depth(cv::Mat &color_image, cv::Mat &xyz_image, pcl::PointCloud<pcl::PointXYZRGB> &color_ptcloud);

    // common functions
    dr::Lucid *CreateDevice(dr::Lucid::ColorConfig &colorConfig);
    dr::Lucid *CreateDevice(dr::Lucid::DepthConfig &depthConfig);
    dr::Lucid *GetDevice(std::string &macAddress);
    bool DestoryDevice(std::string &macAddress);
    void DestoryAllDevice();

  private:
    std::map<std::string, Arena::DeviceInfo> deviceList_;
    std::vector<dr::Lucid *> activeDeviceList_;
    Arena::ISystem *pSystem_;
};
} // namespace dr