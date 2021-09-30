#include "GenTL.h"

#include "ArenaApi.h"
#include "SaveApi.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"

#include "GenICam.h"

#pragma GCC diagnostic pop

#include "ArenaApi.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

// store x, y, z data in mm and intensity for a given point
struct PointData
{
	double x;
	double y;
	double z;
	uint16_t intensity;
};

struct ColorInitialValue
{
  GenICam::gcstring triggerSourceInitial;
  GenICam::gcstring triggerModeInitial;
  GenICam::gcstring triggerSelectorInitial;
  GenICam::gcstring pixelFormatInitial;
  GenICam::gcstring exposureAutoInitial;

};

struct DepthInitialValue
{
  GenICam::gcstring triggerSourceInitial;
  GenICam::gcstring triggerModeInitial;
  GenICam::gcstring triggerSelectorInitial;
  GenICam::gcstring pixelFormatInitial;
  GenICam::gcstring operatingModeInitial;

};

struct ColorConfig
{
  double fps;
  bool trigger_mode;
  int64_t fetch_frame_timeout;
  std::string macAddress;
  int64_t resolution;
  std::string pixel_format;
  bool exposure_auto;
  double exposure_time;
  bool gain_auto;
  int64_t gain;
  bool whitebalance_auto;
  int64_t brightness;
  bool reverse_x;
  bool reverse_y;
};

struct DepthConfig
{
  double fps;
  bool trigger_mode;
  int64_t fetch_frame_timeout;
  std::string macAddress;
  int64_t resolution;
  double exposure_time;
  std::string pixel_format;
  int64_t detect_range;
  double detect_distance_min;
  int64_t amplitude_gain;
  bool confidence_threshold_enable;
  int64_t confidence_threshold_min;
  int64_t image_accumulation;
  std::string conversion_gain;
  bool flying_pixels_removal_enable;
  int64_t flying_pixels_distance_min;
  bool spatial_filter_enable;
};

struct ScaleAndOffset
{
  double scaleX;
  float offsetX;
  double scaleY;
  float offsetY;
  double scaleZ;
  float offsetZ;
};

class Lucid
{
  public:
    std::string macAddress_;
    std::string deviceType_;
    std::string deviceFamily_;
    GenICam::gcstring pixelFormat_;
    std::string deviceModelName_;
    ColorConfig colorConfig_;
    DepthConfig depthConfig_;
    int64_t fetch_frame_timeout_;
    std::string save_path_ = "/home/bot/JHY/Captured_Images/";

    Lucid(Arena::IDevice *pDevice, Arena::ISystem *pSystem, ColorConfig colorConfig);
    Lucid(Arena::IDevice *pDevice, Arena::ISystem *pSystem, DepthConfig depthConfig);
    ~Lucid();
    Arena::IDevice *GetDevice();

    void ConfigureCamera();
    void GetAndSaveImage();
    void StartStream();
    void TriggerArming();
    Arena::IImage *GetImage();
    cv::Mat ImageToCVMat(Arena::IImage *pImage);
    cv::Mat DepthToIntensityImage(Arena::IImage *pImage);
    pcl::PointCloud<pcl::PointXYZ> DepthToPcd(Arena::IImage *pImage);
    void SavePcd(pcl::PointCloud<pcl::PointXYZ> ptcloud, std::string filename);
    void SaveCVMat(cv::Mat cv_image, std::string filename);
    void StopStream();

  private:
    Arena::ISystem *pSystem_;
    Arena::IDevice *pDevice_;
    Arena::IImage *pImage_;
    int counter_;
    ColorInitialValue colorInitialValue_;
    DepthInitialValue depthInitialValue_;
    ScaleAndOffset scaleAndOffset_;

    void ConfigureHLTCamera();
    void ConfigurePHXCamera();
    void ConfigureTRICamera();
    void SaveDepthImage(Arena::IImage *pImage, const char *filename);
    void SaveColorImage(Arena::IImage *pImage, const char *filename);
    void SaveIntensityImage(Arena::IImage *pImage, const char *filename);
    void RequeueBuffer(Arena::IImage *pImage);
    void ReInitialDepthCamera();
    void ReInitialColorCamera();
    std::vector<PointData> ProcessDepthImage(Arena::IImage *pImage);
};

