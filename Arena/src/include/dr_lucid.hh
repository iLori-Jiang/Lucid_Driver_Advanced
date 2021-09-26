// #include "stdafx.h"
#include "GenTL.h"

#include "ArenaApi.h"
#include "SaveApi.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"

#include "GenICam.h"

#pragma GCC diagnostic pop

#include "ArenaApi.h"

// store x, y, z data in mm and intensity for a given point
struct PointData
{
	int16_t x;
	int16_t y;
	int16_t z;
	int16_t intensity;
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
  int64_t fps;
  bool trigger_mode;
  int64_t fetch_frame_timeout;
  std::string mac;
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

    Lucid(Arena::IDevice *pDevice, Arena::ISystem *pSystem, std::string macAddress, std::string pixelFormat, ColorConfig colorConfig);
    Lucid(Arena::IDevice *pDevice, Arena::ISystem *pSystem, std::string macAddress, std::string pixelFormat, DepthConfig depthConfig);
    ~Lucid();
    Arena::IDevice *GetDevice();

    void ConfigureCamera();
    void GetAndSaveImage();
    void StartStream();
    void TriggerArming();
    Arena::IImage *GetImage() const;
    void RequeueBuffer(Arena::IImage *pImage);
    void StopStream();

  private:
    Arena::ISystem *pSystem_;
    Arena::IDevice *pDevice_;
    Arena::IImage *pImage_;
    int counter_;
    ColorInitialValue colorInitialValue_;
    DepthInitialValue depthInitialValue_;

    void ConfigureHLTCamera();
    void ConfigurePHXCamera();
    void ConfigureTRICamera();
    void SaveDepthImage(Arena::IImage *pImage, const char *filename);
    void Depth2IntensityImage(Arena::IImage *pImage, const char *filename);
    void SaveColorImage(Arena::IImage *pImage, const char *filename);
    void SaveIntensityImage(Arena::IImage *pImage, const char *filename);
    void ReInitialDepthCamera();
    void ReInitialColorCamera();
};

