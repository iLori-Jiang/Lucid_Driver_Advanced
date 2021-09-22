#include "stdafx.h"
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

class Lucid
{
  public:
    std::string macAddress_;
    std::string deviceType_;
    GenICam::gcstring pixelFormat_;
    std::string deviceModelName_;

    Lucid(Arena::IDevice *pDevice, Arena::ISystem *pSystem, std::string macAddress, std::string pixelFormat);
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
    ColorInitialValue *colorInitialValue_;
    DepthInitialValue *depthInitialValue_;

    void ConfigureDepthCamera();
    void ConfigureColorCamera();
    void SaveDepthImage(Arena::IImage *pImage, const char *filename);
    void Depth2IntensityImage(Arena::IImage *pImage, const char *filename);
    void SaveColorImage(Arena::IImage *pImage, const char *filename);
    void SaveIntensityImage(Arena::IImage *pImage, const char *filename);
    void ReInitialDepthCamera();
    void ReInitialColorCamera();
};

