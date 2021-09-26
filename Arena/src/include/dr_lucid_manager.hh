#include <map>
#include <vector>

#include "dr_lucid.hh"

class LucidManager{
  public:
    LucidManager();
    ~LucidManager();

    Lucid *CreateDevice(ColorConfig colorConfig);
    Lucid *CreateDevice(DepthConfig depthConfig);
    Lucid *GetDevice(std::string macAddress);
    bool DestoryDevice(std::string macAddress);
    bool DestoryAllDevice();

  private:
    std::map<std::string, Arena::DeviceInfo> deviceList_;
    std::vector<Lucid *> activeDeviceList_;
    Arena::ISystem *pSystem_;
};