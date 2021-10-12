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
    LucidManager();
    ~LucidManager();

    std::vector<dr::Lucid> *Init(const std::string &conf_file_path);
    dr::Lucid *CreateDevice(dr::Lucid::ColorConfig &colorConfig);
    dr::Lucid *CreateDevice(dr::Lucid::DepthConfig &depthConfig);
    dr::Lucid *GetDevice(std::string &macAddress);
    bool DestoryDevice(std::string &macAddress);
    bool DestoryAllDevice();

  private:
    std::map<std::string, Arena::DeviceInfo> deviceList_;
    std::vector<dr::Lucid *> activeDeviceList_;
    Arena::ISystem *pSystem_;
};
} // namespace dr