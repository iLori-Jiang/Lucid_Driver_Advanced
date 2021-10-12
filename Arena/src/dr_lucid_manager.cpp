#include <fstream>
#include <nlohmann/json.hpp>

# include "include/dr_lucid_manager.hh"

namespace dr
{
/**
 * @brief constructor for lucid manager
 */
LucidManager::LucidManager()
{
  // prepare devices
	pSystem_ = Arena::OpenSystem();
	pSystem_->UpdateDevices(1000);

	// traverse the devices and store lucid cameras list
	std::vector<Arena::DeviceInfo> deviceInfos = pSystem_->GetDevices();
	for (uint8_t j=0; j<deviceInfos.size(); j++)
	{ 
    Arena::DeviceInfo device = deviceInfos[j];
		GenICam::gcstring deviceName = device.VendorName();
		if (deviceName == "Lucid Vision Labs"){
      std::string macAddress = device.MacAddressStr().c_str();
      deviceList_.insert(std::pair<std::string, Arena::DeviceInfo>(macAddress, device));
    }
	}
	if (deviceList_.empty())
	{
    // EXCEPTION
		//DRL_ERROR_STREAM("No Lucid camera connected!!!!");
    return;
	}
}

/**
 * @brief deconstrucot for lucid camera
 */
LucidManager::~LucidManager()
{
  Arena::CloseSystem(pSystem_);
}

/**
 * @brief set up certain color camera based on the config
 */
dr::Lucid *LucidManager::CreateDevice(dr::Lucid::ColorConfig &colorConfig)
{ 
  // use iterator to find the devie based on mac address
  std::map<std::string, Arena::DeviceInfo>::iterator it;
  it = deviceList_.find(colorConfig.macAddress);
  if (it != deviceList_.end())
  {
    // create camera device
    Arena::IDevice *pDevice = pSystem_->CreateDevice(it->second);
    dr::Lucid *lucid = new dr::Lucid(pDevice, pSystem_, colorConfig);
    activeDeviceList_.push_back(lucid);
    return lucid;
  }else{
    // EXCEPTION
    //DRL_ERROR_STREAM("No Lucid camera found on the mac address!!!!");
    return;
  }
}

/**
 * @brief set up certain depth camera based on the config
 */
dr::Lucid *LucidManager::CreateDevice(dr::Lucid::DepthConfig &depthConfig)
{ 
  // use iterator to find the devie based on mac address
  std::map<std::string, Arena::DeviceInfo>::iterator it;
  it = deviceList_.find(depthConfig.macAddress);
  if (it != deviceList_.end())
  {
    // create camera device
    Arena::IDevice *pDevice = pSystem_->CreateDevice(it->second);
    dr::Lucid *lucid = new dr::Lucid(pDevice, pSystem_, depthConfig);
    activeDeviceList_.push_back(lucid);
    return lucid;
  }else{
    // EXCEPTION
    //DRL_ERROR_STREAM("No Lucid camera found on the mac address!!!!");
    return;
  }
}

/**
 * @warning
 */
std::vector<dr::Lucid> *LucidManager::Init(const std::string &config_file_path)
{
  std::vector<dr::Lucid> *lucidList;

  try
  {
    std::ifstream conf_stream(config_file_path.c_str());
    nlohmann::json json_config;
    conf_stream >> json_config;

    for (auto camera : json_config.at("cameras"))
    {
      if (camera.at("type") == "color")
      {
        dr::Lucid::ColorConfig colorConfig;
        colorConfig.fps = json_config.at("common").at("fps");
        colorConfig.trigger_mode = json_config.at("common").at("trigger_mode");
        colorConfig.fetch_frame_timeout = json_config.at("common").at("fetch_frame_timeout");
        colorConfig.save_path = json_config.at("common").at("save_path");
        colorConfig.macAddress = camera.at("mac");
        colorConfig.resolution = camera.at("resolution");
        colorConfig.pixel_format = camera.at("pixel_format");
        colorConfig.exposure_auto = camera.at("exposure_auto");
        colorConfig.exposure_time = camera.at("exposure_time");
        colorConfig.gain_auto = camera.at("gain_auto");
        colorConfig.gain = camera.at("gain");
        colorConfig.whitebalance_auto = camera.at("whitebalance_auto");
        colorConfig.brightness = camera.at("brightness");
        colorConfig.reverse_x = camera.at("reverse_x");
        colorConfig.reverse_y = camera.at("reverse_y");

        dr::Lucid *color = CreateDevice(colorConfig);
        lucidList->push_back(color);
      }
      else if (camera.at("type") == "depth")
      {
        dr::Lucid::DepthConfig depthConfig;
        depthConfig.fps = json_config.at("common").at("fps");
        depthConfig.trigger_mode = json_config.at("common").at("trigger_mode");
        depthConfig.fetch_frame_timeout = json_config.at("common").at("fetch_frame_timeout");
        depthConfig.save_path = json_config.at("common").at("save_path");
        depthConfig.macAddress = camera.at("mac");
        depthConfig.resolution = 640;
        depthConfig.exposure_time = camera.at("exposure_time");
        depthConfig.pixel_format = camera.at("pixel_format");
        depthConfig.detect_range = camera.at("detect_range");
        depthConfig.detect_distance_min = camera.at("detect_distance_min");
        depthConfig.amplitude_gain = camera.at("amplitude_gain");
        depthConfig.confidence_threshold_enable = camera.at("confidence_threshold_enable");
        depthConfig.confidence_threshold_min = camera.at("confidence_threshold_min");
        depthConfig.image_accumulation = camera.at("image_accumulation");
        depthConfig.conversion_gain = camera.at("conversion_gain");
        depthConfig.flying_pixels_removal_enable = camera.at("flying_pixels_removal_enable");
        depthConfig.flying_pixels_distance_min = camera.at("flying_pixels_distance_min");
        depthConfig.spatial_filter_enable = camera.at("spatial_filter_enable");

        dr::Lucid *depth = CreateDevice(depthConfig);
        lucidList->push_back(depth);
      }else
      {
        //DRL_ERROR_STREAM("Invalid configuration for Lucid!!!! Please check sample config file.");
        return;
      }
    }
  }
  catch (...)
  {
    //DRL_ERROR_STREAM("Invalid configuration for Lucid!!!! Please check sample config file.");
    throw;
  }
  return lucidList;
};

/**
 * @brief get the certain device based on its mac address
 */
dr::Lucid *LucidManager::GetDevice(std::string &macAddress)
{
  for (uint8_t i=0; i<activeDeviceList_.size(); ++i)
  {
    if (activeDeviceList_[i]->macAddress_ == macAddress)
    {
      return activeDeviceList_[i];
    }
  }
  // EXCEPTION
  //DRL_ERROR_STREAM("No Lucid camera found on the mac address!!!!");
  return;
}

/**
 * @brief destory the certain device based on its mac address
 */
bool LucidManager::DestoryDevice(std::string &macAddress)
{
  for (uint8_t i=0; i<activeDeviceList_.size(); ++i)
  {
    if (activeDeviceList_[i]->macAddress_ == macAddress)
    {
      pSystem_->DestroyDevice(activeDeviceList_[i]->GetDevice());
      activeDeviceList_.erase(activeDeviceList_.begin()+i);
      return true;
    }
  }
  // EXCEPTION
  //DRL_ERROR_STREAM("No Lucid camera found on the mac address!!!!");
  return false;
}

/**
 * @brief destory all the active devices
 */
bool LucidManager::DestoryAllDevice()
{
  for (uint8_t i=0; i<activeDeviceList_.size(); ++i)
  {
    pSystem_->DestroyDevice(activeDeviceList_[i]->GetDevice());
  }
  return true;
}
} // namespace dr