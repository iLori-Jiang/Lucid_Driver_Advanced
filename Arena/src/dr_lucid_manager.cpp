#include <fstream>

# include "include/json.hpp"
# include "include/dr_lucid_manager.hh"

namespace dr
{
#define COLOR_PIXEL_FORMAT "BGR8"
#define DEPTH_PIXEL_FORMAT "Coord3D_ABCY16"
#define INTENSITY_PIXEL_FORMAT "Mono8"

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

  color_enable_ = false;
  depth_enable_ = false;
  gray_enable_ = false;
}

/**
 * @brief deconstrucot for lucid camera
 */
LucidManager::~LucidManager()
{
  DestoryAllDevice();
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
    return NULL;
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
    return NULL;
  }
}

/**
 * @warning
 */
bool LucidManager::init(const std::string &config_file_path)
{
  std::vector<dr::Lucid *> activeDeviceList_;

  try
  {
    std::ifstream conf_stream(config_file_path.c_str());
    nlohmann::json json_config;
    conf_stream >> json_config;

    std::string overlay_config = json_config.at("common").at("overlay_config");
    if (overlay_config != "" || overlay_config != " " || overlay_config != "   ")
    {
      cv::FileStorage fs(overlay_config, cv::FileStorage::READ);
      fs["cameraMatrix"] >> cameraMatrix_;
      fs["distCoeffs"] >> distCoeffs_;
      fs["rotationVector"] >> rotationVector_;
      fs["translationVector"] >> translationVector_;
      fs.release();
      overlay_enable_ = true;
    }
    else {overlay_enable_ = false;}

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
        if (color != NULL) {activeDeviceList_.push_back(color);}
        else 
        {
          //DRL_ERROR_STREAM("Invalid mac for Lucid!!!! Please check config file.");
          return false;
        }

        if (camera.at("pixel_format") == "rgb") {color_enable_ = true;}
        else if (camera.at("pixel_format") == "gray") {gray_enable_ = true;}
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
        if (depth != NULL) {activeDeviceList_.push_back(depth);}
        else 
        {
          //DRL_ERROR_STREAM("Invalid mac for Lucid!!!! Please check config file.");
          return false;
        }

        if (camera.at("pixel_format") == "cloud") {depth_enable_ = true; gray_enable_ = true;}
        else if (camera.at("pixel_format") == "gray") {gray_enable_ = true;}
      }
      else
      {
        //DRL_ERROR_STREAM("Invalid configuration for Lucid!!!! Please check sample config file.");
        return false;
      }
    }
  }
  catch (...)
  {
    //DRL_ERROR_STREAM("Invalid configuration for Lucid!!!! Please check sample config file.");
    throw;
  }
  return true;
};

bool LucidManager::start()
{
  for (auto lucid : activeDeviceList_)
	{
		if(!lucid->ConfigureCamera()) {return false;}
		if(!lucid->StartStream()) {return false;}

    for (int i=0; i<5; ++i)
    {
      if(!lucid->TriggerArming()){return false;}
		  if(!lucid->GetImage()){return false;}
      if(!lucid->RequeueBuffer()){return false;}
    }
    std::cout << "\n";
	}
	return true;
}

bool LucidManager::acquire_data(cv::Mat &color_image, cv::Mat &ir_image, cv::Mat &depth_image, pcl::PointCloud<pcl::PointXYZ> &ptcloud, cv::Mat &xyz_image)
{
  for (auto lucid : activeDeviceList_)
	{
		if(!lucid->TriggerArming()){return false;}
		if(!lucid->GetImage()){return false;}
		if(!lucid->ProcessImage()){return false;}

		if (lucid->pixelFormat_ == COLOR_PIXEL_FORMAT) {color_image = lucid->color_;}
		else if (lucid->pixelFormat_ == INTENSITY_PIXEL_FORMAT) {ir_image = lucid->gray_;}
		else if (lucid->pixelFormat_ == DEPTH_PIXEL_FORMAT) 
    {ir_image = lucid->gray_; ptcloud = lucid->ptcloud_; depth_image = lucid->depth_; xyz_image = lucid->xyz_;}
		else {return false;}

		if(!lucid->RequeueBuffer()){return false;}
	}
  return true;
}

bool LucidManager::stop()
{
  for (auto lucid : activeDeviceList_) 
	{
		if(!lucid->StopStream()) {return false;}
	}
	return true;
}

/**
 * @brief destory all the active devices for reset
 */
bool LucidManager::reset()
{
  auto iter = activeDeviceList_.begin();
  while (iter != activeDeviceList_.end())
  {
    if(!(*iter)->ResetCamera()) {return false;}
    iter = activeDeviceList_.erase(iter);
  }

	return true;
}

/**
bool LucidManager::get_rgb_camera_info(dr::CameraInfo& camera_info)
{
  camera_info.frame_id = "lucid_color_optical_frame";

  camera_info.image_height = 1536;
  camera_info.image_width = 2048;

  camera_info.distortion_model = dr::distortion_models::RATIONAL_POLYNOMIAL;

  for (auto x = 0; x < 5; ++x)
  {
    camera_info.distortion_coefficients.push_back(0);
  }

  camera_info.intrinsic_matrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  camera_info.projection_matrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  camera_info.rotation_matrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};

  return true;
}

bool LucidManager::get_depth_camera_info(dr::CameraInfo& camera_info)
{
  camera_info.frame_id = "lucid_depth_optical_frame";

  camera_info.image_height = 480;
  camera_info.image_width = 640;

  camera_info.distortion_model = dr::distortion_models::RATIONAL_POLYNOMIAL;
  
  for (auto x = 0; x < 5; ++x)
  {
    camera_info.distortion_coefficients.push_back(0);
  }

  camera_info.intrinsic_matrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  camera_info.projection_matrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  camera_info.rotation_matrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};

  return true;
}
**/

bool LucidManager::overlay_color_depth(cv::Mat &color_image, cv::Mat &xyz_image, pcl::PointCloud<pcl::PointXYZRGB> &color_ptcloud)
{
  if (!overlay_enable_) {return false;}

  // reshape image matrix
  int size = xyz_image.rows * xyz_image.cols;
  cv::Mat xyz_points = xyz_image.reshape(3, size);
  
  // project points
  cv::Mat projected_points;
  cv::projectPoints(
    xyz_points,
    rotationVector_,
    translationVector_,
    cameraMatrix_,
    distCoeffs_,
    projected_points
  );

  // loop through projected points to access RGB data at those points
  for (int i = 0; i < width * height; i++)
  {
    unsigned int col = (unsigned int)std::round(projected_points.at<cv::Vec2f>(i)[0]);
    unsigned int row = (unsigned int)std::round(projected_points.at<cv::Vec2f>(i)[1]);

    // only handle appropriate points
    if (ros < 0 || col < 0 ||
			row >= static_cast<unsigned int>(color_image.rows) ||
			col >= static_cast<unsigned int>(color_image.cols))
			continue;

    // access corresponding XYZ and RGB data
    uchar R = color_image.at<cv::Vec3b>(row, col)[0];
    uchar G = color_image.at<cv::Vec3b>(row, col)[1];
    uchar B = color_image.at<cv::Vec3b>(row, col)[2];

    float X = xyz_image.at<cv::Vec3f>(i)[0];
    float Y = xyz_image.at<cv::Vec3f>(i)[1];
    float Z = xyz_image.at<cv::Vec3f>(i)[2];

    pcl::PointXYZRGB pt;
		pt.x = X;
		pt.y = Y;
		pt.z = Z;
    pt.r = R;
    pt.g = G;
    pt.b = B;
    color_ptcloud.points.push_back(pt);
  }

  return true;
}

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
  return NULL;
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
void LucidManager::DestoryAllDevice()
{
  auto iter = activeDeviceList_.begin();
  while (iter != activeDeviceList_.end())
  {
    pSystem_->DestroyDevice((*iter)->GetDevice());
    iter = activeDeviceList_.erase(iter);
  }
  std::cout << "Destroy all active devices complete." << std::endl;
}
} // namespace dr