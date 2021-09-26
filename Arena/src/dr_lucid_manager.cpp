# include "include/dr_lucid_manager.hh"

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
		std::cout << "No Lucid camera connected!!!!" << std::endl;
	}
}

LucidManager::~LucidManager()
{
  Arena::CloseSystem(pSystem_);
}

Lucid *LucidManager::CreateDevice(std::string macAddress, std::string pixelFormat, ColorConfig colorConfig)
{ 
  std::map<std::string, Arena::DeviceInfo>::iterator it;
  it = deviceList_.find(macAddress);
  if (it != deviceList_.end())
  {
    Arena::IDevice *pDevice = pSystem_->CreateDevice(it->second);
    Lucid *lucid = new Lucid(pDevice, pSystem_, macAddress, pixelFormat, colorConfig);
    activeDeviceList_.push_back(lucid);
    return lucid;
  }
  else 
  {
    // EXCEPTION
    return NULL;
  }
}

Lucid *LucidManager::CreateDevice(std::string macAddress, std::string pixelFormat, DepthConfig depthConfig)
{ 
  std::map<std::string, Arena::DeviceInfo>::iterator it;
  it = deviceList_.find(macAddress);
  if (it != deviceList_.end())
  {
    Arena::IDevice *pDevice = pSystem_->CreateDevice(it->second);
    Lucid *lucid = new Lucid(pDevice, pSystem_, macAddress, pixelFormat, depthConfig);
    activeDeviceList_.push_back(lucid);
    return lucid;
  }
  else 
  {
    // EXCEPTION
    return NULL;
  }
}

Lucid *LucidManager::GetDevice(std::string macAddress)
{
  for (uint8_t i=0; i<activeDeviceList_.size(); ++i)
  {
    if (activeDeviceList_[i]->macAddress_ == macAddress)
    {
      return activeDeviceList_[i];
    }
  }
  // EXCEPTION
  return NULL;
}

bool LucidManager::DestoryDevice(std::string macAddress)
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
  return false;
}

bool LucidManager::DestoryAllDevice()
{
  for (uint8_t i=0; i<activeDeviceList_.size(); ++i)
  {
    pSystem_->DestroyDevice(activeDeviceList_[i]->GetDevice());
  }
  return true;
}
