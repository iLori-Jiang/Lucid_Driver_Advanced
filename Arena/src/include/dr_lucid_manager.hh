#include "stdafx.h"
#include "GenTL.h"

#include "ArenaApi.h"
#include "SaveApi.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"

#include "GenICam.h"

#pragma GCC diagnostic pop

#include "ArenaApi.h"

#include <map>
#include <vector>

#include "dr_lucid.hh"

class LucidManager{
  public:
    LucidManager();
    ~LucidManager();

    Lucid *CreateDevice(std::string macAddress, std::string pixelFormat);
    Lucid *GetDevice(std::string macAddress);
    bool DestoryDevice(std::string macAddress);
    bool DestoryAllDevice();

  private:
    std::map<std::string, Arena::DeviceInfo> deviceList_;
    std::vector<Lucid *> activeDeviceList_;
    Arena::ISystem *pSystem_;
};