# Lucid RGB&D Cameras

## Installation

### Download Arena SDK for Lucid

Download `Arena SDK – Linux 64-bit` at: https://thinklucid.com/downloads-hub/, and extract it.

### Make library and build

Make the Arena SDK shared library files accessible by the run-time linker (ld.so or ld-linux.so) using:
```bash
$ cd PATH_TO_ArenaSDK/
$ sudo sh Arena_SDK.conf
```

And build it using:
```bash
$ cd PATH_TO_ArenaSDK/Examples/Arena
$ make
```

## Initial Configuration

### Set up Jumbo Frame

Check the camera Ethernet adapter id, for example `enp0s8`, using:
```bash
$ ip addr show

enp0s8: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc pfifo_fast state UP group default qlen 1000
   link/ether 10:7b:44:7d:6f:33 brd ff:ff:ff:ff:ff:ff
   inet 10.0.9.124/24 brd 10.0.9.255 scope global dynamic noprefixroute enp0s31f6
      valid_lft 81652sec preferred_lft 81652sec
   inet6 fe80::127b:44ff:fe7d:6f33/64 scope link noprefixroute 
      valid_lft forever preferred_lft forever
```

Sets `enp0s8` Ethernet adapter to an MTU size of 9000 to enable jumbo frames:
```bash
$ sudo ifconfig enp0s8 mtu 9000
```

NOTE: This will enable optimal performance on high-bandwidth cameras, and usually reduces CPU load on the host system.

### Enlarge receive buffers size

Install `ethtool` using:
```bash
$ sudo apt-get install ethtool
```

Query and set the maximum receive buffer size on `enp0s8` using:
```bash
$ sudo ethtool -g enp0s8
Ring parameters for enp0s8:
Pre-set maximums:
RX:         4096
RX Mini:    0
RX Jumbo:   0
TX:         4096
Current hardware settings:
RX:         256
RX Mini:    0
RX Jumbo:   0
TX:         256

$ sudo ethtool -G enp0s8 rx 4096
```

NOTE: Using the Ethernet adapter’s maximum supported receive buffer size is recommended. Some network adapter drivers or the operating system itself may set the receive buffer value to a low value by default, which may result in decreased performance.

### Enlarge the socket buffer size

Set the maximum socket buffer size to 32 MB using:
```bash
$ sudo sh -c "echo 'net.core.rmem_default=33554432' >> /etc/sysctl.conf"
$ sudo sh -c "echo 'net.core.rmem_max=33554432' >> /etc/sysctl.conf"
$ sudo sysctl -p
```

NOTE: On most standard Linux installations, the default maximum socket buffer size is a small number which may result in decreased streaming performance.

### IP Address assignment

The camera must be on the same subnet as the Ethernet adapter and have a valid IP address before use. Recommending using persistent camera IP address, though which is disabled in the default camera configuration.

Set up persistent camera IP address using IP Config Utility in Arena SDK:

1. Shows all connected cameras:
```bash
$ PATH_TO_ARENA_SDK/precompiledExamples/IpConfigUtility /list

[index] MAC             IP              SUBNET          GATEWAY                 IP CONFIG
[0]     1C0FAF000001    169.254.101.0   255.255.0.0     0.0.0.0                 DHCP= 1 Persistent Ip= 1 LLA = 1
```

2. Enable and set a persistent IP address `192.168.0.10` on a camera with the MAC address of `1C:0F:AF:00:00:01`, and disable DHCP:
```bash
$ PATH_TO_ARENA_SDK/precompiledExamples/IpConfigUtility /config -m 1C0FAF000001 -p true -d false
$ PATH_TO_ARENA_SDK/precompiledExamples/IpConfigUtility /persist -m 1C0FAF000001 -p true -a 192.168.0.10 -s 255.255.0.0 -g 0.0.0.0
```

3. Cut off and then reconnect the power to restart the camera.

## Usage

NOTE: you don't need to set ip address manually, current program will recognize the devices through mac address, which is permanet.

Developing driver location is in `PATH_TO_ARENA_SDK/Examples/Arena/src/`, where entry file is `entry.cpp`.

To run the file, use following command:
```bash
$ cd PATH_TO_ARENA_SDK/Examples/Arena/src/
$ make
$ cd ../../..
$ ./OutputDirectory/Linux/x64Release/entry
```

Press `Enter` key to take and save image.

## Config Explaination
```cpp
{
  // COMMON for Lucid cameras
  "common": {
    "fps": 5,
    "trigger_mode": true,
    "fetch_frame_timeout": 2000,
    "save_path": "/home/bot/Captured_Images/"
  },
  "cameras": {
    "camera_1": {
      // color for Lucid color cameras, i.e. Phoenix or Triton
      "type": "color",
      "mac": "1c:0f:af:0c:85:61",
      // normal resolution 1280*960, 640*480
      // option: 1280, 640, other values will assign the images to 2048*1536
      "resolution": 1280,
      // color pixel_format (1)rgb, (2)gray
      "pixel_format": "rgb",
      // exposure time unit in us, whether use auto exposure or set exposure time manually
      "exposure_auto": true,
      "exposure_time": 18111.9,
      // gain [0,24], whether use auto gain or set gain manually
      "gain_auto": true,
      "gain": 0,
      // white balance auto recommended set to false
      "whitebalance_auto": true,
      // brightness recommended set to 100
      "brightness": 100,
      // if reverse the image vertically
      "reverse_x": false,
      // if reverse the image horizontally
      "reverse_y": false
    },
    "camera_2": {
       // depth for Lucid depth cameras, i.e. Helios
      "type": "depth",
      "mac": "1c:0f:af:00:46:6f",
      // normal resolution 640*480, ONLY OPTION
      "resolution": 640,
      // depth pixel_format (1)cloud, (2)gray
      "pixel_format": "cloud",
      // exposure time selector, unit in us (1)1000, (2)250, (3)62.5
      "exposure_time": 250,
      // detect range (1)1250 ,(2)3000, (3)4000, (4)5000, (5)6000, (6)8300
      "detect_range": 3000,
      // min of the detect range, range from [0, 10000], detect range is [min, min+range]
      "detect_distance_min": 1000,
      // amplitude gain [0, 30]， affecting the intensity image
      "amplitude_gain": 10,
      // enable confidence threshold [0, 65535]
      "confidence_threshold_enable": true,
      "confidence_threshold_min": 500,
      // image accumulation [1, 32]
      "image_accumulation": 4,
      // conversion gain (1)low, (2)high
      "conversion_gain": "low",
      // flying pixels removal [0, 300]
      "flying_pixels_removal_enable": false,
      "flying_pixels_distance_min": 300,
      // spatial filter enable
      "spatial_filter_enable": true
    }
  }
}
```

## TIPS

Enable 'StreamPacketResendEnable' and 'StreamAutoNegotiatePacketSize' in the code to boost processing

```cpp
Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(),"StreamAutoNegotiatePacketSize",true);

Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(),"StreamPacketResendEnable",true)
```

And enable 'packetdelay' if necessary, aiming to reduce the packet loss rate:
```cpp
GenApi::CIntegerPtr pDeviceStreamChannelPacketDelay = pDevice->GetNodeMap()->GetNode("GevSCPD");
pDeviceStreamChannelPacketDelay->SetValue(4000);
```
