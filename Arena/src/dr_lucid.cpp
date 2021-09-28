#include "stdafx.h"

#include "include/dr_lucid.hh"

#define TAB1 "  "
#define TAB2 "    "
#define TAB3 "        "

// pixel format
#define COLOR_PIXEL_FORMAT "BGR8"
#define DEPTH_PIXEL_FORMAT "Coord3D_ABCY16"
#define INTENSITY_PIXEL_FORMAT "Mono8"

// scale and offset of the image
#define SCALE 0.25f
#define OFFSET 0.00f

Lucid::Lucid(Arena::IDevice *pDevice, Arena::ISystem *pSystem, ColorConfig colorConfig)
{
	colorConfig_ = colorConfig;

	macAddress_ = colorConfig.macAddress;
	fetch_frame_timeout_ = colorConfig.fetch_frame_timeout;

	// initial camera system and device
	pDevice_ = pDevice;
	pSystem_ = pSystem;
	counter_ = 0;

	if (colorConfig.pixel_format == "rgb"){pixelFormat_ = COLOR_PIXEL_FORMAT;}
	else if (colorConfig.pixel_format == "gray"){pixelFormat_ = INTENSITY_PIXEL_FORMAT;}
	else {
		// EXCEPTION
		return;
	}

	// assign camera type
	GenICam::gcstring deviceModelName = Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "DeviceModelName");
	deviceModelName_ = deviceModelName.c_str();
	if (deviceModelName_.rfind("PHX", 0) == 0)
	{
		deviceType_ = "color";
		deviceFamily_ = "Phoenix";
	}
	else if (deviceModelName_.rfind("TRI", 0) == 0)
	{
		deviceType_ = "color";
		deviceFamily_ = "Triton";
	}else{
		// EXCEPTION
		return;
	}
	
	// output camera information
	std::cout << "Mac address of current selected device is: "<< macAddress_ << std::endl;
	std::cout << "Name of current selected device is: " << deviceModelName_ << std::endl;
	std::cout << "Type of current selected device is: "<< deviceType_ << std::endl << std::endl;
}

Lucid::Lucid(Arena::IDevice *pDevice, Arena::ISystem *pSystem, DepthConfig depthConfig)
{
	depthConfig_ = depthConfig;

	macAddress_ = depthConfig.macAddress;
	fetch_frame_timeout_ = depthConfig.fetch_frame_timeout;

	// initial camera system and device
	pDevice_ = pDevice;
	pSystem_ = pSystem;
	counter_ = 0;

	if (depthConfig.pixel_format == "cloud"){pixelFormat_ = DEPTH_PIXEL_FORMAT;}
	else if (depthConfig.pixel_format == "gray"){pixelFormat_ = INTENSITY_PIXEL_FORMAT;}
	else {
		// EXCEPTION
		return;
	}

	// assign camera type
	GenICam::gcstring deviceModelName = Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "DeviceModelName");
	deviceModelName_ = deviceModelName.c_str();
	if (deviceModelName_.rfind("HLT", 0) == 0)
	{
		deviceType_ = "depth";
		deviceFamily_ = "Helios";
	}else{
		// EXCEPTION
		return;
	}
	
	// output camera information
	std::cout << "Mac address of current selected device is: "<< macAddress_ << std::endl;
	std::cout << "Name of current selected device is: " << deviceModelName_ << std::endl;
	std::cout << "Type of current selected device is: "<< deviceType_ << std::endl << std::endl;
}

Lucid::~Lucid()
{
	pSystem_->DestroyDevice(pDevice_);
}

Arena::IDevice *Lucid::GetDevice()
{
	return pDevice_;
}

void Lucid::SaveDepthImage(Arena::IImage *pImage, const char *filename)
{
	bool isSignedPixelFormat = false;

	if ((pImage->GetPixelFormat() == Coord3D_ABC16s) || (pImage->GetPixelFormat() == Coord3D_ABCY16s))
	{
		isSignedPixelFormat = true;
	}

	// Prepare image parameters
	//    An image's width, height, and bits per pixel are required to save to
	//    disk. Its size and stride (i.e. pitch) can be calculated from those 3
	//    inputs. Notice that an image's size and stride use bytes as a unit
	//    while the bits per pixel uses bits.
	std::cout << TAB3 << "Prepare image parameters\n";

	Save::ImageParams params(
			pImage->GetWidth(),
			pImage->GetHeight(),
			pImage->GetBitsPerPixel());

	// Prepare image writer
	//    The image writer requires 3 arguments to save an image: the image's
	//    parameters, a specified file name or pattern, and the image data to
	//    save. Providing these should result in a successfully saved file on the
	//    disk. Because an image's parameters and file name pattern may repeat,
	//    they can be passed into the image writer's constructor.
	std::cout << TAB3 << "Prepare image writer\n";

	Save::ImageWriter writer(
			params,
			filename);

	// set default parameters for SetPly()
	bool filterPoints = true;
	float scale = SCALE;
	float offsetA = OFFSET;
	float offsetB = OFFSET;
	float offsetC = OFFSET;

	// set the output file format of the image writer to .ply
	writer.SetPly(".ply", filterPoints, isSignedPixelFormat, scale, offsetA, offsetB, offsetC);

	// Save image
	//    Passing image data into the image writer using the cascading I/O
	//    operator (<<) triggers a save. Notice that the << operator accepts the
	//    image data as a constant unsigned 8-bit integer pointer (const
	//    uint8_t*) and the file name as a character string (const char*).
	std::cout << TAB3 << "Save image\n";

	writer << pImage->GetData();
}

void Lucid::SaveColorImage(Arena::IImage *pImage, const char *filename)
{
	// Convert image
	//    Convert the image to a displayable pixel format. It is worth keeping in
	//    mind the best pixel and file formats for your application. This example
	//    converts the image so that it is displayable by the operating system.
	std::cout << TAB3 << "Convert image to " << GetPixelFormatName(BGR8) << "\n";

	auto pConverted = Arena::ImageFactory::Convert(pImage, BGR8);

	// Prepare image parameters
	//    An image's width, height, and bits per pixel are required to save to
	//    disk. Its size and stride (i.e. pitch) can be calculated from those 3
	//    inputs. Notice that an image's size and stride use bytes as a unit
	//    while the bits per pixel uses bits.
	std::cout << TAB3 << "Prepare image parameters\n";

	Save::ImageParams params(
		pConverted->GetWidth(),
		pConverted->GetHeight(),
		pConverted->GetBitsPerPixel());

	// Prepare image writer
	//    The image writer requires 3 arguments to save an image: the image's
	//    parameters, a specified file name or pattern, and the image data to
	//    save. Providing these should result in a successfully saved file on the
	//    disk. Because an image's parameters and file name pattern may repeat,
	//    they can be passed into the image writer's constructor.
	std::cout << TAB3 << "Prepare image writer\n";

	Save::ImageWriter writer(
		params,
		filename);

	// Save image
	//    Passing image data into the image writer using the cascading I/O
	//    operator (<<) triggers a save. Notice that the << operator accepts the
	//    image data as a constant unsigned 8-bit integer pointer (const
	//    uint8_t*) and the file name as a character string (const char*).
	std::cout << TAB3 << "Save image\n";

	writer << pConverted->GetData();

	// destroy converted image
	Arena::ImageFactory::Destroy(pConverted);
}

void Lucid::SaveIntensityImage(Arena::IImage *pImage, const char *filename)
{	
	// If the image in complete
	if (pImage->IsIncomplete())
	{
		std::cout << "This image is incomplete!" << std::endl;
	}

	// Prepare image parameters
	//    An image's width, height, and bits per pixel are required to save to
	//    disk. Its size and stride (i.e. pitch) can be calculated from those 3
	//    inputs. Notice that an image's size and stride use bytes as a unit
	//    while the bits per pixel uses bits.
	std::cout << TAB3 << "Prepare image parameters\n";

	Save::ImageParams params(
			pImage->GetWidth(),
			pImage->GetHeight(),
			pImage->GetBitsPerPixel());

	// Prepare image writer
	//    The image writer requires 3 arguments to save an image: the image's
	//    parameters, a specified file name or pattern, and the image data to
	//    save. Providing these should result in a successfully saved file on the
	//    disk. Because an image's parameters and file name pattern may repeat,
	//    they can be passed into the image writer's constructor.
	std::cout << TAB3 << "Prepare image writer\n";

	Save::ImageWriter writer(
			params,
			filename);

	// Save image
	//    Passing image data into the image writer using the cascading I/O
	//    operator (<<) triggers a save. Notice that the << operator accepts the
	//    image data as a constant unsigned 8-bit integer pointer (const
	//    uint8_t*) and the file name as a character string (const char*).
	std::cout << TAB3 << "Save image\n";

	writer << pImage->GetData();
}

void Lucid::ConfigureHLTCamera()
{
	// get node values that will be changed in order to return their values at
	// the end of the example
	depthInitialValue_.triggerSelectorInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "TriggerSelector");
	depthInitialValue_.triggerModeInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "TriggerMode");
	depthInitialValue_.triggerSourceInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "TriggerSource");
	depthInitialValue_.pixelFormatInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "PixelFormat");;

	// validate if Scan3dCoordinateSelector node exists. If not - probaly not
	// Helios camera used running the example
	GenApi::CEnumerationPtr checkpCoordSelector = pDevice_->GetNodeMap()->GetNode("Scan3dCoordinateSelector");
	if (!checkpCoordSelector)
	{
		// EXCEPTION
		std::cout << TAB1 << "Scan3dCoordinateSelector node is not found. Please make sure that Helios device is used.\n";
		return;
	}

	// validate if Scan3dCoordinateOffset node exists. If not - probaly Helios
	// has an old firmware
	GenApi::CFloatPtr checkpCoord = pDevice_->GetNodeMap()->GetNode("Scan3dCoordinateOffset");
	if (!checkpCoord)
	{
		// EXCEPTION
		std::cout << TAB1 << "Scan3dCoordinateOffset node is not found. Please update Helios firmware.\n";
		return;
	}

	// get node values that will be changed in order to return their values at the end of the example	  
	depthInitialValue_.operatingModeInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "Scan3dOperatingMode");
	
	// Set pixel format
	// Warning: HLT003S-001 / Helios2 - has only Coord3D_ABCY16 in this case
	//    This example demonstrates data interpretation for both a signed or
	//    unsigned pixel format. Default PIXEL_FORMAT here is set to
	//    Coord3D_ABCY16 but this can be modified to be a signed pixel format
	//    by changing it to Coord3D_ABCY16s.
	std::cout << TAB1 << "Set " << pixelFormat_ << " to pixel format\n";
	Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "PixelFormat", pixelFormat_);

	// Set scan 3D mode selector
	// Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "Scan3dModelSelector", "Processed");
	
	// Set exposure time
	if (depthConfig_.exposure_time == 1000)
	{
		Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "ExposureTimeSelector", "Exp1000Us");
	}
	else if (depthConfig_.exposure_time == 250)
	{
		Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "ExposureTimeSelector", "Exp250Us");
	}
	else if (depthConfig_.exposure_time == 62.5)
	{
		Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "ExposureTimeSelector", "Exp62_5Us");
	}
	else{
		// EXCEPTION
		return;
	}

	// Set detect range
	if (depthConfig_.detect_range == 1250)
	{
		Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "Scan3dOperatingMode", "Distance1250mmSingleFreq");
	}
	else if (depthConfig_.detect_range == 3000){
		Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "Scan3dOperatingMode", "Distance3000mmSingleFreq");
	}
	else if (depthConfig_.detect_range == 4000){
		Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "Scan3dOperatingMode", "Distance4000mmSingleFreq");
	}
	else if (depthConfig_.detect_range == 5000){
		Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "Scan3dOperatingMode", "Distance5000mmMultiFreq");
	}
	else if (depthConfig_.detect_range == 6000){
		Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "Scan3dOperatingMode", "Distance6000mmSingleFreq");
	}
	else if (depthConfig_.detect_range == 8300){
		Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "Scan3dOperatingMode", "Distance8300mmMultiFreq");
	}
	else{
		// EXCEPTION
		return;
	}

	// Set the min of detecting range, detect range [min, min+range]
	Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(), "Scan3dDistanceMin", depthConfig_.detect_distance_min);

	// Set intensity amplitude gain, range(0,20) will be ok
	Arena::SetNodeValue<double>(pDevice_->GetNodeMap(), "Scan3dAmplitudeGain", depthConfig_.amplitude_gain);

	// Set confidence threshold
	if (depthConfig_.confidence_threshold_enable)
	{
		Arena::SetNodeValue<bool>(pDevice_->GetNodeMap(), "Scan3dConfidenceThresholdEnable", true);
		Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(), "Scan3dConfidenceThresholdMin", depthConfig_.confidence_threshold_min);
	}else{
		Arena::SetNodeValue<bool>(pDevice_->GetNodeMap(), "Scan3dConfidenceThresholdEnable", false);
	}

	// Set image accumulation
	Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(), "Scan3dImageAccumulation", depthConfig_.image_accumulation);

	// Set conversion gain
	if (depthConfig_.conversion_gain == "low")
	{
		Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "ConversionGain", "Low");
	}else if (depthConfig_.conversion_gain == "high")
	{
		Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "ConversionGain", "High");
	}else{
		// EXCEPTION
		return;
	}

	// Set flying pixels removal
	if (depthConfig_.flying_pixels_removal_enable)
	{
		Arena::SetNodeValue<bool>(pDevice_->GetNodeMap(), "Scan3dFlyingPixelsRemovalEnable", true);
		Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(), "Scan3dFlyingPixelsDistanceThreshold", depthConfig_.flying_pixels_distance_min);
	}else{
		Arena::SetNodeValue<bool>(pDevice_->GetNodeMap(), "Scan3dFlyingPixelsRemovalEnable", false);
	}

	// Set spatial filter
	Arena::SetNodeValue<bool>(pDevice_->GetNodeMap(), "Scan3dSpatialFilterEnable", depthConfig_.spatial_filter_enable);

	// Set trigger selector
	//    Set the trigger selector to FrameStart. When triggered, the device will
	//    start acquiring a single frame. This can also be set to
	//    AcquisitionStart or FrameBurstStart.
	std::cout << TAB1 << "Set trigger selector to FrameStart\n";

	Arena::SetNodeValue<GenICam::gcstring>(
			pDevice_->GetNodeMap(),
			"TriggerSelector",
			"FrameStart");

	// Set trigger mode
	//    Enable trigger mode before setting the source and selector and before
	//    starting the stream. Trigger mode cannot be turned on and off while the
	//    device is streaming.
	std::cout << TAB1 << "Enable trigger mode\n";

	Arena::SetNodeValue<GenICam::gcstring>(
			pDevice_->GetNodeMap(),
			"TriggerMode",
			"On");

	// Set trigger source
	//    Set the trigger source to software in order to trigger images without
	//    the use of any additional hardware. Lines of the GPIO can also be used
	//    to trigger.
	std::cout << TAB1 << "Set trigger source to Software\n";

	Arena::SetNodeValue<GenICam::gcstring>(
			pDevice_->GetNodeMap(),
			"TriggerSource",
			"Software");

	// enable stream auto negotiate packet size
	Arena::SetNodeValue<bool>(
		pDevice_->GetTLStreamNodeMap(),
		"StreamAutoNegotiatePacketSize",
		true);

	// enable stream packet resend
	Arena::SetNodeValue<bool>(
		pDevice_->GetTLStreamNodeMap(),
		"StreamPacketResendEnable",
		true);
}

void Lucid::ConfigurePHXCamera()
{
	// get node values that will be changed in order to return their values at
	// the end of the example
	colorInitialValue_.triggerSelectorInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "TriggerSelector");
	colorInitialValue_.triggerModeInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "TriggerMode");
	colorInitialValue_.triggerSourceInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "TriggerSource");
	colorInitialValue_.pixelFormatInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "PixelFormat");;

	// Set pixel format
	std::cout << TAB1 << "Set " << pixelFormat_ << " to pixel format\n";
	Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "PixelFormat", pixelFormat_);

	// Set reverse
	Arena::SetNodeValue<bool>(pDevice_->GetNodeMap(), "ReverseX", colorConfig_.reverse_x);
	Arena::SetNodeValue<bool>(pDevice_->GetNodeMap(), "ReverseY", colorConfig_.reverse_y);

	// Set resolution
	if (colorConfig_.resolution == 1280)
	{
		Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(), "Width", 1280);
		Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(), "Height", 960);
		Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(), "OffsetX", 384);		// (2048-1280)/2
		Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(), "OffsetY", 288);		// (1536-960)/2
	}else if (colorConfig_.resolution == 640)
	{
		Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(), "Width", 640);
		Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(), "Height", 480);
		Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(), "OffsetX", 704);		// (2048-640)/2
		Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(), "OffsetY", 528);		// (1536-480)/2
	}else{
		// EXCEPTION
		return;
	}

	// Set frame rate
	Arena::SetNodeValue<bool>(pDevice_->GetNodeMap(), "AcquisitionFrameRateEnable", true);
	Arena::SetNodeValue<double>(pDevice_->GetNodeMap(), "AcquisitionFrameRate", colorConfig_.fps);

	// Set target brightness
	Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(), "TargetBrightness", colorConfig_.brightness);

	// Set auto exposure
	colorInitialValue_.exposureAutoInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "ExposureAuto");
	if (colorConfig_.exposure_auto)
	{
		Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "ExposureAuto", "Continuous");
		Arena::SetNodeValue<double>(pDevice_->GetNodeMap(), "ExposureAutoLowerLimit", 31);
		Arena::SetNodeValue<double>(pDevice_->GetNodeMap(), "ExposureAutoUpperLimit", 140700);
		Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "ExposureAutoAlgorithm", "Mean");
		Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(), "ExposureAutoDampingRaw", 230);
	}else{
		Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "ExposureAuto", "Off");
		Arena::SetNodeValue<double>(pDevice_->GetNodeMap(), "ExposureTime", colorConfig_.exposure_time);
	}	

	// Set gain
	if (colorConfig_.gain_auto)
	{
		Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "GainAuto", "Continuous");
		Arena::SetNodeValue<double>(pDevice_->GetNodeMap(), "GainAutoLowerLimit", 0);
		Arena::SetNodeValue<double>(pDevice_->GetNodeMap(), "GainAutoUpperLimit", 24);
	}else{
		Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "GainAuto", "Off");
		Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(), "Gain", colorConfig_.gain);
	}

	// Set black level
	Arena::SetNodeValue<double>(pDevice_->GetNodeMap(), "BlackLevel", 0);
	Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(), "BlackLevelRaw", 0);

	// Set white balance auto
	// NOTE: turn it off when the environment is settled down
	Arena::SetNodeValue<bool>(pDevice_->GetNodeMap(), "BalanceWhiteEnable", true);
	if (colorConfig_.whitebalance_auto)
	{
		Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "BalanceWhiteAuto", "Continuous");
		Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "BalanceWhiteAutoAnchorSelector", "MaxRGB");
		Arena::SetNodeValue<bool>(pDevice_->GetNodeMap(), "AwbWhitePatchEnable", true);
	}else{
		Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "BalanceWhiteAuto", "Off");
	}

	// Set gamma
	Arena::SetNodeValue<bool>(pDevice_->GetNodeMap(), "GammaEnable", true);
	Arena::SetNodeValue<double>(pDevice_->GetNodeMap(), "Gamma", 1);

	// Set color transformation
	Arena::SetNodeValue<bool>(pDevice_->GetNodeMap(), "ColorTransformationEnable", true);
	Arena::SetNodeValue<double>(pDevice_->GetNodeMap(), "ColorTransformationValue", 1.5889);

	// Set defect correction
	Arena::SetNodeValue<bool>(pDevice_->GetNodeMap(), "DefectCorrectionEnable", true);

	// Set trigger selector
	//    Set the trigger selector to FrameStart. When triggered, the device will
	//    start acquiring a single frame. This can also be set to
	//    AcquisitionStart or FrameBurstStart.
	std::cout << TAB1 << "Set trigger selector to FrameStart\n";

	Arena::SetNodeValue<GenICam::gcstring>(
			pDevice_->GetNodeMap(),
			"TriggerSelector",
			"FrameStart");

	// Set trigger mode
	//    Enable trigger mode before setting the source and selector and before
	//    starting the stream. Trigger mode cannot be turned on and off while the
	//    device is streaming.
	std::cout << TAB1 << "Enable trigger mode\n";

	Arena::SetNodeValue<GenICam::gcstring>(
			pDevice_->GetNodeMap(),
			"TriggerMode",
			"On");

	// Set trigger source
	//    Set the trigger source to software in order to trigger images without
	//    the use of any additional hardware. Lines of the GPIO can also be used
	//    to trigger.
	std::cout << TAB1 << "Set trigger source to Software\n";

	Arena::SetNodeValue<GenICam::gcstring>(
			pDevice_->GetNodeMap(),
			"TriggerSource",
			"Software");

	// enable stream auto negotiate packet size
	Arena::SetNodeValue<bool>(
		pDevice_->GetTLStreamNodeMap(),
		"StreamAutoNegotiatePacketSize",
		true);

	// enable stream packet resend
	Arena::SetNodeValue<bool>(
		pDevice_->GetTLStreamNodeMap(),
		"StreamPacketResendEnable",
		true);
}

void Lucid::ConfigureTRICamera()
{
	// get node values that will be changed in order to return their values at
	// the end of the example
	colorInitialValue_.triggerSelectorInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "TriggerSelector");
	colorInitialValue_.triggerModeInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "TriggerMode");
	colorInitialValue_.triggerSourceInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "TriggerSource");
	colorInitialValue_.pixelFormatInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "PixelFormat");;

	// Set pixel format
	std::cout << TAB1 << "Set " << pixelFormat_ << " to pixel format\n";
	Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "PixelFormat", pixelFormat_);

	// Set reverse
	Arena::SetNodeValue<bool>(pDevice_->GetNodeMap(), "ReverseX", colorConfig_.reverse_x);
	Arena::SetNodeValue<bool>(pDevice_->GetNodeMap(), "ReverseY", colorConfig_.reverse_y);

	// Set resolution
	if (colorConfig_.resolution == 1280)
	{
		Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(), "Width", 1280);
		Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(), "Height", 960);
		Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(), "OffsetX", 384);		// (2048-1280)/2
		Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(), "OffsetY", 288);		// (1536-960)/2
	}else if (colorConfig_.resolution == 640)
	{
		Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(), "Width", 640);
		Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(), "Height", 480);
		Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(), "OffsetX", 704);		// (2048-640)/2
		Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(), "OffsetY", 528);		// (1536-480)/2
	}else{
		// EXCEPTION
		return;
	}

	// Set frame rate
	Arena::SetNodeValue<bool>(pDevice_->GetNodeMap(), "AcquisitionFrameRateEnable", true);
	Arena::SetNodeValue<double>(pDevice_->GetNodeMap(), "AcquisitionFrameRate", colorConfig_.fps);

	// Set target brightness
	Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(), "TargetBrightness", colorConfig_.brightness);

	// Set auto exposure
	// Set auto exposure
	colorInitialValue_.exposureAutoInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "ExposureAuto");
	if (colorConfig_.exposure_auto)
	{
		Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "ExposureAuto", "Continuous");
		Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "ExposureAutoLimitAuto", "Continuous");
		Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "ExposureAutoAlgorithm", "Mean");
		Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(), "ExposureAutoDampingRaw", 230);
	}else{
		Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "ExposureAuto", "Off");
		Arena::SetNodeValue<double>(pDevice_->GetNodeMap(), "ExposureTime", colorConfig_.exposure_time);
	}

	// Set gain
	if (colorConfig_.gain_auto)
	{
		Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "GainAuto", "Continuous");
		Arena::SetNodeValue<double>(pDevice_->GetNodeMap(), "GainAutoLowerLimit", 0);
		Arena::SetNodeValue<double>(pDevice_->GetNodeMap(), "GainAutoUpperLimit", 24);
	}else{
		Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "GainAuto", "Off");
		Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(), "Gain", colorConfig_.gain);
	}

	// Set black level
	Arena::SetNodeValue<double>(pDevice_->GetNodeMap(), "BlackLevel", 0);
	Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(), "BlackLevelRaw", 0);

	// Set white balance auto
	// NOTE: turn it off when the environment is settled down
	Arena::SetNodeValue<bool>(pDevice_->GetNodeMap(), "BalanceWhiteEnable", true);
	if (colorConfig_.whitebalance_auto)
	{
		Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "BalanceWhiteAuto", "Continuous");
		Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "BalanceWhiteAutoAnchorSelector", "MaxRGB");
		Arena::SetNodeValue<bool>(pDevice_->GetNodeMap(), "AwbWhitePatchEnable", true);
	}else{
		Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "BalanceWhiteAuto", "Off");
	}

	// Set gamma
	Arena::SetNodeValue<bool>(pDevice_->GetNodeMap(), "GammaEnable", true);
	Arena::SetNodeValue<double>(pDevice_->GetNodeMap(), "Gamma", 1);

	// Set color transformation
	Arena::SetNodeValue<bool>(pDevice_->GetNodeMap(), "ColorTransformationEnable", true);
	Arena::SetNodeValue<double>(pDevice_->GetNodeMap(), "ColorTransformationValue", 1.5889);

	// Set defect correction
	Arena::SetNodeValue<bool>(pDevice_->GetNodeMap(), "DefectCorrectionEnable", true);

	// Set trigger selector
	//    Set the trigger selector to FrameStart. When triggered, the device will
	//    start acquiring a single frame. This can also be set to
	//    AcquisitionStart or FrameBurstStart.
	std::cout << TAB1 << "Set trigger selector to FrameStart\n";

	Arena::SetNodeValue<GenICam::gcstring>(
			pDevice_->GetNodeMap(),
			"TriggerSelector",
			"FrameStart");

	// Set trigger mode
	//    Enable trigger mode before setting the source and selector and before
	//    starting the stream. Trigger mode cannot be turned on and off while the
	//    device is streaming.
	std::cout << TAB1 << "Enable trigger mode\n";

	Arena::SetNodeValue<GenICam::gcstring>(
			pDevice_->GetNodeMap(),
			"TriggerMode",
			"On");

	// Set trigger source
	//    Set the trigger source to software in order to trigger images without
	//    the use of any additional hardware. Lines of the GPIO can also be used
	//    to trigger.
	std::cout << TAB1 << "Set trigger source to Software\n";

	Arena::SetNodeValue<GenICam::gcstring>(
			pDevice_->GetNodeMap(),
			"TriggerSource",
			"Software");

	// enable stream auto negotiate packet size
	Arena::SetNodeValue<bool>(
		pDevice_->GetTLStreamNodeMap(),
		"StreamAutoNegotiatePacketSize",
		true);

	// enable stream packet resend
	Arena::SetNodeValue<bool>(
		pDevice_->GetTLStreamNodeMap(),
		"StreamPacketResendEnable",
		true);
}

void Lucid::ConfigureCamera()
{
	if (deviceFamily_ == "Helios"){ConfigureHLTCamera();}
	else if (deviceFamily_ == "Phoenix"){ConfigurePHXCamera();}
	else if (deviceFamily_ == "Triton"){ConfigureTRICamera();}
	else{
		// EXCEPTION
		return;
	}
}

void Lucid::StartStream()
{
	pDevice_->StartStream();
}

void Lucid::TriggerArming()
{
	// Trigger Armed
	//    Continually check until trigger is armed. Once the trigger is armed, it
	//    is ready to be executed.
	std::cout << TAB2 << "Wait until trigger is armed\n";
	bool triggerArmed = false;

	do
	{
		triggerArmed = Arena::GetNodeValue<bool>(pDevice_->GetNodeMap(), "TriggerArmed");
	} while (triggerArmed == false);
}

void Lucid::RequeueBuffer(Arena::IImage *pImage)
{
	// requeue buffer
	std::cout << TAB2 << "Requeue buffer\n";

	pDevice_->RequeueBuffer(pImage);
}

Arena::IImage *Lucid::GetImage() const
{
	// Trigger an image
	//    Trigger an image manually, since trigger mode is enabled. This triggers
	//    the camera to acquire a single image. A buffer is then filled and moved
	//    to the output queue, where it will wait to be retrieved.
	std::cout << TAB2 << "Trigger image\n";

	// Get image
	//    Once an image has been triggered, it can be retrieved. If no image has
	//    been triggered, trying to retrieve an image will hang for the duration
	//    of the timeout and then throw an exception.

	Arena::ExecuteNode(
			pDevice_->GetNodeMap(),
			"TriggerSoftware");

	std::cout << TAB2 << "Get image";
	*pImage = *pDevice_->GetImage(fetch_frame_timeout_);
	std::cout << " (" << pImage->GetWidth() << "x" << pImage->GetHeight() << ")\n";

	RequeueBuffer(pImage);

	return pImage;
}

cv::Mat *Lucid::Image2CVMat(Arena::IImage *pImage)
{
	if (pixelFormat_ == COLOR_PIXEL_FORMAT)
	{
		auto pConverted = Arena::ImageFactory::Convert(pImage, COLOR_PIXEL_FORMAT);
		cv::Mat *cv_image = cv::Mat((int)pConverted->GetHeight(), (int)pConverted->GetWidth(), CV_8UC3, (void*)pConverted->GetData());
		return cv_image;
	}else if (pixelFormat_ == INTENSITY_PIXEL_FORMAT)
	{
		cv::Mat *cv_image = cv::Mat((int)pImage->GetHeight(), (int)pImage->GetWidth(), CV_8UC1, (void*)pImage->GetData());
		return cv_image;
	}else if (pixelFormat_ == DEPTH_PIXEL_FORMAT)
	{
		cv::Mat *cv_image = cv::Mat((int)pImage->GetHeight(), (int)pImage->GetWidth(), CV_32FC3, (uint16_t*)pImage->GetData());
		return cv_image;
	}else
	{
		// EXCEPTION
		return;
	}
}

std::vector<PointData> *Lucid::ProcessDepthImage(Arena::IImage *pImage)
{
	if (pixelFormat_ != DEPTH_PIXEL_FORMAT)
	{
		// EXCEPTION
		return;
	}

	std::vector<PointData> *cloud_points;

	GenApi::INodeMap* pNodeMap = pDevice_->GetNodeMap();
	// get the coordinate scale in order to convert x, y and z values to mm as
	// well as the offset for x and y to correctly adjust values when in an
	// unsigned pixel format
	std::cout << TAB1 << "Get xyz coordinate scales and offsets\n\n";

	Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dCoordinateSelector", "CoordinateA");
	// getting scaleX as float by casting since SetPly() will expect it passed as
	// float
	float scaleX = static_cast<float>(Arena::GetNodeValue<double>(pNodeMap, "Scan3dCoordinateScale"));
	// getting offsetX as float by casting since SetPly() will expect it passed
	// as float
	float offsetX = static_cast<float>(Arena::GetNodeValue<double>(pNodeMap, "Scan3dCoordinateOffset"));
	Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dCoordinateSelector", "CoordinateB");
	double scaleY = Arena::GetNodeValue<double>(pNodeMap, "Scan3dCoordinateScale");
	// getting offsetY as float by casting since SetPly() will expect it passed
	// as float
	float offsetY = static_cast<float>(Arena::GetNodeValue<double>(pNodeMap, "Scan3dCoordinateOffset"));
	Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dCoordinateSelector", "CoordinateC");
	double scaleZ = Arena::GetNodeValue<double>(pNodeMap, "Scan3dCoordinateScale");

	// prepare info from input buffer
	size_t width = pImage->GetWidth();
	size_t height = pImage->GetHeight();
	size_t size = width * height;
	size_t srcBpp = pImage->GetBitsPerPixel();
	size_t srcPixelSize = srcBpp / 8;
	const uint8_t* pInput = pImage->GetData();
	const uint8_t* pIn = pInput;

	for (size_t i = 0; i < size; i++)
	{
		// Extract point data to signed 16 bit integer
		//    The first channel is the x coordinate, second channel is the y
		//    coordinate, the third channel is the z coordinate and the
		//    fourth channel is intensity. We offset pIn by 2 for each
		//    channel because pIn is an 8 bit integer and we want to read it
		//    as a 16 bit integer.
		uint16_t x = *reinterpret_cast<const uint16_t*>(pIn);
		uint16_t y = *reinterpret_cast<const uint16_t*>((pIn + 2));
		uint16_t z = *reinterpret_cast<const uint16_t*>((pIn + 4));
		uint16_t intensity = *reinterpret_cast<const uint16_t*>((pIn + 6));

		// if z is less than max value, as invalid values get filtered to
		// 65535
		if (z < 65535)
		{
			// Convert x, y and z to millimeters
			//    Using each coordinates' appropriate scales, convert x, y
			//    and z values to mm. For the x and y coordinates in an
			//    unsigned pixel format, we must then add the offset to our
			//    converted values in order to get the correct position in
			//    millimeters.
			x = uint16_t(double(x) * scaleX + offsetX);
			y = uint16_t(double(y) * scaleY + offsetY);
			z = uint16_t(double(z) * scaleZ);
		}

		PointData point = {
			x: x,
			y: y,
			z: z,
			intensity: intensity
		};

		cloud_points.push_back(point);

		pIn += srcPixelSize;
	}
	return cloud_points;
}

cv::Mat *Lucid::DepthToIntensityImage(Arena::IImage *pImage)
{	
	if (pixelFormat_ != DEPTH_PIXEL_FORMAT)
	{
		// EXCEPTION
		return;
	}

	size_t height = (int)pImage->GetHeight();
	size_t width = (int)pImage->GetWidth();
	
	auto *data_points = ProcessDepthImage(pImage);
	cv::Mat *cv_image(height, width, CV_8UC1, cv::Scalar::all(0));

	uchar *pointer;
	uint64_t counter = 0;
	for (int i=0; i<height; ++i)
	{
		pointer = cv_image.ptr(i);
		for (int j=0; j<width; ++j)
		{
			pointer[j] = data_points[counter].intensity;
			counter += 1;
		}
	}

	return cv_image;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Lucid::DepthToPcd(Arena::IImage *pImage)
{
	if (pixelFormat_ != DEPTH_PIXEL_FORMAT)
	{
		// EXCEPTION
		return;
	}

	auto *data_points = ProcessDepthImage(pImage);
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloud(new pcl::PointCloud<pcl::PointXYZ>);

	for (auto point : data_points)
	{
		pcl::PointXYZ pt;
		pt.x = point.x;
		pt.y = point.y;
		pt.z = point.z;
		ptcloud->points.push_back(pt);
	}

	return ptcloud;
};

void Lucid::SavePcd(pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloud, const char *filename)
{
	std::cout << TAB3 << "Save image\n";

	pcl::io::savePCDFileASCII (filename, ptcloud);
}

void Lucid::SaveCVMat(cv::Mat *cv_image, const char *filename)
{
	if (cv_image.type().channels() == 0) || (cv_image.type().channels() == 16)
	{
		cv::imwrite(filename, cv_image);
	}else if (cv_image.type().channels() == 21)
	{
		// SAVE DEPTH
	}
}

void Lucid::GetAndSaveImage()
{	
	// Get image
	//    Once an image has been triggered, it can be retrieved. If no image has
	//    been triggered, trying to retrieve an image will hang for the duration
	//    of the timeout and then throw an exception.

	Arena::ExecuteNode(
			pDevice_->GetNodeMap(),
			"TriggerSoftware");

	std::cout << TAB2 << "Get image";
	pImage_ = pDevice_->GetImage(fetch_frame_timeout_);
	std::cout << " (" << pImage_->GetWidth() << "x" << pImage_->GetHeight() << ")\n";
	
	std::string timestamp = std::to_string(pImage_->GetTimestampNs());
	std::string filename = deviceFamily_ + "_" + timestamp + "_" + std::to_string(counter_++);
	
	if (pixelFormat_ == COLOR_PIXEL_FORMAT)
	{
		filename = "/home/bot/JHY/lucid_test_WS/ArenaSDK_v0.1.54_Linux_x64/ArenaSDK_TEST/Captured_Images/Color_Images/" + filename + ".png";
		SaveColorImage(pImage_, filename.c_str());
		std::cout << TAB2 << "save " << filename << "\n";
	}
	else if (pixelFormat_ == DEPTH_PIXEL_FORMAT)
	{
		filename = "/home/bot/JHY/lucid_test_WS/ArenaSDK_v0.1.54_Linux_x64/ArenaSDK_TEST/Captured_Images/Depth_Images/" + filename;
		filename = filename + ".pcd";
		SavePcd(DepthToPcd(pImage_), filename.c_str());
		// SaveDepthImage(pImage_, filename.c_str());
		std::cout << TAB2 << "save " << filename << "\n";
	}
	else if (pixelFormat_ == INTENSITY_PIXEL_FORMAT)
	{
		filename = "/home/bot/JHY/lucid_test_WS/ArenaSDK_v0.1.54_Linux_x64/ArenaSDK_TEST/Captured_Images/Intensity_Images/" + filename + ".png";
		SaveIntensityImage(pImage_, filename.c_str());
		std::cout << TAB2 << "save " << filename << "\n";
	}

	// requeue buffer
	std::cout << TAB2 << "Requeue buffer\n";

	pDevice_->RequeueBuffer(pImage_);
}

void Lucid::ReInitialDepthCamera()
{
	// return nodes to their initial values
	Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "TriggerSource", depthInitialValue_.triggerSourceInitial);
	Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "TriggerMode", depthInitialValue_.triggerModeInitial);
	Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "TriggerSelector", depthInitialValue_.triggerSelectorInitial);
	Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "PixelFormat", depthInitialValue_.pixelFormatInitial);
	Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "Scan3dOperatingMode", depthInitialValue_.operatingModeInitial);

	std::cout << "Return nodes to their initial values" << std::endl;
}

void Lucid::ReInitialColorCamera()
{
	// return nodes to their initial values
	Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "TriggerSource", colorInitialValue_.triggerSourceInitial);
	Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "TriggerMode", colorInitialValue_.triggerModeInitial);
	Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "TriggerSelector", colorInitialValue_.triggerSelectorInitial);
	Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "PixelFormat", colorInitialValue_.pixelFormatInitial);
	Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "ExposureAuto", colorInitialValue_.exposureAutoInitial);

	std::cout << "Return nodes to their initial values" << std::endl;
}

void Lucid::StopStream()
{
	// Stop the stream
	std::cout << TAB1 << "Stop stream\n";

	pDevice_->StopStream();

	// Return camera to its initial state
	if (deviceType_ == "depth"){ReInitialDepthCamera();}
	else if (deviceType_ == "color"){ReInitialColorCamera();}
}
