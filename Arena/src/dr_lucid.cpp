#include "include/dr_lucid.hh"

namespace dr
{
#define TAB1 "  "
#define TAB2 "    "
#define TAB3 "        "
#define TAB4 "            "

// pixel format
#define COLOR_PIXEL_FORMAT "RGB8"
#define DEPTH_PIXEL_FORMAT "Coord3D_ABCY16"
#define INTENSITY_PIXEL_FORMAT "Mono8"

// default scale and offset of the camera
#define SCALE 0.25f
#define OFFSET 0.00f

/**
 * @brief constructor for color camera
 */
Lucid::Lucid(Arena::IDevice *pDevice, Arena::ISystem *pSystem, ColorConfig &colorConfig)
{
	colorConfig_ = colorConfig;
	macAddress_ = colorConfig.macAddress;
	fetch_frame_timeout_ = colorConfig.fetch_frame_timeout;
	save_path_ = colorConfig.save_path;
	is_reset_ = false;
	OpticalDistortionCoefficients distortionCoef_;

	// initial camera system and device
	pDevice_ = pDevice;
	pSystem_ = pSystem;
	counter_ = 0;

	// assign pixel format
	if (colorConfig.pixel_format == "rgb"){pixelFormat_ = COLOR_PIXEL_FORMAT;}
	else if (colorConfig.pixel_format == "gray"){pixelFormat_ = INTENSITY_PIXEL_FORMAT;}
	else {
		// EXCEPTION
		//DRL_ERROR_STREAM("Invalid pixel format config!!!!");
		return;
	}

	// if user want to save images
	if ((colorConfig.save_path == "") || (colorConfig.save_path == " ") || (colorConfig.save_path == "  ")) {save_flag_ = false;}
	else {save_flag_ = true;}

	// assign camera type and family
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
		//DRL_ERROR_STREAM("Current camera family is not supported!!!!");
		return;
	}
	
	// output camera information
	std::cout << "Mac address of current selected device is: "<< macAddress_ << std::endl;
	std::cout << "Name of current selected device is: " << deviceModelName_ << std::endl;
	std::cout << "Type of current selected device is: "<< deviceType_ << std::endl << std::endl;
}

/**
 * @brief constructor for depth camera
 */
Lucid::Lucid(Arena::IDevice *pDevice, Arena::ISystem *pSystem, DepthConfig &depthConfig)
{
	depthConfig_ = depthConfig;
	macAddress_ = depthConfig.macAddress;
	fetch_frame_timeout_ = depthConfig.fetch_frame_timeout;
	save_path_ = depthConfig.save_path;
	is_reset_ = false;
	OpticalDistortionCoefficients distortionCoef_;

	// initial camera system and device
	pDevice_ = pDevice;
	pSystem_ = pSystem;
	counter_ = 0;

	// assign pixel format
	if (depthConfig.pixel_format == "cloud"){pixelFormat_ = DEPTH_PIXEL_FORMAT;}
	else if (depthConfig.pixel_format == "gray"){pixelFormat_ = INTENSITY_PIXEL_FORMAT;}
	else {
		// EXCEPTION
		//DRL_ERROR_STREAM("Invalid pixel format config!!!!");
		return;
	}

	// if user want to save images
	if ((depthConfig.save_path == "") || (depthConfig.save_path == " ") || (depthConfig.save_path == "  ")) {save_flag_ = false;}
	else {save_flag_ = true;}

	// assign camera type and family
	GenICam::gcstring deviceModelName = Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "DeviceModelName");
	deviceModelName_ = deviceModelName.c_str();
	if (deviceModelName_.rfind("HLT", 0) == 0)
	{
		deviceType_ = "depth";
		deviceFamily_ = "Helios";
	}else{
		// EXCEPTION
		//DRL_ERROR_STREAM("Current camera family is not supported!!!!");
		return;
	}
	
	// output camera information
	std::cout << "Mac address of current selected device is: "<< macAddress_ << std::endl;
	std::cout << "Name of current selected device is: " << deviceModelName_ << std::endl;
	std::cout << "Type of current selected device is: "<< deviceType_ << std::endl << std::endl;
}

/**
 * @brief camera deconstructor
 */
Lucid::~Lucid()
{
	if (!is_reset_) {pSystem_->DestroyDevice(pDevice_);}
}

/**
 * @brief provide the private attribute for outer use
 */
Arena::IDevice *Lucid::GetDevice()
{
	return pDevice_;
}

/**
 * @brief save depth image, OFFICIAL FUNCTION
 * @note basic function
 */
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
	bool filterPoints = true; // default
	scaleAndOffset_.offsetZ = 0.0f;	  // default

	// set the output file format of the image writer to .ply
	writer.SetPly(".ply", filterPoints, isSignedPixelFormat, scaleAndOffset_.scaleX, scaleAndOffset_.offsetX, scaleAndOffset_.offsetY, scaleAndOffset_.offsetZ);

	// Save image
	//    Passing image data into the image writer using the cascading I/O
	//    operator (<<) triggers a save. Notice that the << operator accepts the
	//    image data as a constant unsigned 8-bit integer pointer (const
	//    uint8_t*) and the file name as a character string (const char*).
	std::cout << TAB3 << "Save image\n";

	writer << pImage->GetData();
}

/**
 * @brief save color image, OFFICIAL FUNCTION
 * @note basic function
 */
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

/**
 * @brief save intensity image, OFFICIAL FUNCTION
 * @note basic function
 */
void Lucid::SaveIntensityImage(Arena::IImage *pImage, const char *filename)
{	
	// check if the image in complete
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

/**
 * @brief configure Helios2 camera
 * @note basic function
 * @warning might occur error configuring other depth camera, with different config parameter
 */
void Lucid::ConfigureHLTCamera()
{
	// get node values that will be changed in order to return their values at
	// the end of the program
	depthInitialValue_.triggerSelectorInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "TriggerSelector");
	depthInitialValue_.triggerModeInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "TriggerMode");
	depthInitialValue_.triggerSourceInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "TriggerSource");
	depthInitialValue_.pixelFormatInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "PixelFormat");
	depthInitialValue_.operatingModeInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "Scan3dOperatingMode");

	// validate if Scan3dCoordinateSelector node exists. If not - probaly not
	// Helios camera used running the program
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

	// Get the coordinate scale in order to convert x, y and z values to mm as
	// well as the offset for x and y to correctly adjust values when in an
	// unsigned pixel format
	std::cout << TAB1 << "Get xyz coordinate scales and offsets\n\n";

	Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "Scan3dCoordinateSelector", "CoordinateA");
	// getting scaleX as float by casting since SetPly() will expect it passed as float
	scaleAndOffset_.scaleX = static_cast<float>(Arena::GetNodeValue<double>(pDevice_->GetNodeMap(), "Scan3dCoordinateScale"));
	// getting offsetX as float by casting since SetPly() will expect it passed as float
	scaleAndOffset_.offsetX = static_cast<float>(Arena::GetNodeValue<double>(pDevice_->GetNodeMap(), "Scan3dCoordinateOffset"));
	Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "Scan3dCoordinateSelector", "CoordinateB");
	scaleAndOffset_.scaleY = Arena::GetNodeValue<double>(pDevice_->GetNodeMap(), "Scan3dCoordinateScale");
	// getting offsetY as float by casting since SetPly() will expect it passed as float
	scaleAndOffset_.offsetY = static_cast<float>(Arena::GetNodeValue<double>(pDevice_->GetNodeMap(), "Scan3dCoordinateOffset"));
	Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "Scan3dCoordinateSelector", "CoordinateC");
	scaleAndOffset_.scaleZ = Arena::GetNodeValue<double>(pDevice_->GetNodeMap(), "Scan3dCoordinateScale");

	// Enable stream auto negotiate packet size
	Arena::SetNodeValue<bool>(
		pDevice_->GetTLStreamNodeMap(),
		"StreamAutoNegotiatePacketSize",
		true);

	// Enable stream packet resend
	Arena::SetNodeValue<bool>(
		pDevice_->GetTLStreamNodeMap(),
		"StreamPacketResendEnable",
		true);
}

/**
 * @brief configure Phoenix camera
 * @note basic function
 */
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

/**
 * @brief configure Triton camera
 * @note basic function
 */
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

/**
 * @brief automatically decide which camera to configure according to its family
 * @note will call other basic functions
 * @warning currently only support Helios, Phoenix, and Triton
 */
bool Lucid::ConfigureCamera()
{
	if (deviceFamily_ == "Helios"){ConfigureHLTCamera();}
	else if (deviceFamily_ == "Phoenix"){ConfigurePHXCamera();}
	else if (deviceFamily_ == "Triton"){ConfigureTRICamera();}
	else{
		// EXCEPTION
		return false;
	}
	return true;
}

/**
 * @brief camera should start stream first and then begin job on taking images
 * @note basic function
 * @warning all the configuration should be done before this step
 */
bool Lucid::StartStream()
{
	pDevice_->StartStream();
	return true;
}

/**
 * @brief before the camera take a image, it should arm its trigger and then shot the image
 * @note basic function
 */
bool Lucid::TriggerArming()
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

	return true;
}

/**
 * @brief after the camera take a image, it should requeue that image
 * @note basic function
 */
bool Lucid::RequeueBuffer()
{
	// requeue buffer
	std::cout << TAB2 << "Requeue buffer\n";
	pDevice_->RequeueBuffer(pImage_);
	
	return true;
}

/**
 * @brief let the camera get the image
 * @note basic function
 */
bool Lucid::GetImage()
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
	pImage_ = pDevice_->GetImage(fetch_frame_timeout_);
	std::cout << " (" << pImage_->GetWidth() << "x" << pImage_->GetHeight() << ")\n";

	return true;
}

/**
 * @brief transform image to cv::Mat
 * @note basic function
 */
bool Lucid::ImageToCVMat(Arena::IImage *pImage)
{
	// Transform the image based on the pixel format
	if (pixelFormat_ == COLOR_PIXEL_FORMAT)
	{	
		// Convert the image
		auto pConverted = Arena::ImageFactory::Convert(pImage, BGR8);
		// Transformation to format CV_8UC3, 8 bit unsigned int, 3 channels
		color_ = cv::Mat((int)pConverted->GetHeight(), (int)pConverted->GetWidth(), CV_8UC3, (void*)pConverted->GetData());
		return true;
	}
	else if (pixelFormat_ == INTENSITY_PIXEL_FORMAT)
	{
		// Transformation to format CV_8UC1, 8 bit unsigned int, 1 channels
		gray_ = cv::Mat((int)pImage->GetHeight(), (int)pImage->GetWidth(), CV_8UC1, (void*)pImage->GetData());
		return true;
	}
	else
	{
		// EXCEPTION
		return false;
	}
}

/**
 * @brief traverse all the points in the depth image, and store them using PointData vector
 * @note basic function
 */
bool Lucid::ProcessDepthImage(Arena::IImage *pImage)
{
	std::cout << TAB4 << "Ready to process depth image..." << std::endl;

	if (pixelFormat_ != DEPTH_PIXEL_FORMAT)
	{
		// EXCEPTION
		return false;
	}

	// initial vector
	std::vector<PointData> data_points;
	std::vector<double> z_list;

	// prepare info from input buffer
	size_t width = pImage->GetWidth();
	size_t height = pImage->GetHeight();
	size_t size = width * height;
	size_t srcBpp = pImage->GetBitsPerPixel();
	size_t srcPixelSize = srcBpp / 8;
	const uint8_t* pInput = pImage->GetData();
	const uint8_t* pIn = pInput;

	// prepare distance data
	const double z_min = depthConfig_.detect_distance_min;
	const double z_max = z_min + depthConfig_.detect_range;
	const double gray_min = 0;
	const double gray_max = 255;

	// traverse the points
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

		// Convert x, y and z to millimeters
		//    Using each coordinates' appropriate scales, convert x, y
		//    and z values to mm. For the x and y coordinates in an
		//    unsigned pixel format, we must then add the offset to our
		//    converted values in order to get the correct position in
		//    millimeters.
		double xSigned = double(x) * scaleAndOffset_.scaleX + scaleAndOffset_.offsetX;
		double ySigned = double(y) * scaleAndOffset_.scaleY + scaleAndOffset_.offsetY;
		double zSigned = double(z) * scaleAndOffset_.scaleZ;

		PointData point = {
			x: xSigned,
			y: ySigned,
			z: zSigned,
			intensity: intensity
		};

		data_points.push_back(point);

		// map z distance value to color range 16 bits
		z_list.push_back((gray_max - gray_min) / (z_max - z_min) * (zSigned - z_min));

		// loop increment
		pIn += srcPixelSize;
	}

	z_list_ = z_list;
	data_points_ = data_points;
	std::cout << TAB4 << "Process finished" << std::endl;

	return true;
}

/**
 * @brief transform depth image to intensity image and depth image in cv format, using intensity and z information from Coord3D_ABCY16 format
 * @note will call other basic functions
 */
bool Lucid::DepthToCVMat(std::vector<PointData> &data_points, int height, int width)
{	
	if (pixelFormat_ != DEPTH_PIXEL_FORMAT)
	{
		// EXCEPTION
		return false;
	}

	// initial cv::Mat
	gray_ = cv::Mat(height, width, CV_8UC1, cv::Scalar::all(0));
	depth_ = cv::Mat(height, width, CV_8UC1, cv::Scalar::all(0));
	xyz_ = cv::Mat(height, width, CV_32FC3);

	// traverse
	uchar *pointer_gray;					// tool for cv::Mat traverse
	uchar *pointer_depth;
	uint64_t counter = 0;					// counter for points
	for (int i=0; i<height; ++i)
	{
		pointer_gray = gray_.ptr(i);
		pointer_depth = depth_.ptr(i);
		for (int j=0; j<width; ++j)
		{	
			// assign intensity and z distance information
			pointer_gray[j] = data_points[counter].intensity * 255 / 65535;
			pointer_depth[j] = z_list_[counter];

			// assign xyz distance information
			xyz_.at<cv::Vec3f>(i, j)[0] = data_points[counter].x;
			xyz_.at<cv::Vec3f>(i, j)[1] = data_points[counter].y;
			xyz_.at<cv::Vec3f>(i, j)[2] = data_points[counter].z;

			counter += 1;
		}
	}
	std::cout << TAB4 << "Process depth image done" << std::endl;
	return true;
}

/**
 * @brief transform depth image to pcl pointcloud
 * @note will call other basic functions
 */
bool Lucid::DepthToPcd(std::vector<PointData> &data_points)
{
	if (pixelFormat_ != DEPTH_PIXEL_FORMAT)
	{
		// EXCEPTION
		return false;
	}

	// initial pcl pointcloud
	pcl::PointCloud<pcl::PointXYZ> ptcloud;

	// traverse assign the information of each point
	for (long unsigned int i=0; i<data_points.size(); ++i)
	{
		PointData data_point = data_points[i];
		pcl::PointXYZ pt;
		pt.x = data_point.x;
		pt.y = data_point.y;
		pt.z = data_point.z;
		ptcloud.points.push_back(pt);
	}

	// assign the size of the pointcloud
	ptcloud.height = 1;
	ptcloud.width = data_points.size();

	ptcloud_ = ptcloud;

	std::cout << TAB4 << "PCL point cloud generated" << std::endl;

	return true;
};

/**
 * @brief save pcl pointcloud to .pcd file
 * @note basic function
 */
void Lucid::SavePcd(pcl::PointCloud<pcl::PointXYZ> &ptcloud, std::string &filename)
{
	std::cout << TAB3 << "Save pcd image to " << filename << std::endl;

	pcl::io::savePCDFileASCII(filename, ptcloud);
}

/**
 * @brief save cv::Mat to .png file
 * @note basic function
 * @warning only save color and gray(intensity) image
 */
void Lucid::SaveCVMat(cv::Mat &cv_image, std::string &filename)
{
	std::cout << TAB3 << "Save cv matrix to " << filename << std::endl;
	cv::imwrite(filename, cv_image);
}

/**
 * @brief ask the camera to transform and save image based on pixel format
 * @note will call other basic functions
 */
bool Lucid::ProcessImage()
{	
	// Prepare the filename
	std::string timestamp = std::to_string(pImage_->GetTimestampNs());
	std::string filename = deviceFamily_ + "_" + timestamp + "_" + std::to_string(counter_++);
	
	// Save image based on the pixel format
	if (pixelFormat_ == COLOR_PIXEL_FORMAT)
	{
		filename = save_path_ + "Color_Images/" + filename + ".png";
		if(!ImageToCVMat(pImage_)){return false;}
		if(save_flag_){SaveCVMat(color_, filename);}
		// SaveColorImage(pImage_, filename.c_str());
		// std::cout << TAB2 << "save " << filename << "\n";
	}
	else if (pixelFormat_ == DEPTH_PIXEL_FORMAT)
	{
		if(!ProcessDepthImage(pImage_)){return false;}

		filename = save_path_ + "Depth_Images/" + filename;
		std::string pcd_filename = filename + ".pcd";
		std::string intensity_filename = filename + "_gray.png";
		std::string depth_filename = filename + "_depth.png";

		if(!DepthToPcd(data_points_)){return false;}
		if(!DepthToCVMat(data_points_, (int)pImage_->GetHeight(), (int)pImage_->GetWidth())){return false;}
		
		if(save_flag_){SavePcd(ptcloud_, pcd_filename); SaveCVMat(gray_, intensity_filename); SaveCVMat(depth_, depth_filename);}
		// SaveDepthImage(pImage_, filename.c_str());
		// std::cout << TAB2 << "save " << filename << "\n";
	}
	else if (pixelFormat_ == INTENSITY_PIXEL_FORMAT)
	{
		filename = save_path_ + "Intensity_Images/" + filename + ".png";
		if(!ImageToCVMat(pImage_)){return false;}
		if(save_flag_){SaveCVMat(gray_, filename);}
		// SaveIntensityImage(pImage_, filename.c_str());
		// std::cout << TAB2 << "save " << filename << "\n";
	}
	else{return false;}

	return true;
}

/**
 * @brief return nodes to their initial values of depth camera
 * @note basic function
 */
void Lucid::ReInitialDepthCamera()
{
	Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "TriggerSource", depthInitialValue_.triggerSourceInitial);
	Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "TriggerMode", depthInitialValue_.triggerModeInitial);
	Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "TriggerSelector", depthInitialValue_.triggerSelectorInitial);
	Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "PixelFormat", depthInitialValue_.pixelFormatInitial);
	Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "Scan3dOperatingMode", depthInitialValue_.operatingModeInitial);

	std::cout << "Return nodes to their initial values" << std::endl;
}

/**
 * @brief return nodes to their initial values of color camera
 * @note basic function
 */
void Lucid::ReInitialColorCamera()
{
	Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "TriggerSource", colorInitialValue_.triggerSourceInitial);
	Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "TriggerMode", colorInitialValue_.triggerModeInitial);
	Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "TriggerSelector", colorInitialValue_.triggerSelectorInitial);
	Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "PixelFormat", colorInitialValue_.pixelFormatInitial);
	Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "ExposureAuto", colorInitialValue_.exposureAutoInitial);

	std::cout << "Return nodes to their initial values" << std::endl;
}

/**
 * @brief the camera needs to stop stream after it finish its job
 * @note will call other basic functions
 */
bool Lucid::StopStream()
{
	// Stop the stream
	std::cout << "Stop stream\n";

	pDevice_->StopStream();

	// Return camera to its initial state
	/**
	if (deviceType_ == "depth"){ReInitialDepthCamera();}
	else if (deviceType_ == "color"){ReInitialColorCamera();}
	else {return false;}
	**/

	std::cout << std::endl;
	return true;
}

/**
 * @brief reset the camera
 * @note basic functions
 */
bool Lucid::ResetCamera()
{
	if (!is_reset_)
	{
		pSystem_->DestroyDevice(pDevice_);
		is_reset_ = true;
		return true;
	}
	else
	{
		return false;
	}
}
} // namespace dr