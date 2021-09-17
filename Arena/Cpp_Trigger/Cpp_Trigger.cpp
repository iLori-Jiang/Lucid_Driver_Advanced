/***************************************************************************************
 ***                                                                                 ***
 ***  Copyright (c) 2019, Lucid Vision Labs, Inc.                                    ***
 ***                                                                                 ***
 ***  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR     ***
 ***  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,       ***
 ***  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE    ***
 ***  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER         ***
 ***  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,  ***
 ***  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE  ***
 ***  SOFTWARE.                                                                      ***
 ***                                                                                 ***
 ***************************************************************************************/

#include "stdafx.h"
#include "GenTL.h"

#include "ArenaApi.h"
#include "SaveApi.h"

#ifdef __linux__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#endif

#include "GenICam.h"

#ifdef __linux__
#pragma GCC diagnostic pop
#endif

#include "ArenaApi.h"

#define TAB1 "  "
#define TAB2 "    "
#define TAB3 "        "

// Trigger: Introduction
//    This example introduces basic trigger configuration and use. In order to
//    configure trigger, enable trigger mode and set the source and selector. The
//    trigger must be armed before it is prepared to execute. Once the trigger is
//    armed, execute the trigger and retrieve an image.

// =-=-=-=-=-=-=-=-=-
// =-=- SETTINGS =-=-
// =-=-=-=-=-=-=-=-=-

// image timeout
#define TIMEOUT 10000

// pixel format
#define COLOR_PIXEL_FORMAT "BGR8"
#define DEPTH_PIXEL_FORMAT "Coord3D_ABCY16"
#define INTENSITY_PIXEL_FORMAT "Mono8"

// scale and offset of the image
#define SCALE 0.25f
#define OFFSET 0.00f

// =-=-=-=-=-=-=-=-=-
// =-=- EXAMPLE -=-=-
// =-=-=-=-=-=-=-=-=-

// store x, y, z data in mm and intensity for a given point
struct PointData
{
	int16_t x;
	int16_t y;
	int16_t z;
	int16_t intensity;
};

void SaveDepthImage(Arena::IImage *pImage, const char *filename)
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

void Depth2IntensityImage(Arena::IImage *pImage, const char *filename)
{
	// prepare info from input buffer
	size_t width = pImage->GetWidth();
	size_t height = pImage->GetHeight();
	size_t size = width * height;
	size_t srcBpp = pImage->GetBitsPerPixel();
	size_t srcPixelSize = srcBpp / 8;
	const uint8_t* pInput = pImage->GetData();
	const uint8_t* pIn = pInput;

	// minDepth z value is set to 32767 to guarantee closer points exist as this
	// is the largest value possible
	PointData minDepth = { 0, 0, 32767, 0 };
	PointData maxDepth = { 0, 0, 0, 0 };

	// find points with min and max z values
	std::cout << TAB3 << "Find points with min and max z values\n";

	// using strcmp to avoid conversion issue
	int compareResult_ABCY16s = strcmp(DEPTH_PIXEL_FORMAT, "Coord3D_ABCY16s"); // if they are equal compareResult_ABCY16s = 0
	int compareResult_ABCY16 = strcmp(DEPTH_PIXEL_FORMAT, "Coord3D_ABCY16");	 // if they are equal compareResult_ABCY16 = 0

	bool isSignedPixelFormat = false;

	// if PIXEL_FORMAT is equal to Coord3D_ABCY16s
	if (compareResult_ABCY16s == 0)
	{
		isSignedPixelFormat = true;

		for (size_t i = 0; i < size; i++)
		{
			// Extract point data to signed 16 bit integer
			//    The first channel is the x coordinate, second channel is the y
			//    coordinate, the third channel is the z coordinate and the
			//    fourth channel is intensity. We offset pIn by 2 for each
			//    channel because pIn is an 8 bit integer and we want to read it
			//    as a 16 bit integer.
			int16_t x = *reinterpret_cast<const int16_t*>(pIn);
			int16_t y = *reinterpret_cast<const int16_t*>((pIn + 2));
			int16_t z = *reinterpret_cast<const int16_t*>((pIn + 4));
			int16_t intensity = *reinterpret_cast<const int16_t*>((pIn + 6));

			// convert x, y and z values to mm using their coordinate scales
			x = int16_t(double(x) * SCALE);
			y = int16_t(double(y) * SCALE);
			z = int16_t(double(z) * SCALE);

			if (z < minDepth.z && z > 0)
			{
				minDepth.x = x;
				minDepth.y = y;
				minDepth.z = z;
				minDepth.intensity = intensity;
			}
			else if (z > maxDepth.z)
			{
				maxDepth.x = x;
				maxDepth.y = y;
				maxDepth.z = z;
				maxDepth.intensity = intensity;
			}

			pIn += srcPixelSize;
		}

		// display data
		std::cout << TAB3 << "Minimum depth point found with z distance of " << minDepth.z
				<< "mm and intensity " << minDepth.intensity << " at coordinates (" << minDepth.x
				<< "mm, " << minDepth.y << "mm)" << std::endl;

		std::cout << TAB3 << "Maximum depth point found with z distance of " << maxDepth.z
				<< "mm and intensity " << maxDepth.intensity << " at coordinates (" << maxDepth.x
				<< "mm, " << maxDepth.y << "mm)" << std::endl;
	}
	// if PIXEL_FORMAT is equal to Coord3D_ABCY16
	else if (compareResult_ABCY16 == 0)
	{
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
				x = uint16_t(double(x) * SCALE + OFFSET);
				y = uint16_t((double(y) * SCALE) + OFFSET);
				z = uint16_t(double(z) * SCALE);

				if (z < minDepth.z && z > 0)
				{
					minDepth.x = x;
					minDepth.y = y;
					minDepth.z = z;
					minDepth.intensity = intensity;
				}
				else if (z > maxDepth.z)
				{
					maxDepth.x = x;
					maxDepth.y = y;
					maxDepth.z = z;
					maxDepth.intensity = intensity;
				}
			}

			pIn += srcPixelSize;
		}

		// display data
		std::cout << TAB3 << "Minimum depth point found with z distance of " << minDepth.z
				<< "mm and intensity " << minDepth.intensity << " at coordinates (" << minDepth.x
				<< "mm, " << minDepth.y << "mm)" << std::endl;

		std::cout << TAB3 << "Maximum depth point found with z distance of " << maxDepth.z
				<< "mm and intensity " << maxDepth.intensity << " at coordinates (" << maxDepth.x
				<< "mm, " << maxDepth.y << "mm)" << std::endl;
	}
	else
	{
		std::cout << "This example requires the camera to be in either 3D image format Coord3D_ABCY16 or Coord3D_ABCY16s\n\n";
	}

}

void SaveColorImage(Arena::IImage *pImage, const char *filename)
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

void SaveIntensityImage(Arena::IImage *pImage, const char *filename)
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

void ConfigureDepthCamera(Arena::IDevice *pDevice, GenICam::gcstring pixelFormat)
{
	// get node values that will be changed in order to return their values at
	// the end of the example
	GenICam::gcstring triggerSelectorInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "TriggerSelector");
	GenICam::gcstring triggerModeInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "TriggerMode");
	GenICam::gcstring triggerSourceInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "TriggerSource");
	GenICam::gcstring pixelFormatInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "PixelFormat");;

	// validate if Scan3dCoordinateSelector node exists. If not - probaly not
	// Helios camera used running the example
	GenApi::CEnumerationPtr checkpCoordSelector = pDevice->GetNodeMap()->GetNode("Scan3dCoordinateSelector");
	if (!checkpCoordSelector)
	{
		std::cout << TAB1 << "Scan3dCoordinateSelector node is not found. Please make sure that Helios device is used for the example.\n";
		return;
	}

	// validate if Scan3dCoordinateOffset node exists. If not - probaly Helios
	// has an old firmware
	GenApi::CFloatPtr checkpCoord = pDevice->GetNodeMap()->GetNode("Scan3dCoordinateOffset");
	if (!checkpCoord)
	{
		std::cout << TAB1 << "Scan3dCoordinateOffset node is not found. Please update Helios firmware.\n";
		return;
	}

	// get node values that will be changed in order to return their values at the end of the example	  
	GenICam::gcstring operatingModeInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dOperatingMode");
	
	// Set pixel format
	// Warning: HLT003S-001 / Helios2 - has only Coord3D_ABCY16 in this case
	//    This example demonstrates data interpretation for both a signed or
	//    unsigned pixel format. Default PIXEL_FORMAT here is set to
	//    Coord3D_ABCY16 but this can be modified to be a signed pixel format
	//    by changing it to Coord3D_ABCY16s.
	std::cout << TAB1 << "Set " << pixelFormat << " to pixel format\n";
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "PixelFormat", pixelFormat);

	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dOperatingMode", "Distance5000mmMultiFreq");

	// Set intensity amplitude gain, range(0,20) will be ok
	Arena::SetNodeValue<double>(pDevice->GetNodeMap(), "Scan3dAmplitudeGain", 10);

	// Set trigger selector
	//    Set the trigger selector to FrameStart. When triggered, the device will
	//    start acquiring a single frame. This can also be set to
	//    AcquisitionStart or FrameBurstStart.
	std::cout << TAB1 << "Set trigger selector to FrameStart\n";

	Arena::SetNodeValue<GenICam::gcstring>(
			pDevice->GetNodeMap(),
			"TriggerSelector",
			"FrameStart");

	// Set trigger mode
	//    Enable trigger mode before setting the source and selector and before
	//    starting the stream. Trigger mode cannot be turned on and off while the
	//    device is streaming.
	std::cout << TAB1 << "Enable trigger mode\n";

	Arena::SetNodeValue<GenICam::gcstring>(
			pDevice->GetNodeMap(),
			"TriggerMode",
			"On");

	// Set trigger source
	//    Set the trigger source to software in order to trigger images without
	//    the use of any additional hardware. Lines of the GPIO can also be used
	//    to trigger.
	std::cout << TAB1 << "Set trigger source to Software\n";

	Arena::SetNodeValue<GenICam::gcstring>(
			pDevice->GetNodeMap(),
			"TriggerSource",
			"Software");
	
	Arena::SetNodeValue<int64_t>(pDevice->GetNodeMap(), "Scan3dImageAccumulation", 4);
	
	// enable stream auto negotiate packet size
	Arena::SetNodeValue<bool>(
		pDevice->GetTLStreamNodeMap(),
		"StreamAutoNegotiatePacketSize",
		true);

	// enable stream packet resend
	Arena::SetNodeValue<bool>(
		pDevice->GetTLStreamNodeMap(),
		"StreamPacketResendEnable",
		true);
}

void ConfigureColorCamera(Arena::IDevice *pDevice, GenICam::gcstring pixelFormat)
{
	// get node values that will be changed in order to return their values at
	// the end of the example
	GenICam::gcstring triggerSelectorInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "TriggerSelector");
	GenICam::gcstring triggerModeInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "TriggerMode");
	GenICam::gcstring triggerSourceInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "TriggerSource");
	GenICam::gcstring pixelFormatInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "PixelFormat");;

	// Set pixel format
	std::cout << TAB1 << "Set " << pixelFormat << " to pixel format\n";
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "PixelFormat", pixelFormat);

	// Set trigger selector
	//    Set the trigger selector to FrameStart. When triggered, the device will
	//    start acquiring a single frame. This can also be set to
	//    AcquisitionStart or FrameBurstStart.
	std::cout << TAB1 << "Set trigger selector to FrameStart\n";

	Arena::SetNodeValue<GenICam::gcstring>(
			pDevice->GetNodeMap(),
			"TriggerSelector",
			"FrameStart");

	// Set trigger mode
	//    Enable trigger mode before setting the source and selector and before
	//    starting the stream. Trigger mode cannot be turned on and off while the
	//    device is streaming.
	std::cout << TAB1 << "Enable trigger mode\n";

	Arena::SetNodeValue<GenICam::gcstring>(
			pDevice->GetNodeMap(),
			"TriggerMode",
			"On");

	// Set trigger source
	//    Set the trigger source to software in order to trigger images without
	//    the use of any additional hardware. Lines of the GPIO can also be used
	//    to trigger.
	std::cout << TAB1 << "Set trigger source to Software\n";

	Arena::SetNodeValue<GenICam::gcstring>(
			pDevice->GetNodeMap(),
			"TriggerSource",
			"Software");

	// Set auto exposure

	GenICam::gcstring exposureAutoInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "ExposureAuto");
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "ExposureAuto", "Continuous");
	
	// enable stream auto negotiate packet size
	Arena::SetNodeValue<bool>(
		pDevice->GetTLStreamNodeMap(),
		"StreamAutoNegotiatePacketSize",
		true);

	// enable stream packet resend
	Arena::SetNodeValue<bool>(
		pDevice->GetTLStreamNodeMap(),
		"StreamPacketResendEnable",
		true);
}

// demonstrates basic trigger configuration and use
// (1) sets trigger mode, source, and selector
// (2) starts stream
// (3) waits until trigger is armed
// (4) triggers image
// (5) gets image
// (6) requeues buffer
// (7) stops stream
void ConfigureTriggerAndAcquireImage(Arena::IDevice *pDevice, std::string deviceType, GenICam::gcstring pixelFormat)
{	
	// Configure the camera by its type
	if (deviceType == "depth")
	{
		if (pixelFormat == ""){pixelFormat = DEPTH_PIXEL_FORMAT;}
		ConfigureDepthCamera(pDevice, pixelFormat);
	}else if (deviceType == "color")
	{
		if (pixelFormat == ""){pixelFormat = COLOR_PIXEL_FORMAT;}
		ConfigureColorCamera(pDevice, pixelFormat);
	}
	
	// Start stream
	//    When trigger mode is off and the acquisition mode is set to stream
	//    continuously, starting the stream will have the camera begin acquiring
	//    a steady stream of images. However, with trigger mode enabled, the
	//    device will wait for the trigger before acquiring any.
	std::cout << TAB1 << "Start stream\n";

	pDevice->StartStream();

	int num = 0;
	while (true)
	{	
		std::cout << "\nPress enter to get next image\n";
		std::getchar();
		// Trigger Armed
		//    Continually check until trigger is armed. Once the trigger is armed, it
		//    is ready to be executed.
		std::cout << TAB2 << "Wait until trigger is armed\n";
		bool triggerArmed = false;

		do
		{
			triggerArmed = Arena::GetNodeValue<bool>(pDevice->GetNodeMap(), "TriggerArmed");
		} while (triggerArmed == false);

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
				pDevice->GetNodeMap(),
				"TriggerSoftware");

		std::cout << TAB2 << "Get image";

		Arena::IImage *pImage = pDevice->GetImage(TIMEOUT);

		std::cout << " (" << pImage->GetWidth() << "x" << pImage->GetHeight() << ")\n";
		std::string timestamp = std::to_string(pImage->GetTimestampNs());

		if (pixelFormat == DEPTH_PIXEL_FORMAT)
		{
			std::string file_name = "Captured_Images/Depth_Images/" + deviceType + std::to_string(num++) + "_" + timestamp + ".ply";
			std::cout << TAB2 << "save " << file_name << "\n";
			SaveDepthImage(pImage, file_name.c_str());
		}else if (pixelFormat == COLOR_PIXEL_FORMAT)
		{
			std::string file_name = "Captured_Images/Color_Images/" + deviceType + std::to_string(num++) + "_" + timestamp + ".png";
			std::cout << TAB2 << "save " << file_name << "\n";
			SaveColorImage(pImage, file_name.c_str());
		}else if (pixelFormat == INTENSITY_PIXEL_FORMAT)
		{
			std::string file_name = "Captured_Images/Intensity_Images/" + deviceType + std::to_string(num++) + "_" + timestamp + ".png";
			std::cout << TAB2 << "save " << file_name << "\n";
			SaveIntensityImage(pImage, file_name.c_str());
		}

		// requeue buffer
		std::cout << TAB2 << "Requeue buffer\n";

		pDevice->RequeueBuffer(pImage);

	}

	// Stop the stream
	std::cout << TAB1 << "Stop stream\n";

	pDevice->StopStream();

	/**
	 * @brief for future integrating
	// return nodes to their initial values
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "TriggerSource", triggerSourceInitial);
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "TriggerMode", triggerModeInitial);
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "TriggerSelector", triggerSelectorInitial);
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "PixelFormat", pixelFormatInitial);
	**/
}

// =-=-=-=-=-=-=-=-=-
// =- PREPARATION -=-
// =- & CLEAN UP =-=-
// =-=-=-=-=-=-=-=-=-

int main()
{
	// flag to track when an exception has been thrown
	bool exceptionThrown = false;

	std::cout << "Cpp_Trigger\n";

	std::string macAddress = "1c:0f:af:00:46:6f";	// Helios Depth camera
	// std::string macAddress = "1c:0f:af:0c:85:61";	// Phoneix Color camera

	GenICam::gcstring pixelFormat = INTENSITY_PIXEL_FORMAT;

	try
	{
		// prepare example
		Arena::ISystem *pSystem = Arena::OpenSystem();
		pSystem->UpdateDevices(1000);

		// traverse the devices and store lucid cameras list
		std::vector<Arena::DeviceInfo> deviceInfos = pSystem->GetDevices();
		std::vector<uint8_t> lucidList;
		for (uint8_t j=0; j<deviceInfos.size(); j++)
		{
			GenICam::gcstring LucidCamera = deviceInfos[j].VendorName();
      if (LucidCamera == "Lucid Vision Labs"){lucidList.push_back(j);}
		}
		if (lucidList.size() == 0)
		{
			std::cout << "\nNo camera connected\nPress enter to complete\n";
			std::getchar();
			return 0;
		}

		// traverse the lucid cameras list to find the device we want
		uint8_t index = 0;
		for (uint8_t j=0; j<lucidList.size(); j++)
		{
			GenICam::gcstring tempMacAddress = deviceInfos[lucidList[j]].MacAddressStr();
			if (macAddress == tempMacAddress.c_str()){index = j;}
		}
		std::cout << "Mac address of current selected device is: "<< macAddress << std::endl;
		Arena::IDevice *pDevice = pSystem->CreateDevice(deviceInfos[lucidList[index]]);

		// assign camera type
		std::string deviceType = "";
    GenICam::gcstring deviceModelName = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "DeviceModelName");
	  std::string deviceModelName_tmp = deviceModelName.c_str();
		if (deviceModelName_tmp.rfind("HLT", 0) == 0){deviceType = "depth";}
		else if (deviceModelName_tmp.rfind("PHX", 0) == 0){deviceType = "color";}
		std::cout << "Name of current selected device is: " << deviceModelName_tmp << std::endl;
    std::cout << "Type of current selected device is: "<< deviceType << std::endl;

		// run example
		std::cout << "Start shooting...\n\n";
		ConfigureTriggerAndAcquireImage(pDevice, deviceType, pixelFormat);
		std::cout << "\nShooting complete\n";

		// clean up example
		pSystem->DestroyDevice(pDevice);
		Arena::CloseSystem(pSystem);
	}
	catch (GenICam::GenericException &ge)
	{
		std::cout << "\nGenICam exception thrown: " << ge.what() << "\n";
		exceptionThrown = true;
	}
	catch (std::exception &ex)
	{
		std::cout << "Standard exception thrown: " << ex.what() << "\n";
		exceptionThrown = true;
	}
	catch (...)
	{
		std::cout << "Unexpected exception thrown\n";
		exceptionThrown = true;
	}

	std::cout << "Press enter to complete\n";
	std::getchar();

	if (exceptionThrown)
		return -1;
	else
		return 0;
}
