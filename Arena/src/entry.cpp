#include "include/dr_lucid_manager.hh"

int main()
{
  bool exceptionThrown = false;

	// load config

	// color config
	ColorConfig colorConfig={
		fps: 5,
		trigger_mode: true,
		fetch_frame_timeout: 2000,
		macAddress: "1c:0f:af:0c:85:61",
		resolution: 1280,
		pixel_format: "gray",
		exposure_auto: true,
		exposure_time: 18111.9,
		gain_auto: true,
		gain: 0,
		whitebalance_auto: false,
		brightness: 100,
		reverse_x: false,
		reverse_y: false
	};

	// depth config
	DepthConfig depthConfig={
		fps: 5,
		trigger_mode: true,
		fetch_frame_timeout: 2000,
		macAddress: "1c:0f:af:00:46:6f",
		resolution: 640,
		exposure_time: 1000,
		pixel_format: "cloud",
		detect_range: 3000,
		detect_distance_min: 1000,
		amplitude_gain: 30,
		confidence_threshold_enable: true,
    confidence_threshold_min: 500,
    image_accumulation: 4,
    conversion_gain: "low",
    flying_pixels_removal_enable: false,
    flying_pixels_distance_min: 300,
    spatial_filter_enable: true
	};

  try
  {
		// set up lucid manager
    LucidManager *lucidManager = new LucidManager();

		// set up lucid camera
		Lucid *phoneix = lucidManager->CreateDevice(colorConfig);
		Lucid *helios = lucidManager->CreateDevice(depthConfig);

		// configure and start camera
    helios->ConfigureCamera();
    helios->StartStream();
		phoneix->ConfigureCamera();
		phoneix->StartStream();
		// triton->ConfigureCamera();
		// triton->StartStream();

		// take images

		//while (true)
		//{
			std::cout << "\nPress enter to get next image\n";
			std::getchar();
			helios->TriggerArming();
			phoneix->TriggerArming();
			// triton->TriggerArming();

			helios->GetAndSaveImage();
			phoneix->GetAndSaveImage();
			// triton->GetAndSaveImage();
		//}

		// stop camera
    helios->StopStream();
		phoneix->StopStream();
		// triton->StopStream();
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
