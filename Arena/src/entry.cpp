#include "include/dr_lucid_manager.hh"
#include<pcl/visualization/cloud_viewer.h>

int main()
{
  bool exceptionThrown = false;

	// load config

	// color config
	dr::Lucid::ColorConfig colorConfig={
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
	dr::Lucid::DepthConfig depthConfig={
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
    dr::LucidManager *lucidManager = new dr::LucidManager();
		std::string config_file_path = "/config/lucid.json";
		std::vector<dr::Lucid *> dr_lucid_list = lucidManager->Init(config_file_path);
		on_start(dr_lucid_list);

		while (true)
		{
			std::cout << "\nPress enter to get next image\n";
			std::getchar();

			cv::Mat color_image;
			cv::Mat ir_image;
			cv::Mat depth_image;
			std::vector<cv::Point3f> points;
			bool get_data_success = true;
			for (auto lucid : dr_lucid_list)
			{
				if(!lucid->TriggerArming()){get_data_success = false; break;}
				if(!lucid->GetImage()){get_data_success = false; break;}
				if(!lucid->OutputImage()){get_data_success = false; break;}

				if (lucid.pixelFormat_ == COLOR_PIXEL_FORMAT) {color_image = lucid.color_;}
				else if (lucid.pixelFormat_ == INTENSITY_PIXEL_FORMAT) {ir_image = lucid.gray_;}
				else if (lucid.pixelFormat_ == DEPTH_PIXEL_FORMAT) {ir_image = lucid.gray_; points = lucid.cvpoints_; depth_image = lucid.depth_;}
				else {get_data_success = false; break;}

				if(!lucid->RequeueBuffer()){get_data_success = false; break;}
			}

			cv::imshow("color", color_image);
			cv::waitKey(-1);
			cv::destroyAllWindows();
			cv::imshow("gray", ir_image);
			cv::waitKey(-1);
			cv::destroyAllWindows();
			cv::imshow("depth", depth_image);
			cv::waitKey(-1);
			cv::destroyAllWindows();
		}

    on_stop(dr_lucid_list);
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

bool on_start(std::vector<dr::Lucid *> dr_lucid_list) 
{
	for (auto lucid : dr_lucid_list)
	{
		if(!lucid->ConfigureCamera()) {return false;}
		if(!lucid->StartStream()) {return false;}
	}
	return true;
}

bool on_stop(std::vector<dr::Lucid *> dr_lucid_lis) 
{
	for (auto lucid : dr_lucid_list) 
	{
		if(!lucid->StopStream()) {return false;}
	}
	return true;
}

bool on_reset(std::vector<dr::Lucid *> dr_lucid_list) 
{ 
	for (auto lucid : dr_lucid_list)
	{
		if(!lucid->ResetCamera()) {return false;}
	}
	return true;
}