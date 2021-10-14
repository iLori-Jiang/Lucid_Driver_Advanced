#include "include/dr_lucid_manager.hh"

#define COLOR_PIXEL_FORMAT "BGR8"
#define DEPTH_PIXEL_FORMAT "Coord3D_ABCY16"
#define INTENSITY_PIXEL_FORMAT "Mono8"

bool on_start(std::vector<dr::Lucid *> &dr_lucid_list) 
{
	for (auto lucid : dr_lucid_list)
	{
		if(!lucid->ConfigureCamera()) {return false;}
		if(!lucid->StartStream()) {return false;}
	}
	return true;
}

bool on_get_image(std::vector<dr::Lucid *> &dr_lucid_list)
{
	cv::Mat color_image;
	cv::Mat ir_image;
	cv::Mat depth_image;
	std::vector<cv::Point3f> points;
	bool get_data_success = true;
	for (auto lucid : dr_lucid_list)
	{
		if(!lucid->TriggerArming()){get_data_success = false; break;}
		if(!lucid->GetImage()){get_data_success = false; break;}
		if(!lucid->ProcessImage()){get_data_success = false; break;}

		if (lucid->pixelFormat_ == COLOR_PIXEL_FORMAT) {color_image = lucid->color_;}
		else if (lucid->pixelFormat_ == INTENSITY_PIXEL_FORMAT) {ir_image = lucid->gray_;}
		else if (lucid->pixelFormat_ == DEPTH_PIXEL_FORMAT) {ir_image = lucid->gray_; points = lucid->cvpoints_; depth_image = lucid->depth_;}
		else {get_data_success = false; break;}

		if(!lucid->RequeueBuffer()){get_data_success = false; break;}
	}

	if (get_data_success)
	{
		cv::imshow("color", color_image);
		cv::waitKey(-1);
		cv::destroyAllWindows();
		cv::imshow("gray", ir_image);
		cv::waitKey(-1);
		cv::destroyAllWindows();
		cv::imshow("depth", depth_image);
		cv::waitKey(-1);
		cv::destroyAllWindows();
		return true;
	}
	else
	{
		return false;
	}
}

bool on_stop(std::vector<dr::Lucid *> &dr_lucid_list) 
{
	for (auto lucid : dr_lucid_list) 
	{
		if(!lucid->StopStream()) {return false;}
	}
	return true;
}

bool on_reset(std::vector<dr::Lucid *> &dr_lucid_list) 
{ 
	for (auto lucid : dr_lucid_list)
	{
		if(!lucid->ResetCamera()) {return false;}
	}
	return true;
}

int main()
{
  bool exceptionThrown = false;

  try
  {
    dr::LucidManager *lucidManager = new dr::LucidManager();
		std::string config_file_path = "/home/bot/JHY/lucid_test_WS/ArenaSDK_v0.1.54_Linux_x64/ArenaSDK_TEST/Examples/Arena/src/config/lucid.json";
		// std::vector<dr::Lucid *> dr_lucid_list = lucidManager->Init(config_file_path);

		std::vector<dr::Lucid *> dr_lucid_list;
		dr::Lucid::ColorConfig colorConfig;
		colorConfig.macAddress = "1c:0f:af:0c:85:61";
		colorConfig.save_path = "/home/bot/JHY/Captured_Images/";
		dr::Lucid::DepthConfig depthConfig;
		depthConfig.macAddress = "1c:0f:af:00:46:6f";
		depthConfig.save_path = "/home/bot/JHY/Captured_Images/";

		dr_lucid_list.push_back(lucidManager->CreateDevice(colorConfig));
		// dr_lucid_list.push_back(lucidManager->CreateDevice(depthConfig));

		on_start(dr_lucid_list);

		while (true)
		{
			std::cout << "\nPress enter to get next image\n";
			std::getchar();

			on_get_image(dr_lucid_list);
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