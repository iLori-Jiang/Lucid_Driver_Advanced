#include "include/dr_lucid_manager.hh"

int main()
{
  bool exceptionThrown = false;

  try
  {
    dr::LucidManager *lucidManager = new dr::LucidManager();
		std::string config_file_path = "/home/bot/JHY/lucid_test_WS/ArenaSDK_v0.1.54_Linux_x64/ArenaSDK_TEST/Examples/Arena/src/config/lucid.json";
		if(lucidManager->init(config_file_path))
		{
			lucidManager->start();

			// while (true)
			// {
				std::cout << "\nPress enter to get next image\n";
				std::getchar();

				cv::Mat color_image;
				cv::Mat ir_image;
				cv::Mat depth_image;
				pcl::PointCloud<pcl::PointXYZ> ptcloud;
				cv::Mat xyz_image;
				if (lucidManager->acquire_data(color_image, ir_image, depth_image, ptcloud, xyz_image))
				{
					if (lucidManager->color_enable_)
					{
						cv::imshow("color", color_image);
						cv::waitKey(-1);
						cv::destroyAllWindows();
					}
					
					if (lucidManager->gray_enable_)
					{
						cv::imshow("gray", ir_image);
						cv::waitKey(-1);
						cv::destroyAllWindows();
					}
					
					if (lucidManager->depth_enable_)
					{
						cv::imshow("depth", depth_image);
						cv::waitKey(-1);
						cv::destroyAllWindows();
						cv::imshow("xyz", xyz_image);
						cv::waitKey(-1);
						cv::destroyAllWindows();
					}
				// }

				pcl::PointCloud<pcl::PointXYZRGB> color_ptcloud;
				lucidManager->overlay_color_depth(color_image, xyz_image, color_ptcloud);
				pcl::io::savePCDFileASCII("/home/bot/JHY/Captured_Images/color_ptcloud_2.pcd", color_ptcloud);
			}

			lucidManager->stop();
		}
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