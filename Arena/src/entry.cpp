#include "include/dr_lucid_manager.hh"

int main()
{
  bool exceptionThrown = false;

  try
  {
    LucidManager *lucidManager = new LucidManager();
    Lucid *helios = lucidManager->CreateDevice("1c:0f:af:00:46:6f", "Coord3D_ABCY16");
		Lucid *phoneix = lucidManager->CreateDevice("1c:0f:af:0c:85:61", "BGR8");
    helios->ConfigureCamera();
    helios->StartStream();
		phoneix->ConfigureCamera();
		phoneix->StartStream();

		// while (true)
		// {
		std::cout << "\nPress enter to get next image\n";
		std::getchar();
		helios->TriggerArming();
		phoneix->TriggerArming();

		helios->GetAndSaveImage();
		phoneix->GetAndSaveImage();
		// }

    helios->StopStream();
		phoneix->StopStream();
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
