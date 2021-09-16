/***************************************************************************************
 ***                                                                                 ***
 ***  Copyright (c) 2021, Lucid Vision Labs, Inc.                                    ***
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
#include "ArenaCApi.h"
#include <stdbool.h>

// Callbacks: On Device Disconnected
//    This example demonstrates how to register a callback to get
//	  notified when a device has disconnected. At first this example
//    will enumerate devices then if there is any device fount it will
//    regsiter a disconnect callback for a discovered device.  
//	  Next the program will wait until a user inputs an exit 
//    command. While this example waits for input, feel free to 
//    disconnect the device. When the device is disconnected the callback 
//    will be triggered and it will print out info of the device 
//    that was removed by using OnDeviceDisconnected(acDevice hDevice) function.

// system timeout
#define SYSTEM_TIMEOUT 100

// maximum buffer length
#define MAX_BUF 256


// =-=-=-=-=-=-=-=-=-
// =-=- EXAMPLE -=-=-
// =-=-=-=-=-=-=-=-=-

// callback function
// (1) prints information from the callback
void OnDeviceDisconnected_PrintNodeValue(acDevice hDevice)
{
	// get node map
	acNodeMap hNodeMap = NULL;
	acDeviceGetTLDeviceNodeMap(hDevice, &hNodeMap);

	// get serial number
	char pDeviceSerialNumber[MAX_BUF];
	size_t pDeviceSerialNumberBufLen = MAX_BUF;

	acNodeMapGetStringValue(hNodeMap, "DeviceSerialNumber", pDeviceSerialNumber, &pDeviceSerialNumberBufLen);

	printf("Device with Serial: [ %s ] was disconnected.", pDeviceSerialNumber);

	printf("\nPress any key to continue\n");
}

// demonstrates usage of device disconnect callbacks
// (1) registers a disconnect callback
// (2) triggeres the callback
// (3) prints information from device using OnDeviceDisconnect(acDevice hDevice)
// (4) deregisters the callback
AC_ERROR RegisterOnDeviceDisconnect(acSystem hSystem, acDevice hDevice)
{
	AC_ERROR err = AC_ERR_SUCCESS;

	// register a disconnect callback
	acCallback hDeviceDisconnectCallback = NULL;
	err = acSystemRegisterDeviceDisconnectCallback(hSystem, hDevice, &hDeviceDisconnectCallback, OnDeviceDisconnected_PrintNodeValue);
	if (err != AC_ERR_SUCCESS)
		return err;

	printf("Waiting for user to disconnect a device or press enter to continue\n");
	getchar();

	bool isConnected;

	printf("Check if device is connected:\n");
	err = acDeviceIsConnected(hDevice, &isConnected);
	if (err != AC_ERR_SUCCESS)
		return err;

	if (isConnected == false)
	{
		printf("Device is disconnected\n");
	}
	else
	{
		printf("Device is connected\n");
	}

	// clean up - deregester an individual disconnect callback

	/*err = acSystemDeregisterDeviceDisconnectCallback(hSystem, hDeviceDisconnectCallback);
	if (err != AC_ERR_SUCCESS)
		return err;*/

	// clean up - deregester all disconnect callbacks

	err = acSystemDeregisterAllDeviceDisconnectCallbacks(hSystem);
	if (err != AC_ERR_SUCCESS)
		return err;

	return err;
}

// =-=-=-=-=-=-=-=-=-
// =- PREPARATION -=-
// =- & CLEAN UP =-=-
// =-=-=-=-=-=-=-=-=-

// error buffer length
#define ERR_BUF 512

#define CHECK_RETURN                                  \
	if (err != AC_ERR_SUCCESS)                        \
	{                                                 \
		char pMessageBuf[ERR_BUF];                    \
		size_t pBufLen = ERR_BUF;                     \
		acGetLastErrorMessage(pMessageBuf, &pBufLen); \
		printf("\nError: %s", pMessageBuf);           \
		printf("\n\nPress enter to complete\n");      \
		getchar();                                    \
		return -1;                                    \
	}

int main()
{
	printf("C_Callback_OnDeviceDisconnected\n");
	AC_ERROR err = AC_ERR_SUCCESS;

	// prepare example
	acSystem hSystem = NULL;
	err = acOpenSystem(&hSystem);
	CHECK_RETURN;
	err = acSystemUpdateDevices(hSystem, SYSTEM_TIMEOUT);
	CHECK_RETURN;
	size_t numDevices = 0;
	err = acSystemGetNumDevices(hSystem, &numDevices);
	CHECK_RETURN;
	if (numDevices == 0)
	{
		printf("\nNo camera connected\nPress enter to complete\n");
		getchar();
		return -1;
	}
	acDevice hDevice = NULL;
	err = acSystemCreateDevice(hSystem, 0, &hDevice);
	CHECK_RETURN;

	// run example
	printf("Commence example\n\n");
	err = RegisterOnDeviceDisconnect(hSystem, hDevice);
	CHECK_RETURN;
	printf("\nExample complete\n");

	// clean up example
	err = acSystemDestroyDevice(hSystem, hDevice);
	CHECK_RETURN;
	err = acCloseSystem(hSystem);
	CHECK_RETURN;

	printf("Press enter to complete\n");
	getchar();
	return -1;
}
