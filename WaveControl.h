/*
	Copyright 2021 Xi Chen (hypernewbie@gmail.com)

	Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
	associated documentation files (the "Software"), to deal in the Software without restriction,
	including without limitation the rights to use, copy, modify, merge, publish, distribute,
	sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all copies or substantial
	portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
	NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
	NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
	DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT
	OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

#include <cstdint>
#include <cstdio>

#include <thread>
#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <functional>

#include "WaveBackend.h"
#include "WaveDevice.h"

#define WAVECONTROL_LOG printf
#define WAVECONTROL_DEFAULT_BLUETOOTH_ADAPTER 0

struct WaveControlPrivateData;

void WaveControlSetLogCallback( std::function< void( const char* ) > Callback );

class WaveControl
{
	WaveBluetoothBackend Backend;

	WavePeripheralTable PeripheralTable;
	std::unique_ptr< WaveControlPrivateData > PrivateData;
	std::unique_ptr< WaveDeviceBase > ChosenDevices[ WAVECONTROL_DEVICE_NUM ];
	std::shared_ptr< WaveCycleSensorReadState > SensorReadState;
	std::shared_ptr< WaveCycleSensorWriteState > SensorWriteState;

	// Worker thread handling.
	std::unique_ptr< std::thread > WorkThread;
	std::thread::id WorkThreadID;
	std::vector< std::function< void() > > WorkThreadQueue;
	std::mutex WorkThreadQueueMutex;
	bool WorkerThreadExit = false;
	bool WorkThreadScanning = false;
	int WorkThreadScanningAutostopFrames = INT_MAX;

protected:
	std::unique_ptr< WaveDeviceBase > MakeDeviceForUsage( int Usage );

	void WorkerThread_ScanStart();

	void WorkerThread_ScanStop();

	void WorkerThread_ChooseDeviceForUsage( int Usage, std::string UIAddress );

	void WorkerThread_Update();

	void WorkThread_Entry();

	void WorkThread_Do( std::function< void() > Func );

public:
	WaveControl();
	virtual ~WaveControl();

	void ScanStart( int AutoStopFrames = INT_MAX );

	void ScanStop();

	std::vector< std::pair< std::string, std::string > > ListPeripherals();

	void ChooseDeviceForUsage( int Usage, std::string UIAddress );

	// 0 = no, 1 , yes, -1 = device not found.
	int DoesDeviceAdvertiseUsage( int Usage, std::string UIAddress, std::string ServiceID );

	void ChooseWheelSize( std::string Name );

	std::shared_ptr< WaveCycleSensorReadState > GetSensorReadState();

	std::shared_ptr< WaveCycleSensorWriteState > GetSensorWriteState();
};


