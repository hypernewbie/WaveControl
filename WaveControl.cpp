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

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cassert>
#include <cstdarg>

#include <memory>
#include <thread>
#include <mutex>
#include <chrono>

#include "WaveControl.h"

#define WAVECONTROL_WHEELSIZE_DATA_IMPLEMENTATION
#include "WaveControlWheelSizeList.h"

static std::function< void( const char* ) > s_WaveControl_LogCallbackFunctionGlobal = nullptr;

void WaveControlSetLogCallback( std::function< void( const char* ) > Callback )
{
	s_WaveControl_LogCallbackFunctionGlobal = Callback;
}

void WaveControlLog( const char* fmt, ... )
{
	using namespace std;

	static mutex WaveControlLogMutex;
	lock_guard<mutex> WaveControlLogLock( WaveControlLogMutex );

	const int WAVECONTROL_LOG_TEMPBUFFER_LEN = 8092;
	static char temp[WAVECONTROL_LOG_TEMPBUFFER_LEN];

	// Print to temporary location.
	va_list varListParser;
	va_start( varListParser, fmt );
	vsnprintf( temp, WAVECONTROL_LOG_TEMPBUFFER_LEN, fmt, varListParser );
	va_end( varListParser );

	if ( s_WaveControl_LogCallbackFunctionGlobal ) {
		s_WaveControl_LogCallbackFunctionGlobal( temp );
	} else {
		puts( temp );
	}
}

struct WaveControlPrivateData
{
	std::vector< WaveControl_WheelSizeData > WheelSizeData;
	WaveControl_WheelSizeData ChosenWheelSize;
};

static WaveControl_WheelSizeData FindWheelSizeData( WaveControlPrivateData& PrivateData, std::string Name )
{
	for ( auto& Data : PrivateData.WheelSizeData ) {
		if ( Data.FriendlyName == Name )
			return Data;
	}

	WaveControl_WheelSizeData Blank;
	return Blank;
}

// ---------------------------------------------------------------------- WaveControl::WorkerThread -----------------------------------------------------------------------------

std::unique_ptr< WaveDeviceBase > WaveControl::MakeDeviceForUsage( int Usage )
{
	assert( Usage >= 0 && Usage < WAVECONTROL_DEVICE_NUM );
	switch ( Usage )
	{
		case WAVECONTROL_DEVICE_HR:
			return std::make_unique< WaveHRMonitor >( &this->PeripheralTable );
		case WAVECONTROL_DEVICE_CADENCE:
		case WAVECONTROL_DEVICE_SPEED:
			return std::make_unique< WaveCadenceSpeedSensor >( &this->PeripheralTable );
		case WAVECONTROL_DEVICE_POWER:
			return std::make_unique< WavePowerSensor >( &this->PeripheralTable );
		case WAVECONTROL_DEVICE_TRAINER:
			return std::make_unique< WaveTrainerDevice >( &this->PeripheralTable );
		default:
			assert( !"Not implemented." );
			return nullptr;
	}
}

void WaveControl::WorkerThread_ScanStart()
{
	assert( std::this_thread::get_id() == this->WorkThreadID );

	if ( this->WorkThreadScanning )
		return;

	WAVECONTROL_LOG( "Starting scan...\n" );
	this->WorkThreadScanning = true;
	WaveBackend_ScanStart( this->Backend );
}

void WaveControl::WorkerThread_ScanStop()
{
	assert( std::this_thread::get_id() == this->WorkThreadID );

	if ( this->WorkThreadScanning ) {
		WAVECONTROL_LOG( "Stopping scan...\n" );
		WaveBackend_ScanStop( this->Backend );
		this->WorkThreadScanning = false;
	}
}

void WaveControl::WorkerThread_ChooseDeviceForUsage( int Usage, std::string UIAddress )
{
	assert( Usage >= 0 && Usage < WAVECONTROL_DEVICE_NUM );

	if ( !UIAddress.length() ) {
		ChosenDevices[Usage].reset( nullptr );
		return;
	}

	this->Backend.ScannedPeripheralsMutex.lock();
	for ( auto& PeripheralItr : this->Backend.ScannedPeripherals ) {
		auto& Peripheral = *PeripheralItr.second;
		if ( Peripheral.UIAddress == UIAddress ) {

			// Add the peripheral to current peripheral table.
			auto HandleID = this->PeripheralTable.AddPeripheral();
			this->PeripheralTable.SetPeripheral( HandleID, PeripheralItr.second );
			this->PeripheralTable.IncrementPeripheralConnectionRef( HandleID );
			assert( PeripheralItr.second->RefCount == 1 );

			// Select the chosen device.
			ChosenDevices[Usage].reset( nullptr );
			ChosenDevices[Usage] = this->MakeDeviceForUsage( Usage );
			ChosenDevices[Usage]->PeripheralTable = &this->PeripheralTable;
			ChosenDevices[Usage]->PeripheralHandleID = HandleID;
			ChosenDevices[Usage]->ReadState = this->SensorReadState;
			ChosenDevices[Usage]->WriteState = this->SensorWriteState;
			ChosenDevices[Usage]->Enabled = true;

			break;
		}
	}
	this->Backend.ScannedPeripheralsMutex.unlock();

	return;
}

void WaveControl::WorkerThread_Update()
{
	assert( std::this_thread::get_id() == this->WorkThreadID );

	// Auto stop scanning at some point.
	if ( this->WorkThreadScanning && this->WorkThreadScanningAutostopFrames != INT_MAX ) {
		if ( this->WorkThreadScanningAutostopFrames-- <= 0 ) {
			this->WorkThreadScanningAutostopFrames = INT_MAX;
			this->WorkerThread_ScanStop();
		}
	}

	// Propagate sensor metainformation.
	auto SpeedSensor = dynamic_cast< WaveCadenceSpeedSensor* >( ChosenDevices[ WAVECONTROL_DEVICE_SPEED ].get() );
	if ( SpeedSensor ) {
		SpeedSensor->SetWheelSizeData( this->PrivateData->ChosenWheelSize );
	}
	auto CadenceSensor = dynamic_cast< WaveCadenceSpeedSensor* >( ChosenDevices[ WAVECONTROL_DEVICE_CADENCE ].get() );
	if ( CadenceSensor ) {
		CadenceSensor->SetWheelSizeData( this->PrivateData->ChosenWheelSize );
	}

	// Step sensors
	for ( auto& ChosenDevice : ChosenDevices ) {
		if ( !ChosenDevice.get() || !ChosenDevice->Enabled )
			continue;
		ChosenDevice->Update();
	}

	// Step peripheral table.
	this->PeripheralTable.Update();
}

void WaveControl::WorkThread_Entry()
{
	WAVECONTROL_LOG( "Workthread starting!\n" );
	WorkThreadID = std::this_thread::get_id();
	WaveBackend_Init( this->Backend );

	while ( !this->WorkerThreadExit ) {
		this->WorkThreadQueueMutex.lock();
		auto CallbackWorkQueue = this->WorkThreadQueue;
		this->WorkThreadQueue.clear();
		this->WorkThreadQueueMutex.unlock();

		for ( auto& Func : CallbackWorkQueue ) {
			Func();
		}

		WorkerThread_Update();
		std::this_thread::sleep_for( std::chrono::milliseconds( 32 ) );
	}
}

void WaveControl::WorkThread_Do( std::function< void() > Func )
{
	assert( Func );
	assert( std::this_thread::get_id() != this->WorkThreadID );
	this->WorkThreadQueueMutex.lock();
	this->WorkThreadQueue.push_back( Func );
	this->WorkThreadQueueMutex.unlock();
}

// ---------------------------------------------------------------------- WaveControl -----------------------------------------------------------------------------

WaveControl::WaveControl()
{
	this->PrivateData = std::make_unique< WaveControlPrivateData >();
	this->SensorReadState = std::make_shared< WaveCycleSensorReadState >();
	this->SensorWriteState = std::make_shared< WaveCycleSensorWriteState >();

	WaveControl_InitialiseWheelSizes( this->PrivateData->WheelSizeData );

	// Start the worker thread.
	this->WorkThread = std::make_unique< std::thread >( &WaveControl::WorkThread_Entry, this );

	// Wait for backend to init from worker thread.
	volatile bool BackendInitialised = false;
	WorkThread_Do( [&BackendInitialised]() { BackendInitialised = true; } );
	while ( !BackendInitialised ) {
		std::this_thread::sleep_for( std::chrono::milliseconds( 5 ) );
	}
}

WaveControl::~WaveControl()
{
	this->WorkerThreadExit = true;
	this->WorkThread->join();
	this->WorkThread.reset( nullptr );
	
	this->SensorReadState = nullptr;
	this->SensorWriteState = nullptr;

	WaveBackend_Shutdown( this->Backend );
}

void WaveControl::ScanStart( int AutoStopFrames )
{
	WorkThread_Do( [this, AutoStopFrames](){ this->WorkThreadScanningAutostopFrames = AutoStopFrames; } );
	WorkThread_Do( std::bind( &WaveControl::WorkerThread_ScanStart, this ) );
}

void WaveControl::ScanStop()
{
	WorkThread_Do( std::bind( &WaveControl::WorkerThread_ScanStop, this ) );
}

std::vector< std::pair< std::string, std::string > > WaveControl::ListPeripherals()
{
	std::vector< std::pair< std::string, std::string > > Peripherals;
	this->Backend.ScannedPeripheralsMutex.lock();
	for ( auto& PeripheralItr : this->Backend.ScannedPeripherals ) {
		auto& Peripheral = *PeripheralItr.second;
		Peripherals.push_back( std::make_pair( Peripheral.UIName, Peripheral.UIAddress ) );
	}
	this->Backend.ScannedPeripheralsMutex.unlock();
	return Peripherals;
}

void WaveControl::ChooseDeviceForUsage( int Usage, std::string UIAddress )
{
	WorkThread_Do( std::bind( &WaveControl::WorkerThread_ChooseDeviceForUsage, this, Usage, UIAddress ) );
}

int WaveControl::DoesDeviceAdvertiseUsage( int Usage, std::string UIAddress, std::string ServiceID )
{
	this->Backend.ScannedPeripheralsMutex.lock();
	
	for ( auto& PeripheralItr : this->Backend.ScannedPeripherals ) {
		auto& Peripheral = *PeripheralItr.second;
		
		if ( Peripheral.UIAddress == UIAddress ) {
			
			for ( auto& Service: Peripheral.Services ) {
				if ( Service == ServiceID ) {
					return 1;
				}
			}
			return 0;
		
		}
	}
	
	this->Backend.ScannedPeripheralsMutex.unlock();
	return -1;
}

void WaveControl::ChooseWheelSize( std::string Name )
{
	this->PrivateData->ChosenWheelSize = FindWheelSizeData( *this->PrivateData, Name );
}

std::shared_ptr< WaveCycleSensorReadState > WaveControl::GetSensorReadState()
{
	return this->SensorReadState;
}
std::shared_ptr< WaveCycleSensorWriteState > WaveControl::GetSensorWriteState()
{
	return this->SensorWriteState;
}