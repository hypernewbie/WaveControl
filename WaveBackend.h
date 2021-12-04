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
	NONINFRINGEMENT. IyN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
	DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT
	OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

#include <cstdint>
#include <vector>
#include <string> 
#include <map> 

// Default to first found BLE adapter.
#define WAVECONTROL_BACKEND_DEFAULT_ADAPTER_INDEX 0

struct Wave_PeripheralHandle;
struct Wave_AdapterHandle;

struct WavePeripheral
{
	std::string UIName;
	std::string UIAddress;

	std::shared_ptr< Wave_PeripheralHandle > Handle;
	std::vector< std::string > Services;

	int64_t RefCount = 0;
	std::map< std::string, bool> ServicesConnected;
};

bool WavePeripheral_IsConnected( WavePeripheral& Peripheral );

bool WavePeripheral_IsConnectable( WavePeripheral& Peripheral );

void WavePeripheral_Connect( WavePeripheral& Peripheral );

void WavePeripheral_Disconnect( WavePeripheral& Peripheral );

bool WavePeripheral_HasService( WavePeripheral& Peripheral, std::string Service );

bool WavePeripheral_HasCharacteristic( WavePeripheral& Peripheral, std::string Service, std::string Characteristic );

void WavePeripheral_Notify( WavePeripheral& Peripheral, std::string Service, std::string Characteristic, std::function< void( std::string payload ) > Callback );

void WavePeripheral_Indicate( WavePeripheral& Peripheral, std::string Service, std::string Characteristic, std::function< void( std::string payload ) > Callback );

void WavePeripheral_WriteCommand( WavePeripheral& Peripheral, std::string Service, std::string Characteristic, std::string Payload );

struct WaveBluetoothBackend
{
	std::shared_ptr< Wave_AdapterHandle > Adapter;
	std::map< std::string, std::shared_ptr< WavePeripheral > > ScannedPeripherals;
	std::mutex ScannedPeripheralsMutex;
};

void WaveBackend_Init( WaveBluetoothBackend& Backend, int AdapterIndex = WAVECONTROL_BACKEND_DEFAULT_ADAPTER_INDEX );

void WaveBackend_Shutdown( WaveBluetoothBackend& Backend );

void WaveBackend_ScanStart( WaveBluetoothBackend& Backend );

void WaveBackend_ScanStop( WaveBluetoothBackend& Backend );



