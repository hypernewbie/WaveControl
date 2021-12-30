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

#include <cassert>

#include "WaveDevice.h"
#include "WaveControl.h"
#include "WaveBackend.h"
#include "WaveControlCompanyIDList.h"

#define WAVECONTROL_BACKEND_SIMPLEBLE 1

#if WAVECONTROL_BACKEND_SIMPLEBLE
	#include <simpleble/SimpleBLE.h>
#else
	#error "Unknown backend."
#endif // WAVECONTROL_BACKEND_SIMPLEBLE

#if WAVECONTROL_BACKEND_SIMPLEBLE

struct Wave_PeripheralHandle
{
	SimpleBLE::Peripheral Internal;
};

struct Wave_AdapterHandle
{
	SimpleBLE::Adapter Internal;
};

#endif // WAVECONTROL_BACKEND_SIMPLEBLE

static std::map< int, std::string > s_WaveDevice_BluetoothCompanyIDs;

bool WavePeripheral_IsConnected( WavePeripheral& Peripheral )
{
	if ( !Peripheral.Handle.get() )
		return false;

#if WAVECONTROL_BACKEND_SIMPLEBLE
	SimpleBLE::Peripheral* PeripheralInteral = &Peripheral.Handle->Internal;
	return PeripheralInteral->is_connected();
#endif // WAVECONTROL_BACKEND_SIMPLEBLE
}

bool WavePeripheral_IsConnectable( WavePeripheral& Peripheral )
{
	if ( !Peripheral.Handle.get() )
		return false;

#if WAVECONTROL_BACKEND_SIMPLEBLE
	SimpleBLE::Peripheral* PeripheralInteral = &Peripheral.Handle->Internal;
	return PeripheralInteral->is_connectable();
#endif // WAVECONTROL_BACKEND_SIMPLEBLE
}

void WavePeripheral_Connect( WavePeripheral& Peripheral )
{
	if ( !Peripheral.Handle.get() )
		return;

#if WAVECONTROL_BACKEND_SIMPLEBLE
	SimpleBLE::Peripheral* PeripheralInteral = &Peripheral.Handle->Internal;
	try {
		PeripheralInteral->connect();
	} catch( ... ) {
		WAVECONTROL_LOG( "WavePeripheral_Connect: Call failed!\n");
	}
#endif // WAVECONTROL_BACKEND_SIMPLEBLE
}

void WavePeripheral_Disconnect( WavePeripheral& Peripheral )
{
	if ( !Peripheral.Handle.get() )
		return;

#if WAVECONTROL_BACKEND_SIMPLEBLE
	SimpleBLE::Peripheral* PeripheralInteral = &Peripheral.Handle->Internal;
	try {
		PeripheralInteral->disconnect();
	} catch( ... ) {
		WAVECONTROL_LOG( "WavePeripheral_Disconnect: Call failed!\n");
	}
#endif // WAVECONTROL_BACKEND_SIMPLEBLE
}

bool WavePeripheral_HasService( WavePeripheral& Peripheral, std::string Service )
{
	if ( !Peripheral.Handle.get() )
		return false;

#if WAVECONTROL_BACKEND_SIMPLEBLE
	SimpleBLE::Peripheral* PeripheralInteral = &Peripheral.Handle->Internal;
	for ( auto& PeripheralService : PeripheralInteral->services() ) {
		if ( PeripheralService.uuid == Service )
			return true;
	}
	return false;
#endif // WAVECONTROL_BACKEND_SIMPLEBLE
}

bool WavePeripheral_HasCharacteristic( WavePeripheral& Peripheral, std::string Service, std::string Characteristic )
{
	if ( !Peripheral.Handle.get() )
		return false;

#if WAVECONTROL_BACKEND_SIMPLEBLE
	SimpleBLE::Peripheral* PeripheralInteral = &Peripheral.Handle->Internal;
	for ( auto& PeripheralService : PeripheralInteral->services() ) {
		if ( PeripheralService.uuid == Service ) {
			for ( auto& PeripheralCharacteristic : PeripheralService.characteristics ) {
				if ( PeripheralCharacteristic == Characteristic )
					return true;
			}
		}
	}
	return false;
#endif // WAVECONTROL_BACKEND_SIMPLEBLE
}

void WavePeripheral_Notify( WavePeripheral& Peripheral, std::string Service, std::string Characteristic, std::function< void( std::string payload ) > Callback )
{
	if ( !Peripheral.Handle.get() )
		return;

#if WAVECONTROL_BACKEND_SIMPLEBLE
	try {
		SimpleBLE::Peripheral* PeripheralInteral = &Peripheral.Handle->Internal;
		PeripheralInteral->notify( Service, Characteristic, Callback );
	} catch( ... ) {
		// WAVECONTROL_LOG( "WavePeripheral_Notify: Call failed!\n");
	}
#endif // WAVECONTROL_BACKEND_SIMPLEBLE
}

void WavePeripheral_Indicate( WavePeripheral& Peripheral, std::string Service, std::string Characteristic, std::function< void( std::string payload ) > Callback )
{
	if ( !Peripheral.Handle.get() )
		return;

#if WAVECONTROL_BACKEND_SIMPLEBLE
	try {
		SimpleBLE::Peripheral* PeripheralInteral = &Peripheral.Handle->Internal;
		PeripheralInteral->indicate( Service, Characteristic, Callback );
	} catch( ... ) {
		// WAVECONTROL_LOG( "WavePeripheral_Notify: Call failed!\n");
	}
#endif // WAVECONTROL_BACKEND_SIMPLEBLE
}

void WavePeripheral_WriteCommand( WavePeripheral& Peripheral, std::string Service, std::string Characteristic, std::string Payload )
{
	if ( !Peripheral.Handle.get() )
		return;

#if WAVECONTROL_BACKEND_SIMPLEBLE
	try {
		SimpleBLE::Peripheral* PeripheralInteral = &Peripheral.Handle->Internal;
		PeripheralInteral->write_command( Service, Characteristic, Payload );
	} catch( ... ) {
		// WAVECONTROL_LOG( "WavePeripheral_WriteCommand: Call failed!\n");
	}
#endif // WAVECONTROL_BACKEND_SIMPLEBLE
}


// -----------------------------------------------------------  WaveBackend ---------------------------------------------

void WaveBackend_Init( WaveBluetoothBackend& Backend, int AdapterIndex )
{
	// Initialise adapters.
#if WAVECONTROL_BACKEND_SIMPLEBLE
	auto AdapterList = SimpleBLE::Adapter::get_adapters();
	if ( !AdapterList.size() ) {
		WAVECONTROL_LOG( "WaveBackend_Init: Failed to find any Bluetooth adapters. Does your computer support Bluetooth?\n" );
		return;
	}
	assert( AdapterIndex < AdapterList.size() );
	Backend.Adapter = std::make_shared< Wave_AdapterHandle >();
	Backend.Adapter->Internal = AdapterList[ AdapterIndex ];
#endif // WAVECONTROL_BACKEND_SIMPLEBLE
}

void WaveBackend_Shutdown( WaveBluetoothBackend& Backend )
{
#if WAVECONTROL_BACKEND_SIMPLEBLE
	Backend.ScannedPeripheralsMutex.lock();
	for( auto& Peripheral : Backend.ScannedPeripherals ) {
		SimpleBLE::Peripheral* PeripheralInteral = &Peripheral.second->Handle->Internal;
		if ( PeripheralInteral && PeripheralInteral->is_connected() ) {
			WAVECONTROL_LOG( "Disconnecting from %s ...\n", Peripheral.second->UIName.c_str());
			try {
				PeripheralInteral->disconnect();
			} catch( ... ) {}
		}
	}
	Backend.ScannedPeripheralsMutex.unlock();
#endif // WAVECONTROL_BACKEND_SIMPLEBLE
}

void WaveBackend_ScanStart( WaveBluetoothBackend& Backend )
{
	if ( !Backend.Adapter.get() )
		return;

	if ( !s_WaveDevice_BluetoothCompanyIDs.size() ) {
		WaveControl_InitialiseCompanyIDs( s_WaveDevice_BluetoothCompanyIDs );
		assert( s_WaveDevice_BluetoothCompanyIDs.size() > 0 );
	}

#if WAVECONTROL_BACKEND_SIMPLEBLE
	SimpleBLE::Adapter* AdapterInternal = &Backend.Adapter->Internal;
	AdapterInternal->set_callback_on_scan_found(
		[&]( SimpleBLE::Peripheral PeripheralData )
		{
			if ( !PeripheralData.is_connectable() )
				return;

			auto Peripheral = std::make_shared< WavePeripheral >();
			Peripheral->Handle = std::make_shared< Wave_PeripheralHandle >();
			Peripheral->Handle->Internal = PeripheralData;
			Peripheral->UIName = PeripheralData.identifier();
			Peripheral->UIAddress = PeripheralData.address().c_str();

			// Create a friendly name in case local name doesn't work.
			if ( Peripheral->UIName.length() <= 0 ) {
				for ( auto& ManufacturerData : PeripheralData.manufacturer_data() ) {
					auto FriendlyCompanyName = s_WaveDevice_BluetoothCompanyIDs.find( ManufacturerData.first );
					if ( FriendlyCompanyName != s_WaveDevice_BluetoothCompanyIDs.end() ) {
						Peripheral->UIName = FriendlyCompanyName->second;
					} else {
						char HEXName[64]; HEXName[0] = '\0';
						sprintf_s(HEXName, "%d", ( uint32_t ) ManufacturerData.first );
						Peripheral->UIName = HEXName;
					}
				}
			}
			if ( Peripheral->UIName.length() <= 0 ) {
				Peripheral->UIName = "Unknown";
			}
			WAVECONTROL_LOG("    FOUND %s [%s]\n", Peripheral->UIName.c_str(), Peripheral->UIAddress.c_str());

			// Read services.
			for ( auto ServiceUUID : PeripheralData.advertised_services() ) {
				Peripheral->Services.push_back( ServiceUUID );
			}

			// Add Peripheral
			Backend.ScannedPeripheralsMutex.lock();
			if ( Backend.ScannedPeripherals.find( Peripheral->UIAddress ) == Backend.ScannedPeripherals.end() ) {
				Backend.ScannedPeripherals[ Peripheral->UIAddress ] = Peripheral;
			}
			Backend.ScannedPeripheralsMutex.unlock();
		}
	);
	try {
		AdapterInternal->scan_start();
	} catch ( ... ) {
		WAVECONTROL_LOG( "WaveBackend_ScanStart: Call failed. Is the Bluetooth adapter enabled?\n" );
	}
#endif // WAVECONTROL_BACKEND_SIMPLEBLE
}

void WaveBackend_ScanStop( WaveBluetoothBackend& Backend )
{
	if ( !Backend.Adapter.get() )
		return;

#if WAVECONTROL_BACKEND_SIMPLEBLE
	SimpleBLE::Adapter* AdapterInternal = &Backend.Adapter->Internal;
	try {
		AdapterInternal->scan_stop();
	} catch ( ... ) {
		WAVECONTROL_LOG( "WaveBackend_ScanStop: Call failed. Is the Bluetooth adapter enabled?\n" );
	}
#endif // WAVECONTROL_BACKEND_SIMPLEBLE
}
