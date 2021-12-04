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
#include <iostream>
#include <iomanip> // For std::setfill

#include "WaveDevice.h"
#include "WaveControl.h"
#include "WaveBackend.h"

#include "WaveControlWheelSizeList.h"

// ---------------------------------------------------------- WavePeripheralTable ----------------------------------------------------

WavePeripheralTable::WavePeripheralTable()
{
	PeripheralHashtable.reserve( 64 );
}

WavePeripheralTable::~WavePeripheralTable()
{
	for ( auto itr : PeripheralHashtable ) {
		this->RemovePeripheral( itr.first );
	}
	PeripheralHashtable.clear();
}

std::shared_ptr< WavePeripheral > WavePeripheralTable::GetPeripheral( uint64_t HandleID )
{
	return PeripheralHashtable[HandleID];
}

void WavePeripheralTable::SetPeripheral( uint64_t HandleID, std::shared_ptr< WavePeripheral > Value )
{
	PeripheralHashtable[HandleID] = Value;
}

uint64_t WavePeripheralTable::AddPeripheral()
{
	return NextPeripheralHandleID++;
}

void WavePeripheralTable::RemovePeripheral( uint64_t HandleID )
{
	if ( PeripheralHashtable[HandleID].get() ) {
		if ( WavePeripheral_IsConnected( *PeripheralHashtable[HandleID] ) ) {
			WavePeripheral_Disconnect( *PeripheralHashtable[HandleID] );
		}
		PeripheralHashtable[HandleID] = nullptr;
	}
}

void WavePeripheralTable::IncrementPeripheralConnectionRef( uint64_t HandleID )
{
	if ( PeripheralHashtable.find( HandleID ) == PeripheralHashtable.end() )
		return;
	
	auto Data = PeripheralHashtable[HandleID].get();
	if ( !Data || !WavePeripheral_IsConnectable( *Data ) )
		return;

	if ( WavePeripheral_IsConnected( *Data ) ) {
		// This is a shared peripheral. Examples of shared peripheral are speed + cadence,
		// or power + trainer control. We have an existing connection so this means we should alreadu have a connection reference.
		assert( Data->RefCount > 0 );
		Data->RefCount++;
		return;
	}

	if ( !WavePeripheral_IsConnected( *Data ) ) {
		// Create connection to this peripheral.
		// We should have refcount = 0 because we only create the connection on first reference and no other time.
		assert( Data->RefCount == 0 );
		Data->ServicesConnected.clear();
		
		WAVECONTROL_LOG( "Connecting to %s ...\n", Data->UIName.c_str());
		WavePeripheral_Connect( *Data );

		if ( !WavePeripheral_IsConnected( *Data ) ) {
			// Failed initial connection. This could happen if user unplugs power right after we select this device.
			// No issue, Update() function below will keep on retrying over and over again.
			WAVECONTROL_LOG( "Connection to peripheral %s FAILED!!!\n", Data->UIName.c_str());
		}
		WAVECONTROL_LOG( "Connected to %s !\n", Data->UIName.c_str());
	}

	Data->RefCount++;
}

void WavePeripheralTable::DecrementPeripheralConnectionRef( uint64_t HandleID )
{
	if ( PeripheralHashtable.find( HandleID ) == PeripheralHashtable.end() )
		return;

	auto Data = PeripheralHashtable[HandleID].get();
	if ( !Data )
		return;

	assert( Data->RefCount > 0 );
	Data->RefCount--;

	// Note that we don't disconnect here immediately, lest the user quickly switches
	// from the same device to the same device. We're in no good rush to disconnect immediately anyway,
	// so just get Update() take care of it.
}

void WavePeripheralTable::Update()
{
	for ( auto itr : PeripheralHashtable ) {
		auto Data = itr.second.get();
		if ( !Data || !WavePeripheral_IsConnectable( *Data ) )
			continue;

		if ( Data->RefCount <= 0 ) {
			if ( WavePeripheral_IsConnected( *Data ) ) {
				// Disconnect on reaching refcount 0.
				WavePeripheral_Disconnect( *Data );
				assert( !WavePeripheral_IsConnected( *Data ) );
			}
			Data->RefCount = 0;
		}

		if ( !WavePeripheral_IsConnected( *Data ) ) {
			Data->ServicesConnected.clear();
		}

		if ( Data->RefCount > 0 && !WavePeripheral_IsConnected( *Data ) ) {
			// We are asking for device connection, but device is disconnected. This means we lost connection to the BT device.
			// This could either be poor connection or user action (eg. user turned off power or unplugged device).
			// Simply attempt to reconnect.
			WAVECONTROL_LOG( "Lost connection to %s. Attempting to reconnect...\n", Data->UIName.c_str());
			WavePeripheral_Connect( *Data );
			Data->ServicesConnected.clear();
		}
	}
}

bool WavePeripheralTable::IsServiceConnected( uint64_t HandleID, const std::string& ServiceUUID )
{
	if ( PeripheralHashtable.find( HandleID ) == PeripheralHashtable.end() )
		return false;

	auto Data = PeripheralHashtable[HandleID].get();
	if ( !Data )
		return false;

	if ( Data->ServicesConnected.find( ServiceUUID ) == Data->ServicesConnected.end() )
		return false;

	return Data->ServicesConnected[ ServiceUUID ];
}

void WavePeripheralTable::MarkServiceConnected( uint64_t HandleID, const std::string& ServiceUUID, bool Connected )
{
	if ( PeripheralHashtable.find( HandleID ) == PeripheralHashtable.end() )
		return;

	auto Data = PeripheralHashtable[HandleID].get();
	if ( !Data )
		return;

	Data->ServicesConnected[ ServiceUUID ] = Connected;
}

// ------------------------------------------------ WaveDevice -------------------------------------------------

WaveDeviceBase::WaveDeviceBase( WavePeripheralTable* PTable )
	: PeripheralTable( PTable )
{
	assert( PTable );
}

WaveDeviceBase::~WaveDeviceBase()
{
	if ( !this->NeedToReferencePeripheral ) {
		assert( this->PeripheralTable );
		this->PeripheralTable->DecrementPeripheralConnectionRef( this->PeripheralHandleID );
		this->NeedToReferencePeripheral = true;
	}
}


std::shared_ptr< WavePeripheral > WaveDeviceBase::GetPeripheralFromTableInternal()
{
	assert( this->PeripheralTable );
	return this->PeripheralTable->GetPeripheral( this->PeripheralHandleID );
}

void WaveDeviceBase::Update()
{
	if ( this->NeedToReferencePeripheral ) {
		assert( this->PeripheralTable );
		this->PeripheralTable->IncrementPeripheralConnectionRef( this->PeripheralHandleID );
		this->NeedToReferencePeripheral = false;
	}
}

WaveHRMonitor::WaveHRMonitor( WavePeripheralTable* PTable )
	: WaveDeviceBase( PTable )
{
}

void WaveHRMonitor::Update()
{
	WaveDeviceBase::Update();

	auto Data = this->GetPeripheralFromTableInternal();
	if ( !Data || !WavePeripheral_IsConnectable( *Data ) )
		return;
	if ( !WavePeripheral_IsConnected( *Data ) )
		return;

	static const char* HRM_ServiceUUID = "0000180d-0000-1000-8000-00805f9b34fb";
	static const char* HRM_CharUUID = "00002a37-0000-1000-8000-00805f9b34fb";

	bool NeedSetup = !this->PeripheralTable->IsServiceConnected( this->PeripheralHandleID, HRM_ServiceUUID );
	if ( NeedSetup ) {
		WavePeripheral_Notify(
			*Data,
			HRM_ServiceUUID, HRM_CharUUID,
			[&]( std::string Bytes )
			{
				if ( Bytes.size() <= 0)
					return;

				uint8_t Flags = Bytes[0];

				static const uint8_t HRM_uint16_measurement_mask = 0x01;
				static const uint8_t HRM_contact_detected_mask = 0x06;
				static const uint8_t HRM_energy_expended_present_mask = 0x08;
				static const uint8_t HRM_rr_interval_present_mask = 0x10;

				ReadState->HR_SensorContact = ( Flags & HRM_contact_detected_mask ) ? true : false;

				if ( Flags & HRM_uint16_measurement_mask ) {
					assert( Bytes.size() >= 3 );
					ReadState->HR_BPM = ( reinterpret_cast< uint16_t* >( &Bytes[1] ) )[0];
				} else {
					assert( Bytes.size() >= 2 );
					ReadState->HR_BPM = Bytes[1];
				}

				// WAVECONTROL_LOG( "[HRM] %d BPM, %s\n", State->HR_BPM, State->HR_SensorContact ? "Contact" : "No contact" );
			}
		);
		this->PeripheralTable->MarkServiceConnected( this->PeripheralHandleID, HRM_ServiceUUID );
	}
}

WaveCadenceSpeedSensor::WaveCadenceSpeedSensor( WavePeripheralTable* PTable )
	: WaveDeviceBase( PTable )
{
	WheelSize = std::make_unique< WaveControl_WheelSizeData > ();
}

void WaveCadenceSpeedSensor::UpdateCSCFromData()
{
	uint32_t WheelRevsDelta = WheelRevs - Prev_WheelRevs;
	uint32_t LastWheelTimeDelta = LastWheelTime - Prev_LastWheelTime;
	uint32_t CrankRevsDelta = CrankRevs - Prev_CrankRevs;
	uint32_t LastCrankTimeDelta = LastCrankTime - Prev_LastCrankTime;

	if ( WheelSize->WheelCircumferenceMM > 0.0f && LastWheelTimeDelta != 0 ) {
		float LastWheelTimeSeconds = ( float ) LastWheelTimeDelta / 1024.0f;
		float DistanceTravelledMM = WheelRevsDelta * WheelSize->WheelCircumferenceMM;
		float DistanceTravelledM = DistanceTravelledMM / 1000.0f;
		if ( LastWheelTimeSeconds > 0.000001f ) {
			float Speed = DistanceTravelledM / LastWheelTimeSeconds;
			Speed *= ( 3600.0f / 1000.0f );
			ReadState->Speed = Speed;
		}
	}

	if ( LastCrankTimeDelta != 0 ) {
		float LastCrankTimeSeconds =( float ) LastCrankTimeDelta / 1024.0f;
		if ( LastCrankTimeSeconds > 0.000001f ) {
			float Cadence = ( ( float ) CrankRevsDelta * 60.0f ) / LastCrankTimeSeconds;
			ReadState->Cadence = Cadence;
		}
	}

	Prev_WheelRevs = WheelRevs;
	Prev_LastWheelTime = LastWheelTime;
	Prev_CrankRevs = CrankRevs;
	Prev_LastCrankTime = LastCrankTime;
}


void WaveCadenceSpeedSensor::SetWheelSizeData( WaveControl_WheelSizeData& WheelSizeData )
{
	WheelSize.get()[0] = WheelSizeData;
}

void WaveCadenceSpeedSensor::Update()
{
	WaveDeviceBase::Update();

	auto Data = this->GetPeripheralFromTableInternal();
	if ( !Data || !WavePeripheral_IsConnectable( *Data ) )
		return;
	if ( !WavePeripheral_IsConnected( *Data ) )
		return;

	static const char* CSC_ServiceUUID = "00001816-0000-1000-8000-00805f9b34fb";
	static const char* CSC_MeasurementUUID = "00002a5b-0000-1000-8000-00805f9b34fb";
	
	bool NeedSetup = !this->PeripheralTable->IsServiceConnected( this->PeripheralHandleID, CSC_ServiceUUID );
	if ( NeedSetup ) {
		WavePeripheral_Notify(
			*Data,
			CSC_ServiceUUID,
			CSC_MeasurementUUID,
			[&]( std::string Bytes )
			{
				if ( Bytes.size() <= 0 )
					return;

				uint8_t Flags = Bytes[0];

				static const uint8_t CSC_wheel_rev_mask = 1;
				static const uint8_t CSC_crank_rev_mask = 2;

				int Offset = 1;

				if ( Flags & CSC_wheel_rev_mask ) {
					assert( Bytes.size() >= Offset + 6 );
					WheelRevs = ( reinterpret_cast< uint32_t* >( &Bytes[Offset] ) )[0];
					LastWheelTime = ( reinterpret_cast< uint16_t* >( &Bytes[Offset + 4] ) )[0];
					Offset += 6;
				}

				if ( Flags & CSC_crank_rev_mask ) {
					assert( Bytes.size() >= Offset + 4 );
					CrankRevs = ( reinterpret_cast< uint16_t* >( &Bytes[Offset] ) )[0];
					LastCrankTime = ( reinterpret_cast< uint16_t* >( &Bytes[Offset + 2] ) )[0];
				}

				this->UpdateCSCFromData();

				//WAVECONTROL_LOG( "Speed %.2f Km/Hr Cadence %.2f RPM\n", Speed, Cadence );
				//WAVECONTROL_LOG( "WheelRevs %d LastWheelTime %d CrankRevs %d LastCrankTime %d\n", WheelRevs, LastWheelTime, CrankRevs, LastCrankTime );
			}
		);
		this->PeripheralTable->MarkServiceConnected( this->PeripheralHandleID, CSC_ServiceUUID );
	}
}  

WavePowerSensor::WavePowerSensor( WavePeripheralTable* PTable )
	: WaveDeviceBase( PTable )
{
}

void WavePowerSensor::Update()
{
	WaveDeviceBase::Update();
	
	auto Data = this->GetPeripheralFromTableInternal();
	if ( !Data || !WavePeripheral_IsConnectable( *Data ) )
		return;
	if ( !WavePeripheral_IsConnected( *Data ) )
		return;

	static const char* PS_ServiceUUID = "00001818-0000-1000-8000-00805f9b34fb";
	static const char* PS_MeasurementUUID = "00002a63-0000-1000-8000-00805f9b34fb";

	bool NeedSetup = !this->PeripheralTable->IsServiceConnected( this->PeripheralHandleID, std::string( PS_ServiceUUID ) + " " + PS_MeasurementUUID );
	if ( NeedSetup ) {
		WavePeripheral_Notify(
			*Data,
			PS_ServiceUUID,
			PS_MeasurementUUID,   
			[&]( std::string Bytes )
			{
				if ( Bytes.size() <= 0 )
					return;

				uint16_t Flags = ( reinterpret_cast< uint16_t* >( &Bytes[0] ) )[0];
				uint16_t InstaneousPower = ( reinterpret_cast< uint16_t* >( &Bytes[2] ) )[0];
				ReadState->Power = ( float ) InstaneousPower;

				//WAVECONTROL_LOG( "Power %.2lf Watts\n", State->Power );
			}
		); 
		this->PeripheralTable->MarkServiceConnected( this->PeripheralHandleID, std::string( PS_ServiceUUID ) + " " + PS_MeasurementUUID );
	}
}

static const char* s_WaveTrainerDevice_PS_WahooBrakeServiceUUID = "00001818-0000-1000-8000-00805f9b34fb";
static const char* s_WaveTrainerDevice_PS_WahooBrakeExtensionUUID = "a026e005-0a7d-4ab3-97fa-f1500f9feb8b";

WaveTrainerDevice::WaveTrainerDevice( WavePeripheralTable* PTable )
	: WaveDeviceBase( PTable )
{
}

void WaveTrainerDevice::WriteSimulationParametersToDevice( WavePeripheral& Data )
{
	if ( this->TrainerControlMode == WAVECONTROL_TRAINERCONTROL_WAHOO ) {
		uint32_t Weight = this->WriteState->TotalWeight * 100.0f;
		uint32_t RollingCrr = this->WriteState->RollingResistance * 10000.0f;
		uint32_t WindCwr = this->WriteState->WindResistance * 1000.0f;

		std::string Cmdbuf; Cmdbuf.resize(7);
		Cmdbuf[0] = 0x43; // Set rider characteristics command.
		Cmdbuf[1] = ( char ) Weight;
		Cmdbuf[2] = ( char ) ( Weight >> 8 );
		Cmdbuf[3] = ( char ) RollingCrr;
		Cmdbuf[4] = ( char ) ( RollingCrr >> 8 );
		Cmdbuf[5] = ( char ) WindCwr;
		Cmdbuf[6] = ( char ) ( WindCwr >> 8 );

		WavePeripheral_WriteCommand( Data, s_WaveTrainerDevice_PS_WahooBrakeServiceUUID, s_WaveTrainerDevice_PS_WahooBrakeExtensionUUID, Cmdbuf );
	}
}

void WaveTrainerDevice::WriteCurrentGradeToDevice( WavePeripheral& Data )
{
	if ( this->TrainerControlMode == WAVECONTROL_TRAINERCONTROL_WAHOO ) {
		int16_t Gradient = ( this->WriteState->Gradient / 100.0f + 1.0f ) * 32768;

		std::string Cmdbuf; Cmdbuf.resize(3);
		Cmdbuf[0] = 0x46; // Set gradient command.
		Cmdbuf[1] = ( reinterpret_cast< char* >( &Gradient ) )[0];
		Cmdbuf[2] = ( reinterpret_cast< char* >( &Gradient ) )[1];
		
		WavePeripheral_WriteCommand( Data, s_WaveTrainerDevice_PS_WahooBrakeServiceUUID, s_WaveTrainerDevice_PS_WahooBrakeExtensionUUID, Cmdbuf );
	}
}

void WaveTrainerDevice::Update()
{
	WaveDeviceBase::Update();

	auto Data = this->GetPeripheralFromTableInternal();
	if ( !Data || !WavePeripheral_IsConnectable( *Data ) )
		return;
	if ( !WavePeripheral_IsConnected( *Data ) )
		return;

	bool NeedWahooBrakeControlSetup = !this->PeripheralTable->IsServiceConnected( this->PeripheralHandleID, std::string( s_WaveTrainerDevice_PS_WahooBrakeServiceUUID ) + " " + s_WaveTrainerDevice_PS_WahooBrakeExtensionUUID );
	if ( NeedWahooBrakeControlSetup ) {
		
		bool SupportsWahooBrakeControl = WavePeripheral_HasCharacteristic( *Data, s_WaveTrainerDevice_PS_WahooBrakeServiceUUID, s_WaveTrainerDevice_PS_WahooBrakeExtensionUUID );
		if ( SupportsWahooBrakeControl ) {

			// We need to activate this indicate for trainer to be engaged; otherwise command writes wont work.
			WavePeripheral_Indicate(
				*Data,
				s_WaveTrainerDevice_PS_WahooBrakeServiceUUID,
				s_WaveTrainerDevice_PS_WahooBrakeExtensionUUID,   
				[&]( std::string Bytes ){}
			); 
			this->TrainerControlMode = WAVECONTROL_TRAINERCONTROL_WAHOO;
			this->PeripheralTable->MarkServiceConnected( this->PeripheralHandleID, std::string( s_WaveTrainerDevice_PS_WahooBrakeServiceUUID ) + " " + s_WaveTrainerDevice_PS_WahooBrakeExtensionUUID );
			WriteStateCache.reset( nullptr );
		}
	}

	if ( this->TrainerControlMode == WAVECONTROL_TRAINERCONTROL_UNKNOWN )
		return;

	bool NeedsWriteSimulationParametersToDevice = false;
	bool NeedsWriteGradientToDevice = false;

	if ( !WriteStateCache.get() ) {
		WriteStateCache = std::make_unique< WaveCycleSensorWriteState >();
		NeedsWriteSimulationParametersToDevice = true;
		NeedsWriteGradientToDevice = true;
	} else if ( WriteStateCache->TotalWeight != this->WriteState->TotalWeight || WriteStateCache->RollingResistance != this->WriteState->RollingResistance ||
			WriteStateCache->WindResistance != this->WriteState->WindResistance || WriteStateCache->WindSpeed != this->WriteState->WindSpeed ) {
		NeedsWriteSimulationParametersToDevice = true;
	} else if ( WriteStateCache->Gradient != this->WriteState->Gradient ) {
		NeedsWriteGradientToDevice = true;
	}

	if ( NeedsWriteSimulationParametersToDevice ) {
		this->WriteSimulationParametersToDevice( *Data );
	}
	if ( NeedsWriteGradientToDevice ) {
		this->WriteCurrentGradeToDevice( *Data );
	}

	*WriteStateCache = *this->WriteState;
}