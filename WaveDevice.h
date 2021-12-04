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
#include <memory>
#include <unordered_map>

#define WAVECONTROL_DEVICE_HR 0
#define WAVECONTROL_DEVICE_CADENCE 1
#define WAVECONTROL_DEVICE_SPEED 2
#define WAVECONTROL_DEVICE_POWER 3
#define WAVECONTROL_DEVICE_TRAINER 4
#define WAVECONTROL_DEVICE_NUM 5

#define WAVECONTROL_TRAINERCONTROL_UNKNOWN -1
#define WAVECONTROL_TRAINERCONTROL_TACX 0
#define WAVECONTROL_TRAINERCONTROL_WAHOO 1
#define WAVECONTROL_TRAINERCONTROL_FTMS 2

struct WavePeripheral;
struct WaveControl_WheelSizeData;

struct WaveCycleSensorReadState
{
	int HR_BPM = -1;
	bool HR_SensorContact = false;
	float Speed = -1.0f; // Km / Hr;
	float Cadence = -1.0f; // RPM
	float Power = -1.0f; // Watts
};

struct WaveCycleSensorWriteState
{
	float TotalWeight = 80.0f; // Kg
	float RollingResistance = 0.00677f; // 0.0001 unitless
	float WindResistance = 0.6f; // 0.01 Kg/m
	float WindSpeed = 0.0f; // m/s
	float Gradient = 0.0f; // %
};

class WavePeripheralTable
{
	uint64_t NextPeripheralHandleID = 0x1280000;
	std::unordered_map< uint64_t, std::shared_ptr< WavePeripheral > > PeripheralHashtable;

public:
	WavePeripheralTable();
	virtual ~WavePeripheralTable();

	std::shared_ptr< WavePeripheral > GetPeripheral( uint64_t HandleID );

	void SetPeripheral( uint64_t HandleID, std::shared_ptr< WavePeripheral > Value );

	uint64_t AddPeripheral();

	void RemovePeripheral( uint64_t HandleID );

	void IncrementPeripheralConnectionRef( uint64_t HandleID );

	void DecrementPeripheralConnectionRef( uint64_t HandleID );

	void Update();

	bool IsServiceConnected( uint64_t HandleID, const std::string& ServiceUUID );

	void MarkServiceConnected( uint64_t HandleID, const std::string& ServiceUUID, bool Connected = true );
};

// ------------------------------------------------ WaveDevice -------------------------------------------------

class WaveDeviceBase
{
public:
	bool Enabled = false;

	uint64_t PeripheralHandleID = 0;
	WavePeripheralTable* PeripheralTable = nullptr;

	bool NeedToReferencePeripheral = true;
	std::shared_ptr< WaveCycleSensorReadState > ReadState;
	std::shared_ptr< WaveCycleSensorWriteState > WriteState;

protected:
	std::shared_ptr< WavePeripheral > GetPeripheralFromTableInternal();

public:
	WaveDeviceBase( WavePeripheralTable* PTable );
	virtual ~WaveDeviceBase();
	virtual void Update();
};

class WaveHRMonitor : public WaveDeviceBase
{
public:
	WaveHRMonitor( WavePeripheralTable* PTable );
	virtual void Update() override;
};

class WaveCadenceSpeedSensor : public WaveDeviceBase
{
	uint32_t WheelRevs = 0;
	uint16_t LastWheelTime = 0; // This is in 1/1024th of a second.
	uint16_t CrankRevs = 0;
	uint16_t LastCrankTime = 0; // This is in 1/1024th of a second.

	uint32_t Prev_WheelRevs = 0;
	uint16_t Prev_LastWheelTime = 0; // This is in 1/1024th of a second.
	uint16_t Prev_CrankRevs = 0;
	uint16_t Prev_LastCrankTime = 0; // This is in 1/1024th of a second.

	std::unique_ptr< WaveControl_WheelSizeData > WheelSize;

protected:
	void UpdateCSCFromData();

public:
	WaveCadenceSpeedSensor( WavePeripheralTable* PTable );

	void SetWheelSizeData( WaveControl_WheelSizeData& WheelSizeData );

	virtual void Update() override;
};

class WavePowerSensor : public WaveDeviceBase
{
public:
	WavePowerSensor( WavePeripheralTable* PTable );
	virtual void Update() override;
};

class WaveTrainerDevice : public WaveDeviceBase
{
	std::unique_ptr< WaveCycleSensorWriteState > WriteStateCache;
	int TrainerControlMode = WAVECONTROL_TRAINERCONTROL_UNKNOWN;

protected:
	void WriteSimulationParametersToDevice( WavePeripheral& Data );
	void WriteCurrentGradeToDevice( WavePeripheral& Data );

public:
	WaveTrainerDevice( WavePeripheralTable* PTable );
	virtual void Update() override;
};