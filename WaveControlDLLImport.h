/*
	Copyright 2021 Xi Chen (hypernewbie@gmail.com)

	Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
	associated documentation files (the "Software"), to deal in the Software without restriction,
	including without limitation the rights to use, copy, modify, merge, publish, distribute,
	sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all copies or substantial
	portions of the Software.

	THE SOFTWARE IS PROVIDuED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
	NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
	NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
	DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT
	OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

extern "C" {

	typedef void* WaveControlPtr;

	// Keep in sync with WaveDevice.h!
	struct WaveCycleSensorReadStateDLL
	{
		int HR_BPM = -1;
		char HR_SensorContact = 0;
		float Speed = -1.0f; // Km / Hr;
		float Cadence = -1.0f; // RPM
		float Power = -1.0f; // Watts
	};

	// Keep in sync with WaveDevice.h!
	struct WaveCycleSensorWriteStateDLL
	{
		float TotalWeight = 80.0f; // Kg
		float RollingResistance = 0.00677f; // 0.0001 unitless
		float WindResistance = 0.6f; // 0.01 Kg/m
		float WindSpeed = 0.0f; // m/s
		float Gradient = 0.0f; // %
	};

	struct WaveControlDLL
	{
		void (*SetLogCallback) ( void ( *Callback ) ( const char* ) );

		WaveControlPtr (*Init) ( void );

		void (*Shutdown) ( WaveControlPtr Cntl );

		void (*ScanStart) ( WaveControlPtr Cntl, int AutoStopFrames );

		void (*ScanStop) ( WaveControlPtr Cntl );

		void (*ListPeripherals) ( WaveControlPtr Cntl, void( *Callback )( const char*, const char* ) );

		void (*ChooseDeviceForUsage) ( WaveControlPtr Cntl, int Usage, const char* UIAddress );

		// 0 = no, 1 , yes, -1 = device not found.
		int ( *DoesDeviceAdvertiseUsage ) ( WaveControlPtr Cntl, int Usage, const char* UIAddress, const char* ServiceID );

		void ( *ChooseWheelSize ) ( WaveControlPtr Cntl, const char* Name );

		WaveCycleSensorReadStateDLL( *GetSensorReadState ) ( WaveControlPtr Cntl );

		WaveCycleSensorWriteStateDLL( *GetSensorWriteState )( WaveControlPtr Cntl );

		void ( *SetSensorWriteState )( WaveControlPtr Cntl, WaveCycleSensorWriteStateDLL* WriteState );
	};

	// --------------------------------------------------------------------------------------------------------------------------
	
	typedef void* WaveSimulationPtr;

	// Sync this with WaveSimulation.h and WaveControlDLL.cpp!
	struct WaveSimulationStateDLL
	{
		float RiderPower = 170.0f; // Watts
		float RiderWeight = 83.0f; // KG
		float BikeWeight = 9.08f + 0.34f; // KG
		float TireCrr = 0.00677f;
		float BikeDragCoeff = 0.72f;
		float RiderFrontalArea = 0.42f;
		float Grade = 0.0f; // Percent
		float Altitude = 100.0f; // M
		float DrivetrainEfficiency = 0.95f;
		float RiderFTP = 170.0f; // Unused for simulation but may be useful to UI.
	};

	struct WaveSimulationDLL
	{
		WaveSimulationPtr (*Init) ( void );

		void (*Release) ( WaveSimulationPtr Sim );

		void (*Update) ( WaveSimulationPtr Sim, float DeltaTime );

		float (*GetPosition) ( WaveSimulationPtr Sim );

		float (*GetSpeed) ( WaveSimulationPtr Sim );

		void (*ResetRouteState) ( WaveSimulationPtr Sim );

		int (*GetPowerZone) ( WaveSimulationPtr Sim );

		void (*GetState) ( WaveSimulationPtr Sim, WaveSimulationStateDLL* State );

		void (*SetState) ( WaveSimulationPtr Sim, WaveSimulationStateDLL* State );
	};

	// --------------------------------------------------------------------------------------------------------------------------

	typedef void* WaveGPXPtr;
	typedef void* WaveGPXRecordPtr;

	struct WaveGPXPointDLL
	{
		double Lat = 0.0f;
		double Lon = 0.0f;
		double Alt = 0.0f;

		double East = 0.0f;
		double North = 0.0f;
		double Up = 0.0f;

		double Dist = 0.0f;
	};

	struct WaveGPXRouteDLL
	{
		// Owned by DLL memory. Do not touch!
		const char* Name;
		const char* Description;
		const char* Author;
		const char* SourceFile;

		// Owned by DLL memory. Do not touch!
		int NumPoints;
		WaveGPXPointDLL* Points;

		float Stat_Elev = 0.0f;
		float Stat_Length = 1.0f;
		float Stat_HillinessRating = 0.0f;
		float Stat_DifficultyScore = 0.0f;
		float Stat_HighestAlt = 0.0f;
		float Stat_LowestAlt = 0.0f;

		void* InternalObject = nullptr;
	};

	struct WaveGPXDLL
	{
		WaveGPXPtr (*Init) ( void );

		void (*Release) ( WaveGPXPtr GPX );

		int (*LoadRouteGPX) ( WaveGPXPtr GPX, WaveGPXRouteDLL* Route, const char* FileName );

		void (*ReleaseRouteGPX) ( WaveGPXRouteDLL* Route );

		WaveGPXRecordPtr (*CreateRecord) ( WaveGPXPtr GPX );

		void (*ReleaseRecord) ( WaveGPXPtr GPX, WaveGPXRecordPtr Rec );

		void (*RecordStart) ( WaveGPXPtr GPX, WaveGPXRecordPtr Record, const WaveGPXRouteDLL* SrcInfo );

		void (*RecordAddPoint) ( WaveGPXPtr GPX, WaveGPXRecordPtr Record, WaveGPXPointDLL Point, bool TimeNow, uint64_t TimeEpochMS, float Power, float Cadence, float HR );

		bool (*RecordFinish) ( WaveGPXPtr GPX, WaveGPXRecordPtr Record, const char* FileName );

		void (*FillENUFromLLA) ( WaveGPXRouteDLL* Route, WaveGPXPointDLL* Point );

		void (*FillLLAFromENU) ( WaveGPXRouteDLL* Route, WaveGPXPointDLL* Point );
	};

}

#include <functional>

struct WaveControl_LoadDLLInterface
{
	std::function< void* ( const char* FileName ) > LoadLib;
	std::function< void* ( void* Handle, const char* FuncName ) > GetFunc;
};

inline bool WaveControl_LoadDLL( WaveControl_LoadDLLInterface& I, WaveControlDLL* W, WaveSimulationDLL* S, WaveGPXDLL *G, const char* DLLFileName = "WaveControl.dll" )
{
	auto LibHandle = I.LoadLib( DLLFileName );
	if ( !LibHandle )
		return false;

	W->SetLogCallback = ( void ( * ) ( void ( *Callback ) ( const char* ) ) ) I.GetFunc( LibHandle, "WaveControlDLL_SetLogCallback" );
	W->Init = ( WaveControlPtr( * )( void ) ) I.GetFunc( LibHandle, "WaveControlDLL_Init" );
	W->Shutdown = ( void( * )( WaveControlPtr ) ) I.GetFunc( LibHandle, "WaveControlDLL_Shutdown" );
	W->ScanStart = ( void( * )( WaveControlPtr, int AutoStopFrames ) ) I.GetFunc( LibHandle, "WaveControlDLL_ScanStart" );
	W->ScanStop = ( void( * )( WaveControlPtr ) ) I.GetFunc( LibHandle, "WaveControlDLL_ScanStop" );
	W->ListPeripherals = ( void( * )( WaveControlPtr, void( * )( const char*, const char* ) ) ) I.GetFunc( LibHandle, "WaveControlDLL_ListPeripherals" );
	W->ChooseDeviceForUsage = ( void( * )( WaveControlPtr, int, const char* ) ) I.GetFunc( LibHandle, "WaveControlDLL_ChooseDeviceForUsage" );
	W->DoesDeviceAdvertiseUsage = ( int( * )( WaveControlPtr, int, const char*, const char* ) ) I.GetFunc( LibHandle, "WaveControlDLL_DoesDeviceAdvertiseUsage" );
	W->ChooseWheelSize = ( void( * )( WaveControlPtr, const char* ) ) I.GetFunc( LibHandle, "WaveControlDLL_ChooseWheelSize" );
	W->GetSensorReadState = ( WaveCycleSensorReadStateDLL( * ) ( WaveControlPtr Cntl ) ) I.GetFunc( LibHandle, "WaveControlDLL_GetSensorReadState" );
	W->GetSensorWriteState = ( WaveCycleSensorWriteStateDLL( * )( WaveControlPtr Cntl ) ) I.GetFunc( LibHandle, "WaveControlDLL_GetSensorWriteState" );
	W->SetSensorWriteState = ( void ( * )( WaveControlPtr Cntl, WaveCycleSensorWriteStateDLL * WriteState ) ) I.GetFunc( LibHandle, "WaveControlDLL_SetSensorWriteState" );

	S->Init = ( WaveSimulationPtr( * )( void ) ) I.GetFunc( LibHandle, "WaveSimulationDLL_Init" );
	S->Release = ( void( * )( WaveSimulationPtr Sim ) ) I.GetFunc( LibHandle, "WaveSimulationDLL_Release" );
	S->Update = ( void( * )( WaveSimulationPtr Sim, float DeltaTime ) ) I.GetFunc( LibHandle, "WaveSimulationDLL_Update" );
	S->GetPosition = ( float( * )( WaveSimulationPtr Sim ) ) I.GetFunc( LibHandle, "WaveSimulationDLL_GetPosition" );
	S->GetSpeed = ( float( * )( WaveSimulationPtr Sim ) ) I.GetFunc( LibHandle, "WaveSimulationDLL_GetSpeed" );
	S->ResetRouteState = ( void( * )( WaveSimulationPtr Sim ) ) I.GetFunc( LibHandle, "WaveSimulationDLL_ResetRouteState" );
	S->GetPowerZone = ( int( * )( WaveSimulationPtr Sim ) ) I.GetFunc( LibHandle, "WaveSimulationDLL_GetPowerZone" );
	S->GetState = ( void( * )( WaveSimulationPtr Sim, WaveSimulationStateDLL * State ) ) I.GetFunc( LibHandle, "WaveSimulationDLL_GetState" );
	S->SetState = ( void( * )( WaveSimulationPtr Sim, WaveSimulationStateDLL * State ) ) I.GetFunc( LibHandle, "WaveSimulationDLL_SetState" );

	G->Init = ( WaveGPXPtr (*) ( void ) ) I.GetFunc( LibHandle, "WaveGPXDLL_Init");
	G->Release = ( void (*) ( WaveGPXPtr GPX ) ) I.GetFunc( LibHandle, "WaveGPXDLL_Release");
	G->LoadRouteGPX = ( int (*) ( WaveGPXPtr GPX, WaveGPXRouteDLL* Route, const char* FileName ) ) I.GetFunc( LibHandle, "WaveGPXDLL_LoadRouteGPX");
	G->ReleaseRouteGPX = ( void (*) ( WaveGPXRouteDLL* Route ) ) I.GetFunc( LibHandle, "WaveGPXDLL_ReleaseRouteGPX");
	G->CreateRecord = ( WaveGPXRecordPtr (*) ( WaveGPXPtr GPX ) ) I.GetFunc( LibHandle, "WaveGPXDLL_CreateRecord");
	G->ReleaseRecord = ( void (*) ( WaveGPXPtr GPX, WaveGPXRecordPtr Rec ) ) I.GetFunc( LibHandle, "WaveGPXDLL_ReleaseRecord");
	G->RecordStart = ( void (*) ( WaveGPXPtr GPX, WaveGPXRecordPtr Record, const WaveGPXRouteDLL* SrcInfo ) ) I.GetFunc( LibHandle, "WaveGPXDLL_RecordStart");
	G->RecordAddPoint = ( void (*) ( WaveGPXPtr GPX, WaveGPXRecordPtr Record, WaveGPXPointDLL Point, bool TimeNow, uint64_t TimeEpochMS, float Power, float Cadence, float HR ) ) I.GetFunc( LibHandle, "WaveGPXDLL_RecordAddPoint");
	G->RecordFinish = ( bool (*) ( WaveGPXPtr GPX, WaveGPXRecordPtr Record, const char* FileName ) ) I.GetFunc( LibHandle, "WaveGPXDLL_RecordFinish");
	G->FillENUFromLLA = ( void (*) ( WaveGPXRouteDLL* Route, WaveGPXPointDLL* Point ) ) I.GetFunc( LibHandle, "WaveGPXDLL_FillENUFromLLA");
	G->FillLLAFromENU = ( void (*) ( WaveGPXRouteDLL* Route, WaveGPXPointDLL* Point ) ) I.GetFunc( LibHandle, "WaveGPXDLL_FillLLAFromENU");

	return true;
}