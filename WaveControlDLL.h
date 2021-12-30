/*
	Copyright 2021 Xi Chen (hypernewbie@gmail.com)

	Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
	associated documentation files (the "Software"), to deal in the Software without restriction,
	including without limitation the rights to use, copy, modify, merge, publish, distribute,l
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

#include <cstdint>
#include "WaveControlDLLImport.h"

extern "C" {

	__declspec( dllexport ) void WaveControlDLL_SetLogCallback( void ( *Callback ) ( const char* ) );

	__declspec( dllexport ) WaveControlPtr WaveControlDLL_Init( void );

	__declspec( dllexport ) void WaveControlDLL_Shutdown( WaveControlPtr Cntl );

	__declspec( dllexport ) void WaveControlDLL_ScanStart( WaveControlPtr Cntl, int AutoStopFrames );

	__declspec( dllexport ) void WaveControlDLL_ScanStop( WaveControlPtr Cntl );

	__declspec( dllexport ) void WaveControlDLL_ListPeripherals( WaveControlPtr Cntl, void( *Callback )( const char*, const char* ) );

	__declspec( dllexport ) void WaveControlDLL_ChooseDeviceForUsage( WaveControlPtr Cntl, int Usage, const char* UIAddress );

	__declspec( dllexport ) int WaveControlDLL_DoesDeviceAdvertiseUsage( WaveControlPtr Cntl, int Usage, const char* UIAddress, const char* ServiceID );

	__declspec( dllexport ) void WaveControlDLL_ChooseWheelSize( WaveControlPtr Cntl, const char* Name );

	__declspec( dllexport ) WaveCycleSensorReadStateDLL WaveControlDLL_GetSensorReadState( WaveControlPtr Cntl );

	__declspec( dllexport ) WaveCycleSensorWriteStateDLL WaveControlDLL_GetSensorWriteState( WaveControlPtr Cntl );

	__declspec( dllexport ) void WaveControlDLL_SetSensorWriteState( WaveControlPtr Cntl, WaveCycleSensorWriteStateDLL* WriteState );

	// --------------------------------------------------------------------------------------------------------------------------

	__declspec( dllexport ) WaveSimulationPtr WaveSimulationDLL_Init( void );

	__declspec( dllexport ) void WaveSimulationDLL_Release( WaveSimulationPtr Sim );

	__declspec( dllexport ) void WaveSimulationDLL_Update( WaveSimulationPtr Sim, float DeltaTime );
	
	__declspec( dllexport ) float WaveSimulationDLL_GetPosition( WaveSimulationPtr Sim );

	__declspec( dllexport ) float WaveSimulationDLL_GetSpeed( WaveSimulationPtr Sim );

	__declspec( dllexport ) void WaveSimulationDLL_ResetRouteState( WaveSimulationPtr Sim );

	__declspec( dllexport ) int WaveSimulationDLL_GetPowerZone( WaveSimulationPtr Sim );

	__declspec( dllexport ) void WaveSimulationDLL_GetState( WaveSimulationPtr Sim, WaveSimulationStateDLL* State );

	__declspec( dllexport ) void WaveSimulationDLL_SetState( WaveSimulationPtr Sim, WaveSimulationStateDLL* State );

	// --------------------------------------------------------------------------------------------------------------------------

	__declspec( dllexport ) WaveGPXPtr WaveGPXDLL_Init( void );

	__declspec( dllexport ) void WaveGPXDLL_Release( WaveGPXPtr GPX );

	__declspec( dllexport ) int WaveGPXDLL_LoadRouteGPX( WaveGPXPtr GPX, WaveGPXRouteDLL* Route, const char* FileName );

	__declspec( dllexport ) void WaveGPXDLL_ReleaseRouteGPX( WaveGPXRouteDLL* Route );

	__declspec( dllexport ) WaveGPXRecordPtr WaveGPXDLL_CreateRecord( WaveGPXPtr GPX );

	__declspec( dllexport ) void WaveGPXDLL_ReleaseRecord( WaveGPXPtr GPX, WaveGPXRecordPtr Rec );

	__declspec( dllexport ) void WaveGPXDLL_RecordStart( WaveGPXPtr GPX, WaveGPXRecordPtr Record, const WaveGPXRouteDLL* SrcInfo );

	__declspec( dllexport ) void WaveGPXDLL_RecordAddPoint( WaveGPXPtr GPX, WaveGPXRecordPtr Record, WaveGPXPointDLL Point, bool TimeNow, uint64_t TimeEpochMS, float Power, float Cadence, float HR );

	__declspec( dllexport ) bool WaveGPXDLL_RecordFinish( WaveGPXPtr GPX, WaveGPXRecordPtr Record, const char* FileName );

	__declspec( dllexport ) void WaveGPXDLL_FillENUFromLLA( WaveGPXRouteDLL* Route, WaveGPXPointDLL* Point );

	__declspec( dllexport ) void WaveGPXDLL_FillLLAFromENU( WaveGPXRouteDLL* Route, WaveGPXPointDLL* Point );
}

