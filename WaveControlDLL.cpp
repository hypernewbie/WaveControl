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

#include "WaveControlDLL.h"

#include "WaveControl.h"
#include "WaveGPX.h"
#include "WaveSimulation.h"

#pragma optimize("", off);

void WaveControlDLL_SetLogCallback( void ( *Callback ) ( const char* ) )
{
	WaveControlSetLogCallback( Callback );
}

WaveControlPtr WaveControlDLL_Init( void )
{
	return ( WaveControlPtr ) new WaveControl();
}

void WaveControlDLL_Shutdown( WaveControlPtr Cntl )
{
	auto W = ( WaveControl* ) Cntl;
	assert( W && W->MagicID == WAVECONTROL_MAGIC_ID );
	delete W;
}

void WaveControlDLL_ScanStart( WaveControlPtr Cntl, int AutoStopFrames )
{
	auto W = ( WaveControl* ) Cntl;
	assert( W && W->MagicID == WAVECONTROL_MAGIC_ID );
	W->ScanStart( AutoStopFrames );
}

void WaveControlDLL_ScanStop( WaveControlPtr Cntl )
{
	auto W = ( WaveControl* ) Cntl;
	assert( W && W->MagicID == WAVECONTROL_MAGIC_ID );
	W->ScanStop();
}

void WaveControlDLL_ListPeripherals( WaveControlPtr Cntl, void( *Callback )( const char*, const char* ) )
{
	auto W = ( WaveControl* ) Cntl;
	assert( Callback );
	assert( W && W->MagicID == WAVECONTROL_MAGIC_ID );
	
	auto Peripherals = W->ListPeripherals();
	for ( auto& P : Peripherals ) {
		Callback( P.first.c_str() , P.second.c_str() );
	}
}

void WaveControlDLL_ChooseDeviceForUsage( WaveControlPtr Cntl, int Usage, const char* UIAddress )
{
	auto W = ( WaveControl* ) Cntl;
	assert( W && W->MagicID == WAVECONTROL_MAGIC_ID );
	W->ChooseDeviceForUsage( Usage, UIAddress );
}

int WaveControlDLL_DoesDeviceAdvertiseUsage( WaveControlPtr Cntl, int Usage, const char* UIAddress, const char* ServiceID )
{
	auto W = ( WaveControl* ) Cntl;
	assert( W && W->MagicID == WAVECONTROL_MAGIC_ID );
	return W->DoesDeviceAdvertiseUsage( Usage, UIAddress, ServiceID );
}

void WaveControlDLL_ChooseWheelSize( WaveControlPtr Cntl, const char* Name )
{
	auto W = ( WaveControl* ) Cntl;
	assert( W && W->MagicID == WAVECONTROL_MAGIC_ID );
	W->ChooseWheelSize( Name );
}

WaveCycleSensorReadStateDLL WaveControlDLL_GetSensorReadState( WaveControlPtr Cntl )
{
	auto W = ( WaveControl* ) Cntl;
	assert( W && W->MagicID == WAVECONTROL_MAGIC_ID );
	assert( sizeof( WaveCycleSensorReadStateDLL ) == sizeof( WaveCycleSensorReadState ) );
	return *reinterpret_cast<WaveCycleSensorReadStateDLL*>( W->GetSensorReadState().get() );
}

WaveCycleSensorWriteStateDLL WaveControlDLL_GetSensorWriteState( WaveControlPtr Cntl )
{
	auto W = ( WaveControl* ) Cntl;
	assert( W && W->MagicID == WAVECONTROL_MAGIC_ID );
	assert( sizeof( WaveCycleSensorWriteStateDLL ) == sizeof( WaveCycleSensorWriteState ) );
	return *reinterpret_cast<WaveCycleSensorWriteStateDLL*>( W->GetSensorWriteState().get() );
}

void WaveControlDLL_SetSensorWriteState( WaveControlPtr Cntl, WaveCycleSensorWriteStateDLL* WriteState )
{
	auto W = ( WaveControl* ) Cntl;
	assert( W && W->MagicID == WAVECONTROL_MAGIC_ID );
	assert( WriteState );
	assert( sizeof( WaveCycleSensorWriteStateDLL ) == sizeof( WaveCycleSensorWriteState ) );
	*W->GetSensorWriteState() = *reinterpret_cast<WaveCycleSensorWriteState*>( WriteState );
}

// --------------------------------------------------------------------------------------------------------------------------

WaveSimulationPtr WaveSimulationDLL_Init( void )
{
	return ( WaveSimulationPtr ) new WaveSimulation();
}

void WaveSimulationDLL_Release( WaveSimulationPtr Sim )
{
	auto S = ( WaveSimulation* ) Sim;
	assert( S && S->MagicID == WAVESIM_MAGIC_ID );
	delete S;
}

void WaveSimulationDLL_Update( WaveSimulationPtr Sim, float DeltaTime )
{
	auto S = ( WaveSimulation* ) Sim;
	assert( S && S->MagicID == WAVESIM_MAGIC_ID );
	S->Update( DeltaTime );
}

float WaveSimulationDLL_GetPosition( WaveSimulationPtr Sim )
{
	auto S = ( WaveSimulation* ) Sim;
	assert( S && S->MagicID == WAVESIM_MAGIC_ID );
	return S->GetPosition();
}

float WaveSimulationDLL_GetSpeed( WaveSimulationPtr Sim )
{
	auto S = ( WaveSimulation* ) Sim;
	assert( S && S->MagicID == WAVESIM_MAGIC_ID );
	return S->GetSpeed();
}

void WaveSimulationDLL_ResetRouteState( WaveSimulationPtr Sim )
{
	auto S = ( WaveSimulation* ) Sim;
	assert( S && S->MagicID == WAVESIM_MAGIC_ID );
	S->ResetRouteState();
}

int WaveSimulationDLL_GetPowerZone( WaveSimulationPtr Sim )
{
	auto S = ( WaveSimulation* ) Sim;
	assert( S && S->MagicID == WAVESIM_MAGIC_ID );
	return S->GetPowerZone();
}

void WaveSimulationDLL_GetState( WaveSimulationPtr Sim, WaveSimulationStateDLL* State )
{
	auto S = ( WaveSimulation* ) Sim;
	assert( S && S->MagicID == WAVESIM_MAGIC_ID );
	assert( State );

	// These are inputs. Sync this with WaveControlDLLImport.h!
	State->RiderPower = S->RiderPower;
	State->RiderWeight = S->RiderWeight;
	State->BikeWeight = S->BikeWeight;
	State->TireCrr = S->TireCrr;
	State->BikeDragCoeff = S->BikeDragCoeff;
	State->RiderFrontalArea = S->RiderFrontalArea;
	State->Grade = S->Grade;
	State->Altitude = S->Altitude;
	State->DrivetrainEfficiency = S->DrivetrainEfficiency;
	State->RiderFTP = S->RiderFTP;
}

void WaveSimulationDLL_SetState( WaveSimulationPtr Sim, WaveSimulationStateDLL* State )
{
	auto S = ( WaveSimulation* ) Sim;
	assert( S && S->MagicID == WAVESIM_MAGIC_ID );
	assert( State );

	// These are inputs. Sync this with WaveControlDLLImport.h!
	S->RiderPower = State->RiderPower;
	S->RiderWeight = State->RiderWeight;
	S->BikeWeight = State->BikeWeight;
	S->TireCrr = State->TireCrr;
	S->BikeDragCoeff = State->BikeDragCoeff;
	S->RiderFrontalArea = State->RiderFrontalArea;
	S->Grade = State->Grade;
	S->Altitude = State->Altitude;
	S->DrivetrainEfficiency = State->DrivetrainEfficiency;
	S->RiderFTP = State->RiderFTP;
}

// --------------------------------------------------------------------------------------------------------------------------

WaveGPXPtr WaveGPXDLL_Init( void )
{
	return ( WaveSimulationPtr ) new WaveGPX();
}

void WaveGPXDLL_Release( WaveGPXPtr GPX )
{
	auto G = ( WaveGPX* ) GPX;
	assert( G && G->MagicID == WAVESIM_MAGIC_ID );
	delete G;
}

int WaveGPXDLL_LoadRouteGPX( WaveGPXPtr GPX, WaveGPXRouteDLL* Route, const char* FileName )
{
	auto G = ( WaveGPX* ) GPX;
	assert( G && G->MagicID == WAVESIM_MAGIC_ID );
	assert( sizeof( WaveGPXPointDLL ) == sizeof( FWaveGPXPoint ) );

	FWaveGPXRoute* Temp = (FWaveGPXRoute*) Route->InternalObject;
	if ( !Route->InternalObject ) {
		Temp = new FWaveGPXRoute();
		Route->InternalObject = ( FWaveGPXRoute* ) Temp;
	}
	
	if ( !G->LoadRouteGPX( *Temp, FileName ) )
		return 0;

	Route->Name = Temp->Name.c_str();
	Route->Description = Temp->Description.c_str();
	Route->Author = Temp->Author.c_str();
	Route->SourceFile = Temp->SourceFile.c_str();

	Route->NumPoints = ( int ) Temp->Points.size();
	Route->Points = ( WaveGPXPointDLL* ) Temp->Points.data();

	Route->Stat_Elev = Temp->Stat_Elev;
	Route->Stat_Length = Temp->Stat_Length;
	Route->Stat_HillinessRating = Temp->Stat_HillinessRating;
	Route->Stat_DifficultyScore = Temp->Stat_DifficultyScore;
	Route->Stat_HighestAlt = Temp->Stat_HighestAlt;
	Route->Stat_LowestAlt = Temp->Stat_LowestAlt;

	return 1;
}

void WaveGPXDLL_ReleaseRouteGPX( WaveGPXRouteDLL* Route )
{
	if ( Route->InternalObject ) {
		FWaveGPXRoute* Temp = ( FWaveGPXRoute* ) Route->InternalObject;
		delete Temp;
		Route->InternalObject = nullptr;
	}
}

WaveGPXRecordPtr WaveGPXDLL_CreateRecord( WaveGPXPtr GPX )
{
	auto G = ( WaveGPX* ) GPX;
	assert( G && G->MagicID == WAVESIM_MAGIC_ID );
	FWaveGPXRecord* Record = new FWaveGPXRecord();
	return Record;
}

void WaveGPXDLL_ReleaseRecord( WaveGPXPtr GPX, WaveGPXRecordPtr Rec )
{
	auto G = ( WaveGPX* ) GPX;
	assert( G && G->MagicID == WAVESIM_MAGIC_ID );
	auto Record = ( FWaveGPXRecord* ) Rec;
	delete Record;
}

void WaveGPXDLL_RecordStart( WaveGPXPtr GPX, WaveGPXRecordPtr Record, const WaveGPXRouteDLL* SrcInfo )
{
	auto G = ( WaveGPX* ) GPX;
	assert( G && G->MagicID == WAVESIM_MAGIC_ID );
	
	auto Rec = ( FWaveGPXRecord* ) Record;
	auto RouteSrcInfo = ( const FWaveGPXRoute* ) SrcInfo->InternalObject;
	G->RecordStart( *Rec, *RouteSrcInfo );
}

void WaveGPXDLL_RecordAddPoint( WaveGPXPtr GPX, WaveGPXRecordPtr Record, WaveGPXPointDLL Point, bool TimeNow, uint64_t TimeEpochMS, float Power, float Cadence, float HR )
{
	auto G = ( WaveGPX* ) GPX;
	assert( G && G->MagicID == WAVESIM_MAGIC_ID );
	assert( sizeof( WaveGPXPointDLL ) == sizeof( FWaveGPXPoint ) );
	
	auto Rec = ( FWaveGPXRecord* ) Record;
	std::chrono::system_clock::time_point Time = std::chrono::system_clock::now();
	if ( !TimeNow ) {
		// ref: https://stackoverflow.com/questions/31255486/c-how-do-i-convert-a-stdchronotime-point-to-long-and-back
		std::chrono::milliseconds dur( TimeEpochMS );
		std::chrono::time_point<std::chrono::system_clock> dt( dur );
		Time = dt;
	}
	auto PointInternal = ( FWaveGPXPoint* ) &Point;

	G->RecordAddPoint( *Rec, *PointInternal, Time, Power, Cadence, HR );
}

bool WaveGPXDLL_RecordFinish( WaveGPXPtr GPX, WaveGPXRecordPtr Record, const char* FileName )
{
	auto G = ( WaveGPX* ) GPX;
	assert( G && G->MagicID == WAVESIM_MAGIC_ID );

	auto Rec = ( FWaveGPXRecord* ) Record;
	return G->RecordFinish( *Rec, FileName );
}

void WaveGPXDLL_FillENUFromLLA( WaveGPXRouteDLL* Route, WaveGPXPointDLL* Point )
{
	assert( sizeof( WaveGPXPointDLL ) == sizeof( FWaveGPXPoint ) );
	auto PointInternal = ( FWaveGPXPoint* ) Point;
	auto RouteSrcInfo = ( const FWaveGPXRoute* ) Route->InternalObject;
	WaveRouteUtil_FillENUFromLLA( *RouteSrcInfo, *PointInternal );
}

void WaveGPXDLL_FillLLAFromENU( WaveGPXRouteDLL* Route, WaveGPXPointDLL* Point )
{
	assert( sizeof( WaveGPXPointDLL ) == sizeof( FWaveGPXPoint ) );
	auto PointInternal = ( FWaveGPXPoint* ) Point;
	auto RouteSrcInfo = ( const FWaveGPXRoute* ) Route->InternalObject;
	WaveRouteUtil_FillLLAFromENU( *RouteSrcInfo, *PointInternal );
}