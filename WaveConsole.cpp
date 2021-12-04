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

#include "WaveControl.h"
#include "WaveGPX.h"
#include "WaveSimulation.h"

#include <iostream>
#include "Include/cppconlib/include/conmanip.h"

bool WaveTest( int argc, char * argv[] );

void SimpleDemo( WaveControl& w )
{
	auto SensorReadState = w.GetSensorReadState();
	auto SensorWriteState = w.GetSensorWriteState();
	for ( int i = 0; i < 600; i++ ) {
		std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
		SensorWriteState->Gradient = i / 100.0f;
		WAVECONTROL_LOG( "HR: %d BPM (%s) | Speed %.2f Km/hr | Cadence %.0f RPM | Power %.0f W | Gradient %.1f\n",
			SensorReadState->HR_BPM, SensorReadState->HR_SensorContact ? "Contact" : "No Contact",
			SensorReadState->Speed, SensorReadState->Cadence, SensorReadState->Power, SensorWriteState->Gradient
		);
	}
}

#pragma optimize("", off);

void FullRouteSimulationDemo( WaveControl& w )
{
	using namespace conmanip;
	console_out_context ctxout;
	console_out conout(ctxout);

	conout.settitle( "W A V E C O N T R O L    D E M O" );

	WaveSimulation Sim;
	Sim.RiderWeight = 83.5f;
	Sim.RiderPower = 0.0f;

	WaveRouteState WRS;
	FWaveGPXRoute Route;
	WRS.LoadRouteGPX( Route, "TestFiles/HawkHill.gpx" );

	auto SensorReadState = w.GetSensorReadState();
	auto SensorWriteState = w.GetSensorWriteState();
	SensorWriteState->TotalWeight = Sim.RiderWeight + Sim.BikeWeight;
	SensorWriteState->RollingResistance = Sim.TireCrr;

	int FrameIdx = 0;
	float VAM = 0.0f, PrevAlt = FLT_MAX;
	int FrameTimeMS = 33;

	while ( true ) {
		//Sim.RiderPower = SensorReadState->Power;
		Sim.RiderPower = 9002370.0f;
		Sim.Update( FrameTimeMS / 1000.0f );
		auto CurrentSimulationPos = WaveRouteUtil_FindENUPosAtDist( Route, Sim.GetPosition() );
		auto Gradient = WaveRouteUtil_FindGradePosAtDist( Route, Sim.GetPosition() );
		Sim.Grade = Gradient;

		// --------------- Draw Stats ------------------------

		std::cout << setposx( 1 ) << setposy( 8 ); printf( ">> Power %.0f W ( %.1f W / kg )       ", Sim.RiderPower, Sim.RiderPower / Sim.RiderWeight );
		std::cout << setposx( 1 ) << setposy( 9 ); printf( ">> HR %d BPM        ", SensorReadState->HR_BPM );
		std::cout << setposx( 1 ) << setposy( 10 ); printf( ">> Cadence %.0f RPM        ", SensorReadState->Cadence );

		std::cout << setposx( 1 ) << setposy( 13 ); printf( ">> Speed %.1f Km/hr ( %.1f MPH )        ", Sim.GetSpeed() * 3.6, Sim.GetSpeedMPH() );
		std::cout << setposx( 1 ) << setposy( 14 ); printf( ">> Gradient %.1f %%        ", SensorWriteState->Gradient );
		std::cout << setposx( 1 ) << setposy( 15 ); printf( ">> Distance %.1f m / %.1f m        ", Sim.GetPosition(), Route.Stat_Length );
		std::cout << setposx( 1 ) << setposy( 16 ); printf( ">> Elev %.2f m ( %.1f ft )       ", CurrentSimulationPos.Alt, CurrentSimulationPos.Alt * 3.28084f );
		std::cout << setposx( 1 ) << setposy( 17 ); printf( ">> VAM %.0f m/hr        ", VAM );

		// --------------- Draw Map ------------------------

		static char MapBuffer[20][41];
		for ( int i = 0; i < 20; i++ ) {
			for ( int j = 0; j < 40; j++ ) {
				MapBuffer[i][j] = 176;
			}
			MapBuffer[i][40] = '\0';
		}
		for ( int i = 0; i < Route.Points.size(); i++ ) {
			int MapX = ( Route.Points[i].East - CurrentSimulationPos.East ) * 0.015f * 2.0f;
			int MapY = ( Route.Points[i].North - CurrentSimulationPos.North ) * -0.015f;
			MapX += 20; MapY += 10;
			if ( MapX >= 0 && MapX < 40 && MapY >= 0 && MapY < 20 ) {
				MapBuffer[MapY][MapX] = 178;
			}
		}
		MapBuffer[10][20] = 2;
		for ( int i = 0; i < 20; i++ ) {
			std::cout << setposx( 40 ) << setposy( 1 + i );
			puts( MapBuffer[i] );
		}


		// --------------- Draw Elevation Map ------------------------

		static char ElevMapBuffer[10][81];
		for ( int i = 0; i < 10; i++ ) {
			for ( int j = 0; j < 80; j++ ) {
				ElevMapBuffer[i][j] = ' ';
			}
			ElevMapBuffer[i][80] = '\0';
		}
		{
			int CurrentPos = ( Sim.GetPosition() / Route.Stat_Length ) * 80.0f;
			if ( CurrentPos >= 0 && CurrentPos < 80 ) {
				for ( int i = 0; i < 10; i++ ) {
					ElevMapBuffer[i][CurrentPos] = '|';
				}
			}
		}
		for ( int i = 0; i < 80; i++ ) {
			float Dist = ( i / 79.0f ) * Route.Stat_Length;
			
			auto MapPos = WaveRouteUtil_FindENUPosAtDist( Route, Dist );
			auto MapPosGradient = WaveRouteUtil_FindGradePosAtDist( Route, Dist );
			int MapZ = ( MapPos.Alt / Route.Stat_HighestAlt ) * 10.0f;
			
			if ( MapZ >= 0 && MapZ < 10 ) {
				char GradientCharSlope = '-';
				if ( MapPosGradient > 3.5f ) GradientCharSlope = '/';
				if ( MapPosGradient < -3.5f ) GradientCharSlope = '\\';
				ElevMapBuffer[9 - MapZ][i] = GradientCharSlope;
			}
		}
		for ( int i = 0; i < 10; i++ ) {
			std::cout << setposx( 1 ) << setposy( 22 + i );
			puts( ElevMapBuffer[i] );
		}

		std::cout << setposx( 0 ) << setposy( 20 );

		// Rate limit changes to gradient to not spam too much.
		if ( FrameIdx++ > 20 ) {
			FrameIdx = 0;
			SensorWriteState->Gradient = Gradient;
		}

		// Calculate VAM.
		float InstantVAM = ( CurrentSimulationPos.Alt - PrevAlt ) / ( FrameTimeMS / ( 3600.f * 1000.0f ) );
		if ( PrevAlt != FLT_MAX && InstantVAM > 100.0f ) {
			VAM = 0.99f * VAM + 0.01f * InstantVAM;
		}
		PrevAlt = CurrentSimulationPos.Alt;

		std::this_thread::sleep_for( std::chrono::milliseconds( FrameTimeMS ) );
	}

	ctxout.restore(console_cleanup_options::restore_attibutes);
}

int main( int argc, char * argv[] )
{
	WAVECONTROL_LOG( "W A V E C O N T R O L    D E M O\n" );

	//if ( WaveTest( argc, argv ) ) {
	//	return 0;
	//}

	WaveControl w;

	/*w.ScanStart( 60 );
	std::this_thread::sleep_for( std::chrono::milliseconds( 5000 ) );
	w.ListPeripherals();
	w.ChooseWheelSize( "700c x 23mm" );

	w.ChooseDeviceForUsage( WAVECONTROL_DEVICE_HR, "dc:f4:6b:f4:c0:47" );
	w.ChooseDeviceForUsage( WAVECONTROL_DEVICE_CADENCE, "e8:b7:91:03:67:d2" );
	w.ChooseDeviceForUsage( WAVECONTROL_DEVICE_POWER, "cb:7f:a9:d6:a7:4a" );
	w.ChooseDeviceForUsage( WAVECONTROL_DEVICE_TRAINER, "cb:7f:a9:d6:a7:4a" );
	std::this_thread::sleep_for( std::chrono::milliseconds( 10000 ) );*/
	system( "cls" );

	FullRouteSimulationDemo( w );
}