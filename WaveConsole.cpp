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

void FullRouteSimulationDemo( WaveControl& w )
{
	using namespace conmanip;
	console_out_context ctxout;
	console_out conout(ctxout);

	conout.settitle( "W A V E C O N T R O L    D E M O" );

	WaveSimulation Sim;
	Sim.RiderWeight = 83.8f;
	Sim.RiderPower = 0.0f;
	Sim.RiderFTP = 196.0f;

	WaveGPX WRS;
	FWaveGPXRoute Route;
	WRS.LoadRouteGPX( Route, "TestFiles/MachsChowMein.gpx" );

	auto SensorReadState = w.GetSensorReadState();
	auto SensorWriteState = w.GetSensorWriteState();
	SensorWriteState->TotalWeight = Sim.RiderWeight + Sim.BikeWeight;
	SensorWriteState->RollingResistance = Sim.TireCrr;

	int FrameIdx = INT_MAX, RecordFrameIdx = INT_MAX;
	float VAM = 0.0f, PrevAlt = FLT_MAX;
	int FrameTimeMS = 33;

	FWaveGPXRecord Record;
	WRS.RecordStart( Record, Route );

	static console_text_colors PowerZoneColour[] = {
		console_text_colors::light_white,
		console_text_colors::light_blue,
		console_text_colors::light_green,
		console_text_colors::light_yellow,
		console_text_colors::light_red,
		console_text_colors::light_magenta
	};

	while ( true ) {
		Sim.RiderPower = SensorReadState->Power;
		// Sim.RiderPower = 4000.0f;
		Sim.Update( FrameTimeMS / 1000.0f );
		auto CurrentSimulationPos = WaveRouteUtil_FindENUPosAtDist( Route, Sim.GetPosition() );
		auto Gradient = WaveRouteUtil_FindGradePosAtDist( Route, Sim.GetPosition() );
		
		Sim.Grade = Gradient;
		Sim.Altitude = CurrentSimulationPos.Alt;

		// Use the drop bar when descending aggressively.
		Sim.RiderFrontalArea = Gradient <= -3.0f ? WAVESIM_RIDER_FRONTALAREA_DROPS : WAVESIM_RIDER_FRONTALAREA_HOODS;

		// --------------- Draw Stats ------------------------

		std::cout << settextcolor( PowerZoneColour[ Sim.GetPowerZone() - 1 ] );
		std::cout << setposx( 1 ) << setposy( 8 ); printf( ">> Power %.0f W ( %.1f W / kg )       ", Sim.RiderPower, Sim.RiderPower / Sim.RiderWeight );
		
		std::cout << settextcolor( console_text_colors::light_white );
		std::cout << setposx( 1 ) << setposy( 9 ); printf( ">> HR %d BPM        ", SensorReadState->HR_BPM );
		std::cout << setposx( 1 ) << setposy( 10 ); printf( ">> Cadence %.0f RPM        ", SensorReadState->Cadence );

		std::cout << setposx( 1 ) << setposy( 13 ); printf( ">> Speed %.1f Km/hr ( %.1f MPH )        ", Sim.GetSpeed() * 3.6, Sim.GetSpeedMPH() );
		{
			if ( Gradient > 9.0f ) std::cout << settextcolor( console_text_colors::light_magenta );
			else if ( Gradient > 7.0f ) std::cout << settextcolor( console_text_colors::light_red );
			else if ( Gradient > 4.5f ) std::cout << settextcolor( console_text_colors::light_yellow );
			else if ( Gradient > 2.5f ) std::cout << settextcolor( console_text_colors::light_blue );
			else std::cout << settextcolor( console_text_colors::light_white );
		}
		std::cout << setposx( 1 ) << setposy( 14 ); printf( ">> Gradient %.1f %%        ", Gradient );
		std::cout << settextcolor( console_text_colors::light_white );
		std::cout << setposx( 1 ) << setposy( 15 ); printf( ">> Dist %.2f/%.2f Km ( %.1f %% )        ", Sim.GetPosition() / 1000.0f, Route.Stat_Length / 1000.0f, ( Sim.GetPosition() * 100.0f ) /  Route.Stat_Length );
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

		static char ElevMapBuffer[11][81];
		for ( int i = 0; i < 11; i++ ) {
			for ( int j = 0; j < 80; j++ ) {
				ElevMapBuffer[i][j] = ' ';
			}
			ElevMapBuffer[i][80] = '\0';
		}
		{
			int CurrentPos = ( Sim.GetPosition() / Route.Stat_Length ) * 80.0f;
			if ( CurrentPos >= 0 && CurrentPos < 80 ) {
				for ( int i = 0; i < 11; i++ ) {
					ElevMapBuffer[i][CurrentPos] = '|';
				}
			}
		}
		for ( int i = 0; i < 80; i++ ) {
			float Dist = ( i / 79.0f ) * Route.Stat_Length;
			
			auto MapPos = WaveRouteUtil_FindENUPosAtDist( Route, Dist );
			auto MapPosGradient = WaveRouteUtil_FindGradePosAtDist( Route, Dist, 10.0f );
			
			int MapZ = 0; float MapZFrac = 0.0f;
			float AltRange = Route.Stat_HighestAlt - Route.Stat_LowestAlt;
			if ( AltRange > 0.001f ) {
				float MapZf = ( ( MapPos.Alt - Route.Stat_LowestAlt ) / ( Route.Stat_HighestAlt - Route.Stat_LowestAlt ) ) * 10.0f;
				MapZ = ( int ) MapZf;
				MapZFrac = MapZf - MapZ;
			}
			
			if ( MapZ >= 0 && MapZ < 10 ) {
				char GradientCharSlope = '-';
				for ( int j = 0; j < MapZ; j++ ) {
					ElevMapBuffer[9 - j][i] = 219;
				}
				ElevMapBuffer[9 - MapZ][i] = MapZFrac > 0.5f ? 219 : 220;
				if ( MapPosGradient < 0.0f ) ElevMapBuffer[10][i] = '-';
				else ElevMapBuffer[10][i] = int( MapPosGradient ) > 9 ? ( 'A' + int( MapPosGradient ) - 10 ) : '0' + int( MapPosGradient );
			}
		}
		for ( int i = 0; i < 11; i++ ) {
			std::cout << setposx( 1 ) << setposy( 22 + i );
			puts( ElevMapBuffer[i] );
		}

		std::cout << setposx( 0 ) << setposy( 34 );

		// Rate limit changes to gradient to not spam too much.
		if ( FrameIdx++ > 20 ) {
			FrameIdx = 0;

			// Halve descent gradient to reduce noise.
			SensorWriteState->Gradient = ( Gradient < 0.0f ) ? ( Gradient * 0.5f ) : Gradient;
		}

		// Calculate VAM.
		float InstantVAM = ( CurrentSimulationPos.Alt - PrevAlt ) / ( FrameTimeMS / ( 3600.f * 1000.0f ) );
		if ( PrevAlt != FLT_MAX && InstantVAM > 100.0f ) {
			VAM = 0.99f * VAM + 0.01f * InstantVAM;
		}
		PrevAlt = CurrentSimulationPos.Alt;

		// Record current point.
		if ( RecordFrameIdx++ > 25 ) {
			RecordFrameIdx = 0;
			WRS.RecordAddPoint( Record, CurrentSimulationPos, std::chrono::system_clock::now(), SensorReadState->Power, SensorReadState->Cadence, SensorReadState->HR_BPM );
		}

		// Step ride when we get to the end.
		if ( Sim.GetPosition() > Route.Stat_Length || GetAsyncKeyState( VK_ESCAPE ) ) {
			break;
		}

		std::this_thread::sleep_for( std::chrono::milliseconds( FrameTimeMS ) );
	}
	ctxout.restore(console_cleanup_options::restore_attibutes);

	WAVECONTROL_LOG( "Ride finished! Congrats!!! Saving to replay TestFiles/RecordedRide.gpx\n" );
	WRS.RecordFinish( Record, "TestFiles/RecordedRide.gpx" );
}

int main( int argc, char * argv[] )
{
	WAVECONTROL_LOG( "W A V E C O N T R O L    D E M O\n" );

	//if ( WaveTest( argc, argv ) ) {
	//	return 0;
	//}

	WaveControl w;

	w.ScanStart( 140 );
	std::this_thread::sleep_for( std::chrono::milliseconds( 8000 ) );
	w.ListPeripherals();
	w.ChooseWheelSize( "700c x 23mm" );

	w.ChooseDeviceForUsage( WAVECONTROL_DEVICE_HR, "dc:f4:6b:f4:c0:47" );
	//w.ChooseDeviceForUsage( WAVECONTROL_DEVICE_CADENCE, "e8:b7:91:03:67:d2" );
	w.ChooseDeviceForUsage( WAVECONTROL_DEVICE_CADENCE, "f7:03:39:de:3d:38" );
	w.ChooseDeviceForUsage( WAVECONTROL_DEVICE_POWER, "cb:7f:a9:d6:a7:4a" );
	w.ChooseDeviceForUsage( WAVECONTROL_DEVICE_TRAINER, "cb:7f:a9:d6:a7:4a" );
	std::this_thread::sleep_for( std::chrono::milliseconds( 10000 ) );
	system( "cls" );

	FullRouteSimulationDemo( w );
}