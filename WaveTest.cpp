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

#define CATCH_CONFIG_RUNNER
#include "Include/catch.hpp"

TEST_CASE( "Simple GPX Route Load", "[WaveGPX]" )
{
	WaveRouteState WRS;
	FWaveGPXRoute Route;
	WRS.LoadRouteGPX( Route, "TestFiles/HawkHill.gpx" );
	REQUIRE( Route.Name.size() > 0 );
	REQUIRE( Route.Points.size() > 0 );
	REQUIRE( Route.SourceFile.size() > 0 );
}

TEST_CASE( "Route Find Point At Dist", "[WaveGPX]" )
{
	WaveRouteState WRS;
	FWaveGPXRoute Route;
	WRS.LoadRouteGPX( Route, "TestFiles/HawkHill.gpx" );

	REQUIRE( Route.Name.size() > 0 );
	REQUIRE( Route.Points.size() > 0 );
	REQUIRE( Route.SourceFile.size() > 0 );

	REQUIRE( WaveRouteUtil_FindPointAtDist( Route, -1.0f ) == 0 );
	REQUIRE( WaveRouteUtil_FindPointAtDist( Route, 0.0f ) == 0 );
	REQUIRE( WaveRouteUtil_FindPointAtDist( Route, 10.0f ) == 0 );
	REQUIRE( WaveRouteUtil_FindPointAtDist( Route, 20.0f ) == 1 );
	REQUIRE( WaveRouteUtil_FindPointAtDist( Route, 40.0f ) == 3 );
	REQUIRE( WaveRouteUtil_FindPointAtDist( Route, 80.0f ) == 8 );
	REQUIRE( WaveRouteUtil_FindPointAtDist( Route, 160.0f ) == 14 );
	REQUIRE( WaveRouteUtil_FindPointAtDist( Route, 9999999.0f ) == Route.Points.size() - 1 );
}

TEST_CASE( "Route Get Point At Dist", "[WaveGPX]" )
{
	WaveRouteState WRS;
	FWaveGPXRoute Route;
	WRS.LoadRouteGPX( Route, "TestFiles/HawkHill.gpx" );

	REQUIRE( Route.Name.size() > 0 );
	REQUIRE( Route.Points.size() > 0 );
	REQUIRE( Route.SourceFile.size() > 0 );

	for ( float x = 0; x < 100.0f; x += 1.0f ) {
		auto P = WaveRouteUtil_FindENUPosAtDist( Route, x );
		auto G = WaveRouteUtil_FindGradePosAtDist( Route, x );
		auto I = WaveRouteUtil_FindPointAtDist( Route, x );
		WAVECONTROL_LOG( "Route[%.2f] Idx[%d] = LLA { %.2f %.2f %.2f }, Grade %.1f%%\n\n", x, I, P.Lat, P.Lon, P.Alt, G );
	}
}

TEST_CASE( "Basic Simulation", "[WaveSim]" )
{
	WaveSimulation Sim;
	for ( int i = 0; i < 512; i++ ) {
		Sim.Update( 1.0f / 5.0f );
	}
}

TEST_CASE( "Gradient Simulation", "[WaveSim]" )
{
	WaveSimulation Sim;
	Sim.Grade = 4.0f;
	for ( int i = 0; i < 128; i++ ) {
		Sim.Update( 1.0f / 5.0f );
	}
}

TEST_CASE( "Gradient Simulation 2", "[WaveSim]" )
{
	WaveSimulation Sim;
	Sim.Grade = -4.0f;
	for ( int i = 0; i < 128; i++ ) {
		Sim.Update( 1.0f / 5.0f );
	}
}

bool WaveTest( int argc, char * argv[] )
{
	Catch::Session().run( argc, argv );
	return true;
}