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

#include "WaveGPX.h"
#include "WaveControl.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <fstream>
#include <functional>
#include <algorithm>

#include <gpx/GPX.h>
#include <gpx/Parser.h>
#include <gpx/Report.h>
#include <gpx/ReportCerr.h>
#include <geodetic_conv.hpp>

#include <Eigen/Core>
#include <Eigen/Eigen>
typedef Eigen::Vector3d WVec3;
inline WVec3 WaveGPX_PointToVec(const FWaveGPXPoint& P)
{
	return WVec3( P.East, P.North, P.Up );
}

static float WaveLerp( float A, float B, float t )
{
	return A * ( 1.0f - t ) + B * t;
}

WaveRouteState::WaveRouteState()
{
}

WaveRouteState::~WaveRouteState()
{
}

bool WaveRouteState::LoadRouteGPX( FWaveGPXRoute& Route, const std::string FileName )
{
	std::ifstream FileStream( FileName );

	if( !FileStream.is_open() ) {
		WAVECONTROL_LOG( "ERROR: Failed to open file %s!\n", FileName.c_str() );
		return false;
	}

	// Parse the GPX file format using gpxlib library.
	gpx::ReportCerr GPXError;
	gpx::Parser GPXParser( &GPXError );
	gpx::GPX* GPXRoot = GPXParser.parse( FileStream );
	if( !GPXRoot ) {
		WAVECONTROL_LOG(
			"ERROR: GPX Parse failure due to %s on line %d col %d!\n",
			GPXParser.errorText().c_str(),
			GPXParser.errorLineNumber(),
			GPXParser.errorColumnNumber()
		);
		return false;
	}

	WAVECONTROL_LOG( "GPX Version: %s\n", GPXRoot->version().getValue().c_str() );

	// Read metadata from the GPX file.
	if ( GPXRoot->metadata().hasElements() ) {
		Route.Name = GPXRoot->metadata().name().getValue();
		Route.Description = GPXRoot->metadata().desc().getValue();
		Route.Author = GPXRoot->metadata().author().name().getValue();
	} else {
		Route.Name = "";
		Route.Description = "";
		Route.Author = "Unknown";
	}
	Route.SourceFile = FileName;
	Route.Points.clear();

	// Read waypoint from the GPX file.

	auto ConvertGPXDecimalToFloat = []( gpx::Decimal& Val ) -> float
	{
		float Ret = 0.0f;
		sscanf_s( Val.getValue().c_str(), "%f", &Ret);
		return Ret;
	};

	geodetic_converter::GeodeticConverter GConverter;

	bool ReferencePointInitialised = false;
	auto AppendPointsToRoute = [&]( std::list< gpx::WPT* >& Points )
	{
		for( auto& Point : Points ) {
			FWaveGPXPoint P;

			// Convert string LLA to double.
			P.Lat = ConvertGPXDecimalToFloat( Point->lat() );
			P.Lon = ConvertGPXDecimalToFloat( Point->lon() );
			P.Alt = ConvertGPXDecimalToFloat( Point->ele() );

			// Convert LLA waypoint to ENU.
			if ( !ReferencePointInitialised ) {
				GConverter.initialiseReference( P.Lat, P.Lon, P.Alt );
				ReferencePointInitialised = true;
			}
			GConverter.geodetic2Enu( P.Lat, P.Lon, P.Alt, &P.East, &P.North, &P.Up );

			// WAVECONTROL_LOG( "Point %lf %lf %lf ( %lf %lf %lf )!\n", P.Lat, P.Lon, P.Alt, P.East, P.North, P.Up );
			Route.Points.push_back( P );
		}
	};

	auto& GPXTracks = GPXRoot->trks().list();
	for( auto& Track : GPXTracks ) {

		if ( !Route.Name.length() ) {
			// No metadata, use the first track's name as the route name.
			Route.Name = Track->name().getValue();
		}

		auto& Segments = Track->trksegs().list();
		for( auto& Segment : Segments ) {
			auto& Points = Segment->trkpts().list();
			AppendPointsToRoute( Points );
		}

		break;
	}
	FileStream.close();

	if ( !Route.Name.length() ) {
		// No metadata, or track name, use the file name as final fallback.
		Route.Name = Route.SourceFile;
	}
	WAVECONTROL_LOG( "Name: %s\n" , Route.Name.c_str() );
	WAVECONTROL_LOG( "Desc: %s\n" , Route.Description.c_str() );
	WAVECONTROL_LOG( "Author: %s\n" , Route.Author.c_str() );
	this->CalcRouteStats( Route );

	return true;
}

void WaveRouteState::CalcRouteStats( FWaveGPXRoute& Route )
{
	Route.Stat_Elev = 0.0f;
	Route.Stat_Length = 0.0f;
	Route.Stat_HillinessRating = 0.0f;
	Route.Stat_DifficultyScore = 0.0f;
	Route.Stat_HighestAlt = Route.Points.size() ? Route.Points[0].Alt : 0.0f;

	for ( int i = 1; i < Route.Points.size(); i++ ) {
		auto& PrevPoint = Route.Points[ i - 1 ];
		auto& CurrPoint = Route.Points[ i ];

		auto PrevPointVector = WaveGPX_PointToVec( PrevPoint );
		auto CurrPointVector = WaveGPX_PointToVec( CurrPoint );
		auto Dist = (CurrPointVector - PrevPointVector).norm();
		auto Elev = CurrPointVector.z() - PrevPointVector.z();
		CurrPoint.Dist = PrevPoint.Dist + Dist;

		Route.Stat_Length += ( float ) Dist;
		Route.Stat_Elev += ( Elev > 0 ) ? ( float ) Elev : 0.0f;
		Route.Stat_HighestAlt = ( CurrPoint.Alt > Route.Stat_HighestAlt ) ? CurrPoint.Alt : Route.Stat_HighestAlt;
	}
	
	// Hilliness rating is a heuristic taken from Western Wheelers club system, which I believe
	// is quite intuitive:
	//     https://westernwheelersbicycleclub.wildapricot.org/page-1374754
	//
	float LengthMiles = Route.Stat_Length * 0.000621371f;
	float ElevFeet = Route.Stat_Elev * 3.28084;
	if ( LengthMiles > 0.000001f ) {
		Route.Stat_HillinessRating = ( ElevFeet / LengthMiles ) / 25.0f;
	}

	WAVECONTROL_LOG( "Length: %.1f KM ( %.1f Miles )\n" , Route.Stat_Length / 1000.0f, LengthMiles );
	WAVECONTROL_LOG( "Elevation: %.1f M ( %.1f Feet )\n" , Route.Stat_Elev, ElevFeet );
	WAVECONTROL_LOG( "Hilliness Rating: %.1f\n" , Route.Stat_HillinessRating );
}

int WaveRouteUtil_FindPointAtDist( const FWaveGPXRoute& Route, float Dist )
{
	FWaveGPXPoint Temp;
	Temp.Dist = Dist;

	auto Itr = std::lower_bound(
		Route.Points.begin(), Route.Points.end(), Temp,
		[]( const FWaveGPXPoint& A, const FWaveGPXPoint& B ) {
			return A.Dist < B.Dist;
		}
	);
	if ( Itr == Route.Points.end() ) {
		return Route.Points.size() - 1;
	}
	if ( Itr == Route.Points.begin() ) {
		return 0;
	}
	return ( Itr - Route.Points.begin() - 1 );
}

#pragma optimize("", off)

FWaveGPXPoint WaveRouteUtil_FindENUPosAtDist( const FWaveGPXRoute& Route, float Dist )
{
	FWaveGPXPoint Point;
	
	if ( Route.Points.size() <= 0 ) {
		return Point;
	}

	auto Index = WaveRouteUtil_FindPointAtDist( Route, Dist );
	if ( Index < 0 || Index >= Route.Points.size() ) {
		Point.Dist = -1.0f;
		return Point;
	}
	
	if ( Index == Route.Points.size() - 1 || Dist <= Route.Points[Index].Dist ) {
		Point.East = Route.Points[Index].East;
		Point.North = Route.Points[Index].North;
		Point.Up = Route.Points[Index].Up;

		Point.Lat = Route.Points[Index].Lat;
		Point.Lon = Route.Points[Index].Lon;
		Point.Alt = Route.Points[Index].Alt;

		Point.Dist = Dist;
		return Point;
	}

	assert( Dist >= Route.Points[Index].Dist );
	float InterpRun = Route.Points[Index + 1].Dist - Route.Points[Index].Dist;
	float InterpX = ( Dist - Route.Points[Index].Dist ) / ( InterpRun > 0.01f ? InterpRun : 0.01f );

	Point.Lat = WaveLerp( Route.Points[Index].Lat, Route.Points[Index + 1].Lat, InterpX );
	Point.Lon = WaveLerp( Route.Points[Index].Lon, Route.Points[Index + 1].Lon, InterpX );
	Point.Alt = WaveLerp( Route.Points[Index].Alt, Route.Points[Index + 1].Alt, InterpX );

	// Convert ENU waypoint back to LLA.
	geodetic_converter::GeodeticConverter GConverter;
	GConverter.initialiseReference( Route.Points[0].Lat, Route.Points[0].Lon, Route.Points[0].Alt );
	GConverter.geodetic2Enu( Point.Lat, Point.Lon, Point.Alt, &Point.East, &Point.North, &Point.Up );

	return Point;
}

float WaveRouteUtil_FindGradePosAtDist( const FWaveGPXRoute& Route, float Dist, float Smoothness )
{
	if ( Route.Points.size() <= 0 ) {
		return 0.0f;
	}

	auto PosA = WaveRouteUtil_FindENUPosAtDist( Route, Dist - Smoothness );
	auto PosB = WaveRouteUtil_FindENUPosAtDist( Route, Dist + Smoothness );
	
	float ElevationChange = PosB.Alt - PosA.Alt;
	float HorizontalDistanceCovered = 2.0f * Smoothness;
	float Grade = ElevationChange / HorizontalDistanceCovered;

	return Grade * 100.0f;
}