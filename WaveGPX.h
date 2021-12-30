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

#pragma once

#include <string>
#include <vector>
#include <chrono>

#define WaveGPX_MAGIC_ID 0xf20ae21

struct FWaveGPXPoint
{
	double Lat = 0.0f;
	double Lon = 0.0f;
	double Alt = 0.0f;

	double East = 0.0f;
	double North = 0.0f;
	double Up = 0.0f;

	double Dist = 0.0f;
};

struct FWaveGPXRoute
{
	std::string Name;
	std::string Description;
	std::string Author;
	std::string SourceFile;

	std::vector< FWaveGPXPoint > Points;

	float Stat_Elev = 0.0f;
	float Stat_Length = 1.0f;
	float Stat_HillinessRating = 0.0f;
	float Stat_DifficultyScore = 0.0f;
	float Stat_HighestAlt = 0.0f;
	float Stat_LowestAlt = 0.0f;
};

struct FWaveGPXRecord
{
	FWaveGPXRoute Route;

	// These should be same size as Route::Points.
	std::vector< std::chrono::system_clock::time_point > Time;
	std::vector< float > Power;
	std::vector< float > Cadence;
	std::vector< float > HR;
};

class WaveGPX
{
	std::vector< FWaveGPXRoute > LoadedRoutes;
	FWaveGPXRoute RideRoute;

protected:
	static void CalcRouteStats( FWaveGPXRoute& Route );

public:
	WaveGPX();
	virtual ~WaveGPX();
	uint32_t MagicID = WaveGPX_MAGIC_ID;

	bool LoadRouteGPX( FWaveGPXRoute& Route, const std::string FileName );

	inline const std::vector< FWaveGPXRoute >& GetLoadedRoutes()
	{
		return LoadedRoutes;
	}

	inline void AddLoadedRoute( FWaveGPXRoute& Route )
	{
		LoadedRoutes.push_back( Route );
	}

	inline void ClearLoadedRoutes()
	{
		LoadedRoutes.clear();
	}

	void SetRideRoute( const FWaveGPXRoute& Route )
	{
		this->RideRoute = Route;
	}


	void RecordStart( FWaveGPXRecord& Record, const FWaveGPXRoute& SrcInfo );

	void RecordAddPoint( FWaveGPXRecord& Record, FWaveGPXPoint Point, std::chrono::system_clock::time_point Time = std::chrono::system_clock::now(), float Power = -1.0f, float Cadence = -1.0f, float HR = -1.0f );

	bool RecordFinish( FWaveGPXRecord& Record, const std::string FileName );

};

int WaveRouteUtil_FindPointAtDist( const FWaveGPXRoute& Route, float Dist );

// Uses simple linear interpolation, which is good for demo apps and testing purposes.
//
FWaveGPXPoint WaveRouteUtil_FindENUPosAtDist( const FWaveGPXRoute& Route, float Dist );

// Uses simple central difference with linear interpolation, which is good for demo apps and testing purposes.
float WaveRouteUtil_FindGradePosAtDist( const FWaveGPXRoute& Route, float Dist, float Smoothness = 2.5f );

void WaveRouteUtil_FillENUFromLLA( const FWaveGPXRoute& Route, FWaveGPXPoint& Point );

void WaveRouteUtil_FillLLAFromENU( const FWaveGPXRoute& Route, FWaveGPXPoint& Point );