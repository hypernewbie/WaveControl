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
};

class WaveRouteState
{
	std::vector< FWaveGPXRoute > LoadedRoutes;
	FWaveGPXRoute RideRoute;

protected:
	void CalcRouteStats( FWaveGPXRoute& Route );

public:
	WaveRouteState();
	virtual ~WaveRouteState();

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
};

int WaveRouteUtil_FindPointAtDist( const FWaveGPXRoute& Route, float Dist );

// Uses simple linear interpolation, which is good for demo apps and testing purposes.
//
FWaveGPXPoint WaveRouteUtil_FindENUPosAtDist( const FWaveGPXRoute& Route, float Dist );

// Uses simple central difference with linear interpolation, which is good for demo apps and testing purposes.
float WaveRouteUtil_FindGradePosAtDist( const FWaveGPXRoute& Route, float Dist, float Smoothness = 8.5f );