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

#define WAVESIM_SUBSTEPS 8
#define WAVESIM_MAGIC_ID 0x00670233

// Some useful tyre values to plug in.
#define WAVESIM_TIRE_CRR_EXAMPLE_ROAD_FAST			0.00267f
#define WAVESIM_TIRE_CRR_EXAMPLE_ROAD_MID			0.00408f
#define WAVESIM_TIRE_CRR_EXAMPLE_ROAD_SLOW			0.00677f
#define WAVESIM_TIRE_CRR_EXAMPLE_HYBRID_FAST		0.00594f
#define WAVESIM_TIRE_CRR_EXAMPLE_HYBRID_SLOW		0.00881f
#define WAVESIM_TIRE_CRR_EXAMPLE_MTB_FAST			0.00710f
#define WAVESIM_TIRE_CRR_EXAMPLE_MTB_SLOW			0.01067f

// Some useful rider position values
#define WAVESIM_RIDER_FRONTALAREA_HOODS 0.42f
#define WAVESIM_RIDER_FRONTALAREA_FLAT 0.56f
#define WAVESIM_RIDER_FRONTALAREA_DROPS 0.38f
#define WAVESIM_RIDER_FRONTALAREA_AERO 0.3f

// Some useful cycling drag coefficient values.
#define WAVESIM_DRAG_AERO 0.55f
#define WAVESIM_DRAG_ROAD 0.72f
#define WAVESIM_DRAG_MTB 0.8f
#define WAVESIM_DRAG_UPRIGHT 1.1f

// Simplify into a 2D model, since this seems to be the most robust solution to modelling cycling.
// We can add hacks to make cornering look and feel more realistic.
//
class WaveSimulation
{
	// These are outputs.
	float Accel = 0.0f;
	float Velocity = 0.0f;
	float Position = 0.0f;

public:
	// These are inputs. Sync this with WaveControlDLLImport.h!
	float RiderPower = 170.0f; // Watts
	float RiderWeight = 83.0f; // KG
	float BikeWeight = 9.08f + 0.34f; // KG
	float TireCrr = WAVESIM_TIRE_CRR_EXAMPLE_ROAD_SLOW; // KG
	float BikeDragCoeff = WAVESIM_DRAG_ROAD; // KG
	float RiderFrontalArea = WAVESIM_RIDER_FRONTALAREA_HOODS; // KG
	float Grade = 0.0f; // Percent
	float Altitude = 100.0f; // M
	float DrivetrainEfficiency = 0.95f;
	float RiderFTP = 170.0f; // Unused for simulation but ay be useful to UI.
	uint32_t MagicID = WAVESIM_MAGIC_ID;

protected:
	void UpdateSubstep( float DeltaTime );

public:
	void Update( float DeltaTime );

	// In m
	inline float GetPosition()
	{
		return this->Position;
	}

	// In miles
	inline float GetPositionMiles()
	{
		return this->Position * 0.000621371f;
	}

	// In m / s.
	inline float GetSpeed()
	{
		return this->Velocity;
	}

	// In miles / hr.
	inline float GetSpeedMPH()
	{
		return this->Velocity * 2.23694f;
	}

	inline void ResetRouteState()
	{
		this->Accel = 0.0f;
		this->Velocity = 0.0f;
		this->Position = 0.0f;
	}

	int GetPowerZone();
};