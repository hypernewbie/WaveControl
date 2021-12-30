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

#include "WaveControl.h"
#include "WaveSimulation.h"

#define WAVESIM_CONSTANT_G 9.8067 // M/S
#define WAVESIM_CONSTANT_T0_KELVINS 288.16 // US Standard temperature
#define WAVESIM_CONSTANT_B 0.00650 // US Standard temperatur9 e lapse rate in K/m
#define WAVESIM_CONSTANT_R 287.0f // US Standard temperature gas constant in J/kgK
#define WAVESIM_CONSTANT_MIN_VELOCITY 0.25f // US Standard temperature lapse rate in K/m
#define WAVESIM_CONSTANT_STANDARD_AIR_PRESSURE 1.225f

void WaveSimulation::UpdateSubstep( float DeltaTime )
{
	float Temperature = WAVESIM_CONSTANT_T0_KELVINS - WAVESIM_CONSTANT_B * this->Altitude;
	float AirDensity = powf(
		1.0f - ( WAVESIM_CONSTANT_B *  this->Altitude ) / WAVESIM_CONSTANT_T0_KELVINS,
		WAVESIM_CONSTANT_G / ( WAVESIM_CONSTANT_R * WAVESIM_CONSTANT_B )
	) * (
		WAVESIM_CONSTANT_T0_KELVINS /
		( WAVESIM_CONSTANT_T0_KELVINS - ( WAVESIM_CONSTANT_B *  this->Altitude ) )
	);
	AirDensity *= WAVESIM_CONSTANT_STANDARD_AIR_PRESSURE;
	
	float TotalWeight = this->RiderWeight + this->BikeWeight;
	float Gradient = this->Grade * 0.01f;

	// Calculate force.
	float VelocityScale = ( this->Velocity > WAVESIM_CONSTANT_MIN_VELOCITY ) ? this->Velocity : WAVESIM_CONSTANT_MIN_VELOCITY;
	float FAccel = ( this->DrivetrainEfficiency * this->RiderPower ) / VelocityScale;
	float FAirResistance = -0.5f * this->RiderFrontalArea * AirDensity * ( this->Velocity * this->Velocity ) * this->BikeDragCoeff;
	float FRollingResistance = -WAVESIM_CONSTANT_G * cos( atan( Gradient ) ) * TotalWeight * this->TireCrr;
	float FGravity = -WAVESIM_CONSTANT_G * sin( atan( Gradient ) ) * TotalWeight;

	// Calculate acceleration.
	this->Accel = ( FAccel + FAirResistance + FRollingResistance + FGravity ) / TotalWeight;

	// Integrate to velocity.
	this->Velocity += this->Accel * DeltaTime;

	// Prevent from going backwards. We aren't modelling for BMX bikers sorry!
	if ( this->Velocity < 0.0f ) this->Velocity = 0.0f;
	this->Position += this->Velocity * DeltaTime;
}

void WaveSimulation::Update( float DeltaTime )
{
	for ( int i = 0; i < WAVESIM_SUBSTEPS; i++ ) {
		this->UpdateSubstep( DeltaTime / WAVESIM_SUBSTEPS );
	}
	// WAVECONTROL_LOG( "Dist: %.1f M Speed: %.1f ( %.1f MPH )\n", this->Position, this->Velocity * 3.6f, this->Velocity * 2.23694 );
}

int WaveSimulation::GetPowerZone()
{
	if ( this->RiderFTP <= 0.00001f ) return 1;
	float Percentage = this->RiderPower * 100.0f / this->RiderFTP;
	if ( Percentage < 55.0f ) return 1;
	if ( Percentage < 75.0f ) return 2;
	if ( Percentage < 90.0f ) return 3;
	if ( Percentage < 105.0f ) return 4;
	if ( Percentage < 120.0f ) return 5;
	return 6;
}