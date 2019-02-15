/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


#ifndef __D_VEHICLE_SOLVER_H__
#define __D_VEHICLE_SOLVER_H__

#include "dStdafxVehicle.h"

class dVehicleChassis;

class dVehicleSolver: public dAnimAcyclicSolver
{
	public:
	DVEHICLE_API dVehicleSolver() ;
	DVEHICLE_API virtual ~dVehicleSolver();
	DVEHICLE_API void Finalize(dVehicleChassis* const vehicle);
};
#endif 
