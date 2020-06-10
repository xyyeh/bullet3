#pragma once

#define DOF (20)
struct RobotInterface
{
	// command
	double tau_J_d[DOF];

	// link side measurements
	double q[DOF];
	double dq[DOF];
	double ddq[DOF];

	// flags
	unsigned char controller_to_hw;
	unsigned char hw_to_controller;

	// parameters
	unsigned char dof;
};