#pragma once

#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
#include "b3RobotSimulatorClientAPI.h"

#include <string>
#include <vector>

class Butter
{
#define NZEROS 2
#define NPOLES 2
#define GAIN 1.482463775e+01

private:
	float xv[NZEROS + 1];
	float yv[NPOLES + 1];

public:
	Butter() = default;  // 100 Hz filter
	virtual ~Butter() = default;

	float Filter(const float& input)
	{
		xv[0] = xv[1];
		xv[1] = xv[2];
		xv[2] = input / GAIN;
		yv[0] = yv[1];
		yv[1] = yv[2];
		yv[2] = (xv[0] + xv[2]) + 2 * xv[1] + (-0.4128015981 * yv[0]) + (1.1429805025 * yv[1]);

		return yv[2];
	}
};

class ContactSensor
{
private:
	b3RobotSimulatorGetContactPointsArgs contact_args_;
	Butter filtered_values_[6];

public:
	ContactSensor() = default;
	virtual ~ContactSensor() = default;

	/**
   * @brief Sets up the colliding bodies, where body_a is the sensor
   * @param[in] body_a Body A (sensor) of the colliding pairs
   * @param[in] body_b Body A (sensor) of the colliding pairs
   */
	void Setup(const int& body_a, const int& body_b, const int& link_a = -1, const int& link_b = -1);

	/**
   * @brief Gets the raw or filtered wrench forces
   * @param[in] sim Simulation handler
   * @param[out] force_torque Force/torque values
   * @param[in] filter_enabled Enable filter (default to disabled)
   */
	void Wrench(class b3RobotSimulatorClientAPI_NoGUI* sim, double force_torque[], const bool& filter_enabled = false);
};