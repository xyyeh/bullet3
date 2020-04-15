#pragma once

#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"

class FlexibleJoint
{
	struct JointParameters
	{
		double B{1};
		double K{10000};
		double D{10};
	};

	struct InternalDynamics
	{
		double ddtheta{0};
		double theta{0};
		double dtheta{0};

		double q{0};
		double dq{0};

		double tau_s{0};
		double dtau_s{0};
	};

private:
	InternalDynamics dyn_;
	JointParameters param_;
	double dt_{0.001};

public:
	FlexibleJoint() = default;
	virtual ~FlexibleJoint(){};

	void SetParameters(const double& B, const double& K, const double& D);
	void SetInputs(const double& tau, const double& q, const double& dq);

	double MotorPosition() const;
	double MotorVelocity() const;
	double LinkPosition() const;
	double LinkVelocity() const;
	double LinkTorque() const;
	double LinkDtorque() const;
};
