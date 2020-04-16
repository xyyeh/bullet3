#include "flexible_joint.h"

void FlexibleJoint::SetParameters(const double& B, const double& K, const double& D)
{
	param_.B = B;
	param_.K = K;
	param_.D = D;
}

void FlexibleJoint::SetInputs(const double& tau_cmd, const double& q, const double& dq)
{
	dyn_.tau_cmd = tau_cmd;
	dyn_.q = q;
	dyn_.dq = dq;
}

void FlexibleJoint::StepDynamics(const double& dt)
{
	// new input
	double prev_tau_s = dyn_.tau_s;
	dyn_.tau_s = 100 * (dyn_.theta - dyn_.q) + 0.3 * (dyn_.dtheta - dyn_.dq);
	dyn_.dtau_s = (dyn_.tau_s - prev_tau_s) / dt;

	// new values
	dyn_.ddtheta = (1.0 / 0.0742) * (dyn_.tau_cmd - dyn_.tau_s);
	dyn_.dtheta = dyn_.ddtheta * dt + dyn_.dtheta;
	dyn_.theta = dyn_.dtheta * dt + dyn_.theta;
}

double FlexibleJoint::MotorPosition() const
{
	return dyn_.theta;
}

double FlexibleJoint::MotorVelocity() const
{
	return dyn_.dtheta;
}

double FlexibleJoint::LinkPosition() const
{
	return dyn_.q;
}

double FlexibleJoint::LinkVelocity() const
{
	return dyn_.dq;
}

double FlexibleJoint::LinkTorque() const
{
	return dyn_.tau_s;
}

double FlexibleJoint::LinkDtorque() const
{
	return dyn_.dtau_s;
}
