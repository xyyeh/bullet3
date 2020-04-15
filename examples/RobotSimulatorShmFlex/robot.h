#pragma once

#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
#include "flexible_joint.h"

#include <string>

class Robot
{
private:
	struct RobotInternalData* data_;
	void ResetPose(class b3RobotSimulatorClientAPI_NoGUI* sim);

	FlexibleJoint joints_[2];

public:
	Robot();
	virtual ~Robot();

	unsigned int Dof() const;
	int Setup(class b3RobotSimulatorClientAPI_NoGUI* sim, const std::string& urdf_path, const class btVector3& startPos = btVector3(0, 0, 0), const class btQuaternion& startOrn = btQuaternion(0, 0, 0, 1));

	void SetDesiredQ(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& jointIdx, const double& desiredAngle, const double& maxTorque = 300, const double& kp = 400, const double& kd = 40);
	void SetDesiredTau(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& jointIdx, const double& desiredTorque);

	void JointState(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& jointIdx, double& q, double& dq);

	double StepJointDynamics(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& jointIdx, const double& tau_cmd, double& q, double& dq);
};
