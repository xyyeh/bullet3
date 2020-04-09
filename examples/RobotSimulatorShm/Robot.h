#ifndef LWR_SIMULATION_SETUP_H
#define LWR_SIMULATION_SETUP_H

#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"

#include <string>

class Robot
{
	struct RobotInternalData* m_data;
	void ResetPose(class b3RobotSimulatorClientAPI_NoGUI* sim);

public:
	Robot();
	virtual ~Robot();

	unsigned int Dof() const;
	int Setup(class b3RobotSimulatorClientAPI_NoGUI* sim, const std::string& urdf_path, const class btVector3& startPos = btVector3(0, 0, 0), const class btQuaternion& startOrn = btQuaternion(0, 0, 0, 1));

	void SetDesiredQ(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& jointIdx, const double& desiredAngle, const double& maxTorque = 300, const double& kp = 400, const double& kd = 40);
	void SetDesiredTau(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& jointIdx, const double& desiredTorque);

	void JointState(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& jointIdx, double& q, double& dq);
};
#endif  //LWR_SIMULATION_SETUP_H
