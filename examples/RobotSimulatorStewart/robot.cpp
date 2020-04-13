#include "robot.h"
#include "b3RobotSimulatorClientAPI_NoGUI.h"
#include "Bullet3Common/b3HashMap.h"

#include <iostream>

struct RobotInternalData
{
	int robot_id;
	uint32_t robot_dof;

	RobotInternalData()
		: robot_id(-1), robot_dof(0)
	{
	}

	b3HashMap<b3HashString, int> joint_name_to_id;
};

Robot::Robot()
{
	data_ = new RobotInternalData();
}

Robot::~Robot()
{
	delete data_;
}

unsigned int Robot::Dof() const
{
	return data_->robot_dof;
}

void Robot::SetDesiredQ(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& jointIdx, const double& desiredAngle, const double& maxTorque, const double& kp, const double& kd)
{
	b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_POSITION_VELOCITY_PD);
	controlArgs.m_maxTorqueValue = maxTorque;
	controlArgs.m_kd = kd;
	controlArgs.m_kp = kp;
	controlArgs.m_targetPosition = desiredAngle;
	sim->setJointMotorControl(data_->robot_id, jointIdx, controlArgs);
}

void Robot::SetDesiredTau(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& jointIdx, const double& desiredTorque)
{
	b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_TORQUE);
	controlArgs.m_maxTorqueValue = desiredTorque;
	sim->setJointMotorControl(data_->robot_id, jointIdx, controlArgs);

	struct b3JointSensorState state;
	sim->getJointState(data_->robot_id, jointIdx, &state);
	// std::cout << controlArgs.m_maxTorqueValue << std::endl;
}

void Robot::SetDesiredTaus(class b3RobotSimulatorClientAPI_NoGUI* sim, double desiredTorque[])
{
	const unsigned int n_legs = 6;
	int joint_idx[n_legs] = {*data_->joint_name_to_id["joint1"], *data_->joint_name_to_id["joint2"], *data_->joint_name_to_id["joint3"],
							 *data_->joint_name_to_id["joint4"], *data_->joint_name_to_id["joint5"], *data_->joint_name_to_id["joint6"]};

	for (unsigned int i = 0; i < n_legs; i++)
	{
		SetDesiredTau(sim, joint_idx[i], desiredTorque[i]);
	}
}

void Robot::JointState(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& jointIdx, double& q, double& dq)
{
	struct b3JointSensorState state;
	sim->getJointState(data_->robot_id, jointIdx, &state);
	q = state.m_jointPosition;
	dq = state.m_jointVelocity;
}

void Robot::JointStates(class b3RobotSimulatorClientAPI_NoGUI* sim, double q[], double dq[])
{
	struct b3JointSensorState state;
	const unsigned int n_legs = 6;
	int joint_idx[n_legs] = {*data_->joint_name_to_id["joint1"], *data_->joint_name_to_id["joint2"], *data_->joint_name_to_id["joint3"],
							 *data_->joint_name_to_id["joint4"], *data_->joint_name_to_id["joint5"], *data_->joint_name_to_id["joint6"]};
	for (unsigned int i = 0; i < n_legs; i++)
	{
		JointState(sim, joint_idx[i], q[i], dq[i]);
	}
}

void Robot::ResetPose(class b3RobotSimulatorClientAPI_NoGUI* sim)
{
	// release all motors
	int numJoints = sim->getNumJoints(data_->robot_id);
	for (int i = 0; i < numJoints; i++)
	{
		b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
		controlArgs.m_maxTorqueValue = 0;
		sim->setJointMotorControl(data_->robot_id, i, controlArgs);
	}

	auto MakeLegJointInfo = [](double pos_x, double pos_y) {
		b3JointInfo joint_info;
		joint_info.m_jointType = ePoint2PointType;
		joint_info.m_parentFrame[0] = 0;
		joint_info.m_parentFrame[1] = 0;
		joint_info.m_parentFrame[2] = 0;
		joint_info.m_childFrame[0] = pos_x;
		joint_info.m_childFrame[1] = pos_y;
		joint_info.m_childFrame[2] = 0;
		return joint_info;
	};

	// leg constraints
	const double R = 0.4;
	const double leg_sym_angle = 11.5;
	const double leg_sym_angle_2 = leg_sym_angle * 2;
	const double base_angle = B3_RADS_PER_DEG * (30 + leg_sym_angle);
	const unsigned int n_legs = 6;
	double angles[n_legs] = {
		0, 120 - leg_sym_angle_2, 120, 240 - leg_sym_angle_2, 240, 360 - leg_sym_angle_2};
	for (unsigned int i = 0; i < n_legs; i++)
	{
		angles[i] *= B3_RADS_PER_DEG;
	}
	int joint_idx[n_legs] = {*data_->joint_name_to_id["joint1"], *data_->joint_name_to_id["joint2"], *data_->joint_name_to_id["joint3"],
							 *data_->joint_name_to_id["joint4"], *data_->joint_name_to_id["joint5"], *data_->joint_name_to_id["joint6"]};

	// setup constraints
	b3JointInfo jointInfo[n_legs];

	for (unsigned int i = 0; i < 6; i++)
	{
		jointInfo[i] = MakeLegJointInfo(R * cos(base_angle + angles[i]), R * sin(base_angle + angles[i]));
		sim->resetJointState(data_->robot_id, joint_idx[i], -0.555272);
		sim->createConstraint(data_->robot_id, joint_idx[i], 0, -1, &jointInfo[i]);
	}
}

int Robot::Setup(class b3RobotSimulatorClientAPI_NoGUI* sim, const std::string& urdf_path, const btVector3& startPos, const btQuaternion& startOrn)
{
	b3RobotSimulatorLoadUrdfFileArgs args;
	args.m_startPosition = startPos;
	args.m_startOrientation = startOrn;

	data_->robot_id = sim->loadURDF(urdf_path, args);

	data_->robot_dof = sim->getNumJoints(data_->robot_id);
	for (int i = 0; i < data_->robot_dof; i++)
	{
		b3JointInfo info;
		sim->getJointInfo(data_->robot_id, i, &info);
		if (info.m_jointName[0])
		{
			std::cout << "joint " << i << " maps to " << info.m_jointName << std::endl;
			data_->joint_name_to_id.insert(info.m_jointName, i);
		}
	}

	ResetPose(sim);

	return data_->robot_id;
}
