#include "robot.h"
#include "b3RobotSimulatorClientAPI_NoGUI.h"
#include "Bullet3Common/b3HashMap.h"

#include <iostream>
#include <algorithm>
#include <string>
#include <map>

struct RobotInternalData
{
	int robot_id;
	unsigned int robot_loaded_dof;
	unsigned int robot_active_dof;

	RobotInternalData()
		: robot_id(-1), robot_loaded_dof(0), robot_active_dof(0)
	{
	}

	b3HashMap<b3HashString, int> joint_name_to_id;

	std::map<unsigned int, unsigned int> joint_active_id_to_id;
};

Robot::Robot()
{
	data_ = new RobotInternalData();
}

Robot::~Robot()
{
	delete data_;
}

unsigned int Robot::LoadedDof() const
{
	return data_->robot_loaded_dof;
}

unsigned int Robot::ActiveDof() const
{
	return data_->robot_active_dof;
}

void Robot::SetDesiredQ(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& joint_idx, const double& desiredAngle, const double& maxTorque, const double& kp, const double& kd)
{
	b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_POSITION_VELOCITY_PD);
	controlArgs.m_maxTorqueValue = maxTorque;
	controlArgs.m_kd = kd;
	controlArgs.m_kp = kp;
	controlArgs.m_targetPosition = desiredAngle;
	sim->setJointMotorControl(data_->robot_id, data_->joint_active_id_to_id.at(joint_idx), controlArgs);
}

void Robot::SetDesiredTau(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& joint_idx, const double& tau_d)
{
	b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_TORQUE);
	controlArgs.m_maxTorqueValue = tau_d;
	sim->setJointMotorControl(data_->robot_id, data_->joint_active_id_to_id.at(joint_idx), controlArgs);
}

void Robot::SetDesiredTaus(class b3RobotSimulatorClientAPI_NoGUI* sim, const std::vector<double>& tau_d)
{
	for (unsigned int i = 0; i < data_->joint_active_id_to_id.size(); i++)
	{
		SetDesiredTau(sim, i, tau_d[i]);
	}
}

void Robot::JointState(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& joint_idx, double& q, double& dq)
{
	struct b3JointSensorState state;
	sim->getJointState(data_->robot_id, data_->joint_active_id_to_id.at(joint_idx), &state);
	q = state.m_jointPosition;
	dq = state.m_jointVelocity;
}

void Robot::JointStates(class b3RobotSimulatorClientAPI_NoGUI* sim, std::vector<double>& q, std::vector<double>& dq)
{
	for (unsigned int i = 0; i < data_->joint_active_id_to_id.size(); i++)
	{
		JointState(sim, i, q[i], dq[i]);
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
}

int Robot::Setup(class b3RobotSimulatorClientAPI_NoGUI* sim, const std::string& urdf_path, const btVector3& start_pos, const btQuaternion& start_ori)
{
	b3RobotSimulatorLoadUrdfFileArgs args;
	args.m_startPosition = start_pos;
	args.m_startOrientation = start_ori;
	args.m_forceOverrideFixedBase = true;

	// load urdf
	data_->robot_id = sim->loadURDF(urdf_path, args);
	data_->robot_loaded_dof = sim->getNumJoints(data_->robot_id);

	// lambda to find a substring ignoring case
	auto FindIgnoreCase = [](const std::string& full_string, const std::string& to_find) {
		auto it = std::search(
			full_string.begin(), full_string.end(),
			to_find.begin(), to_find.end(),
			[](char ch1, char ch2) { return std::toupper(ch1) == std::toupper(ch2); });
		return (it != full_string.end());
	};

	// search for active joints
	data_->robot_active_dof = 0;
	for (int i = 0; i < data_->robot_loaded_dof; i++)
	{
		b3JointInfo jointInfo;
		sim->getJointInfo(data_->robot_id, i, &jointInfo);
		if (jointInfo.m_jointName[0])
		{
			std::string full_string(jointInfo.m_jointName);
			std::string sub_string = "fixed";

			if (!FindIgnoreCase(full_string, sub_string))
			{
				// map dofs
				std::cout << "joint " << i << " maps to " << jointInfo.m_jointName << std::endl;
				data_->joint_name_to_id.insert(jointInfo.m_jointName, i);
				data_->joint_active_id_to_id.insert(std::make_pair(data_->robot_active_dof, i));

				// increment active dofs
				data_->robot_active_dof++;
			}
		}
	}

	// for (auto const& p : data_->joint_active_id_to_id)
	// {
	// 	std::cout << "{" << p.first << " : " << p.second << "}" << std::endl;
	// }

	ResetPose(sim);

	return data_->robot_id;
}
