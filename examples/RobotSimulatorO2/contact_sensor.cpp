#include "contact_sensor.h"

#include <iostream>

void ContactSensor::Setup(const int& body_a, const int& body_b, const int& link_a, const int& link_b)
{
	// setup contact arguments
	contact_args_.m_bodyUniqueIdA = body_a;
	contact_args_.m_bodyUniqueIdB = body_b;
	contact_args_.m_linkIndexA = link_a;
	contact_args_.m_linkIndexB = link_b;
}

void ContactSensor::Wrench(class b3RobotSimulatorClientAPI_NoGUI* sim, double force_torque[], const bool& filter_enabled)
{
	b3ContactInformation contact_info;

	// get contact information
	sim->getContactPoints(contact_args_, &contact_info);

	// run algorithm to find forces and torques only if there is a contact point
	if (contact_info.m_numContactPoints > 0)
	{
		// b3LinkState link_state;
		// sim->getLinkState(contact_args_.m_bodyUniqueIdA, contact_args_.m_linkIndexA, 1, 1, &link_state);
		btVector3 position;
		btQuaternion orientation;
		sim->getBasePositionAndOrientation(contact_args_.m_bodyUniqueIdA, position, orientation);

		// std::cout
		// 	<< link_state.m_worldLinkFramePosition[0] << "," << link_state.m_worldLinkFramePosition[1] << "," << link_state.m_worldLinkFramePosition[2] << std::endl;
		// std::cout << position[0] << "," << position[1] << "," << position[2] << std::endl;

		double force[6] = {0, 0, 0, 0, 0, 0};
		for (unsigned int i = 0; i < contact_info.m_numContactPoints; i++)
		{
			// get normal force vectors
			double fi[3];
			double normal_force = contact_info.m_contactPointData[i].m_normalForce;
			for (unsigned int k = 0; k < 3; k++)
			{
				fi[k] = -normal_force * contact_info.m_contactPointData[i].m_contactNormalOnBInWS[k];
			}

			// get total forces
			for (unsigned int k = 0; k < 3; k++)
			{
				force[k] += fi[k];
			}

			// get torques
			// compute r
			double r[3];
			for (unsigned int k = 0; k < 3; k++)
			{
				r[k] = contact_info.m_contactPointData[i].m_positionOnAInWS[k] - position[k];
			}

			// compute rxF
			force[3] += r[1] * fi[2] - r[2] * fi[1];
			force[4] += -(r[0] * fi[2] - r[2] * fi[0]);
			force[5] += r[0] * fi[1] - r[1] * fi[0];
		}

		for (unsigned int i = 0; i < 6; i++)
		{
			if (filter_enabled)
			{
				force_torque[i] = force[i];
			}
			else
			{
				force_torque[i] = filtered_values_[i].Filter(force[i]);
			}
		}
	}
	else
	{
		for (unsigned int i = 0; i < 6; i++)
		{
			force_torque[i] = 0;
		}
	}
}