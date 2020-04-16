#include "b3RobotSimulatorClientAPI.h"

#include <string.h>
#include <iostream>
#include <assert.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <vector>
#include <fstream>

#include "robot.h"
#include "shm_sem.h"

const unsigned int kSecToNanosec = 1e9;

#define DOF (2)
struct RobotData
{
	double tau[DOF];
	double q[DOF];
	double dq[DOF];
};

int main(int argc, char* argv[])
{
	b3RobotSimulatorClientAPI* sim = new b3RobotSimulatorClientAPI();
	bool isConnected = sim->connect(eCONNECT_GUI);

	if (!isConnected)
	{
		std::cout << "Cannot connect" << std::endl;
		return -1;
	}

	// shared memory
	ShmSemaphore shared_memory("/physics_shm");
	shared_memory.Create(sizeof(struct RobotData));
	shared_memory.Attach();
	RobotData* ptr = static_cast<RobotData*>(shared_memory.Data());

	// setup visualizer
	sim->configureDebugVisualizer(COV_ENABLE_GUI, 0);
	sim->configureDebugVisualizer(COV_ENABLE_SHADOWS, 0);

	// setup timeout
	sim->setTimeOut(10);

	// syncBodies is only needed when connecting to an existing physics server that has already some bodies
	sim->syncBodies();

	// setup fixed time step
	btScalar kFixedTimeStep = 1.0 / 1000.0;
	sim->setTimeStep(kFixedTimeStep);

	// setup gravity
	sim->setGravity(btVector3(0, 0, 0));

	// setup world
	// int plane_id = sim->loadURDF("plane.urdf");

	// setup robot
	Robot arm;
	int robot_id = arm.Setup(sim, "./panda/two_joints.urdf", btVector3(0, 0, 0.0000));

	// disable collisions
	for (int i = -1; i < arm.Dof(); i++)
	{
		sim->setCollisionFilterGroupMask(robot_id, i, 0, 0);
	}
	// sim->setCollisionFilterGroupMask(plane_id, -1, 0, 0);

	// setup dynamics
	struct b3RobotSimulatorChangeDynamicsArgs dynArgs;
	dynArgs.m_linearDamping = 0;
	dynArgs.m_angularDamping = 0;
	sim->changeDynamics(robot_id, 0, dynArgs);

	// frame
	b3RobotSimulatorAddUserDebugLineArgs frame_args;
	btVector3 start(0, 0, 0);
	btVector3 end_x(1, 0, 0);
	btVector3 end_z(0, 0, 1);
	frame_args.m_lineWidth = 5;
	frame_args.m_colorRGB[0] = 1;
	frame_args.m_colorRGB[1] = 0;
	frame_args.m_colorRGB[2] = 0;
	int id_x = sim->addUserDebugLine(start, end_x, frame_args);
	frame_args.m_colorRGB[0] = 0;
	frame_args.m_colorRGB[1] = 0;
	frame_args.m_colorRGB[2] = 1;
	int id_z = sim->addUserDebugLine(start, end_z, frame_args);

	// simulation paramters
	sim->setRealTimeSimulation(false);

	// start loop
	double sim_time = 0;
	double prev_time = 0;

	// feedback
	std::vector<double> theta_d(arm.Dof());
	std::vector<double> q(arm.Dof());
	std::vector<double> dq(arm.Dof());
	std::vector<double> theta(arm.Dof());
	std::vector<double> dtheta(arm.Dof());
	std::vector<double> tau_cmd(arm.Dof());

	theta_d[0] = 0;

	std::cout << "Total dof = " << arm.Dof() << std::endl;

	// file
	std::ofstream fs("data.csv");

	// start looping after one second
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	t.tv_sec++;
	while (1)
	{
		if (!sim->canSubmitCommand())
		{
			std::cout << "//////////////////// WARNING ////////////////////" << std::endl;
		}

		sim_time += kFixedTimeStep;

		if (sim_time >= 2)
		{
			theta_d[0] = 1;
		}

		if (sim_time < 10)
		{
			fs << theta_d[0] << "," << theta[0] << "," << q[0] << std::endl;
		}

		// feedback ready
		for (uint32_t i = 0; i < arm.Dof(); i++)
		{
			theta[i] = arm.MotorPosition(i);
			dtheta[i] = arm.MotorVelocity(i);
			q[i] = arm.LinkPosition(i);
			dq[i] = arm.LinkVelocity(i);
		}

		// shared_memory.Lock();
		// for (uint32_t i = 0; i < arm.Dof(); i++)
		// {
		// 	ptr->q[i] = q[i];
		// 	ptr->dq[i] = dq[i];
		// 	tau_cmd[i] = ptr->tau[i];
		// }
		// shared_memory.Unlock();

		// compute control and step dynamics
		for (uint32_t i = 0; i < arm.Dof(); i++)
		{
			tau_cmd[i] = 1000 * (theta_d[i] - theta[i]) + 0 * (0 - dtheta[i]);
			arm.SetJointTorque(sim, i, tau_cmd[i]);
			arm.StepJointDynamics(i, kFixedTimeStep);
		}

		// link torque
		for (uint32_t i = 0; i < arm.Dof(); i++)
		{
			arm.SetDesiredTau(sim, i, arm.LinkTorque(i));
		}

		// step simulation
		sim->stepSimulation();

		if ((sim_time - prev_time) > 1)
		{
			prev_time = sim_time;
			printf("sim time = %f\n", sim_time);
			// for (int i = 0; i < 3; i++)
			// {

			// }
			// printf("\n");
			// printf("%.3f\n", state.m_jointPosition);
		}

		// calculate next shot
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
		t.tv_nsec += (kFixedTimeStep * kSecToNanosec);
		while (t.tv_nsec >= kSecToNanosec)
		{
			t.tv_nsec -= kSecToNanosec;
			t.tv_sec++;
		}
	}

	fs.close();

	std::cout << "Disconnect sim" << std::endl;

	sim->disconnect();

	std::cout << "Delete sim and exit" << std::endl;
	delete sim;
}
