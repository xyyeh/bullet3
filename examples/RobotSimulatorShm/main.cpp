
// #ifdef B3_USE_ROBOTSIM_GUI
#include "b3RobotSimulatorClientAPI.h"
// #else
// #include "b3RobotSimulatorClientAPI_NoGUI.h"
// #endif

// #include "../Utils/b3Clock.h"

#include <string.h>
#include <iostream>
#include <assert.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <atomic>

#define ASSERT_EQ(a, b) assert((a) == (b));
#include "Robot.h"

const unsigned int kSecToNanosec = 1e9;
const unsigned int kShmSize = 1024;
const unsigned int kShmKey = 8765;

struct RobotData
{
	double tau[7];
	double q[7];
	double dq[7];
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

	// connect shm
	RobotData* shm;

	// create segment
	int shmid;
	if ((shmid = shmget(kShmKey, kShmSize, IPC_CREAT | 0666)) < 0)
	{
		std::cerr << "shmget";
		exit(1);
	}

	// attach segment to data space
	if ((shm = (RobotData*)shmat(shmid, 0, 0)) == (RobotData*)-1)
	{
		std::cerr << "shmat";
		exit(1);
	}

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
	sim->setGravity(btVector3(0, 0, -9.81));
	// sim->setGravity(btVector3(0, 0, 0));

	// setup world
	int planeId = sim->loadURDF("plane.urdf");

	// setup robot
	Robot LWR;
	int kukaId = LWR.Setup(sim, "./panda/panda.urdf", btVector3(0, 0, 0.00001));

	// setup dynamics
	struct b3RobotSimulatorChangeDynamicsArgs dynArgs;
	dynArgs.m_linearDamping = 0;
	dynArgs.m_angularDamping = 0;
	sim->changeDynamics(kukaId, 0, dynArgs);

	// setup additional stuffs
	// int blockId = sim->loadURDF("cube.urdf");
	// sim->resetBasePositionAndOrientation(blockId, btVector3(0, 0, 3), btQuaternion(0, 0, 0, 1));

	// simulation paramters
	sim->setRealTimeSimulation(false);

	// start loop
	double sim_time = 0;
	double tau[7], q[7], dq[7];
	double q_d[7] = {0, 0.3, 0, -0.3, 0, 1.5, 0};
	double prev_time = 0;

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

		// read states
		for (uint32_t i = 0; i < LWR.Dof(); i++)
		{
			LWR.JointState(sim, i, q[i], dq[i]);
		}

		// send torques
		for (uint32_t i = 0; i < LWR.Dof(); i++)
		{
			LWR.SetDesiredTau(sim, i, shm->tau[i]);
		}

		// update shared memory
		for (uint32_t i = 0; i < LWR.Dof(); i++)
		{
			shm->q[i] = q[i];
			shm->dq[i] = dq[i];
		}

		sim->stepSimulation();

		// if ((sim_time - prev_time) > 0.25)
		// {
		// 	prev_time = sim_time;
		// 	// print time
		// 	// printf("%.3f\n", sim_time);
		// 	for (int i = 0; i < LWR.Dof(); i++)
		// 	{
		// 		printf("%.3f\t", shm->tau[i]);
		// 	}
		// 	printf("\n");
		// }

		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

		// calculate next shot
		t.tv_nsec += (kFixedTimeStep * kSecToNanosec);
		while (t.tv_nsec >= kSecToNanosec)
		{
			t.tv_nsec -= kSecToNanosec;
			t.tv_sec++;
		}
	}

	std::cout << "Disconnect sim" << std::endl;

	sim->disconnect();

	std::cout << "Delete sim and exit" << std::endl;
	delete sim;
}
