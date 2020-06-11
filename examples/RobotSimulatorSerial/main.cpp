#include "bullet_api/b3RobotSimulatorClientAPI.h"

#include <string.h>
#include <iostream>
#include <assert.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <memory>
#include <vector>

#include "external/include/robot.h"
#include "external/include/shm_sem.h"
#include "external/include/shm_struct.h"
#include "external/include/timer.h"

const double kSwitchingTime = 3.0;

int main(int argc, char* argv[])
{
	// parse options
	std::string urdf_path;
	double sim_frequency;
	if (argc == 3)
	{
		urdf_path = std::string(argv[1]);
		sim_frequency = std::atof(argv[2]);
	}
	else if (argc == 2)
	{
		urdf_path = std::string(argv[1]);
		sim_frequency = 4000;
	}
	else
	{
		std::cout << "Please run with: <executable> <urdf path> <simulation frequency>" << std::endl;
		return -1;
	}

	// thread properties
	struct sched_param params;
	params.sched_priority = 49;  // max value at sched_get_priority_max(SCHED_FIFO);
	auto ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &params);
	if (ret != 0)
	{
		std::cout << "Unsuccessful in setting thread realtime priority" << std::endl;
		return -1;
	}

	// pointer to simulation
	std::unique_ptr<b3RobotSimulatorClientAPI> sim = std::make_unique<b3RobotSimulatorClientAPI>();
	if (!sim->connect(eCONNECT_GUI))
	{
		std::cout << "Cannot connect" << std::endl;
		return -1;
	}

	// shared memory
	SharedMemory shmem;
	shmem.Initialize("/bullet_shm");
	shmem.Create(sizeof(struct RobotInterface));
	shmem.Attach();
	RobotInterface* ptr = static_cast<RobotInterface*>(shmem.Data());

	// setup visualizer
	sim->configureDebugVisualizer(COV_ENABLE_GUI, 0);
	sim->configureDebugVisualizer(COV_ENABLE_SHADOWS, 1);

	// setup timeout
	sim->setTimeOut(10);

	// syncBodies is only needed when connecting to an existing physics server that has already some bodies
	sim->syncBodies();

	// setup fixed time step
	RTTimer timer;
	timer.SetFrequency(sim_frequency);
	sim->setTimeStep(1.0 / timer.Frequency());

	// setup world
	sim->setGravity(btVector3(0, 0, 0));
	int plane_id = sim->loadURDF("plane.urdf");

	// setup robot
	Robot rbt;
	int rbt_id = rbt.Setup(sim.get(), urdf_path, btVector3(0, 0, 0));
	if (rbt_id < 0)
	{
		std::cout << "Robot not parsed" << std::endl;
		return -1;
	}
	struct b3RobotSimulatorChangeDynamicsArgs dynArgs;
	dynArgs.m_linearDamping = 0;
	dynArgs.m_angularDamping = 0;
	sim->changeDynamics(rbt_id, 0, dynArgs);

	// simulation paramters
	sim->setRealTimeSimulation(false);

	// disable collisions
	for (int i = -1; i < rbt.Dof(); i++)
	{
		sim->setCollisionFilterGroupMask(rbt_id, i, 0, 0);
	}
	sim->setCollisionFilterGroupMask(plane_id, -1, 0, 0);

	// local feedback and commands
	std::vector<double> q(rbt.Dof());
	std::vector<double> dq(rbt.Dof());
	std::vector<double> tau_default(rbt.Dof());
	std::vector<double> tau_J_d(rbt.Dof());

	// start looping after one second
	double sim_time = 0, prev_time = 0;
	double gamma = 0, ctrler_on_time = 0;
	timer.StartWithDelay(1);
	while (1)
	{
		if (!sim->canSubmitCommand())
		{
			std::cout << "Failed to send command!" << std::endl;
		}
		sim_time += (1.0 / timer.Frequency());

		// joint feedback and command
		for (uint32_t i = 0; i < rbt.Dof(); i++)
		{
			rbt.JointState(sim.get(), i, q[i], dq[i]);
			rbt.SetDesiredTau(sim.get(), i, tau_J_d[i]);
		}

		// update shared memory
		shmem.Lock();
		for (uint32_t i = 0; i < rbt.Dof(); i++)
		{
			ptr->q[i] = q[i];
			ptr->dq[i] = dq[i];
			tau_J_d[i] = ptr->tau_J_d[i];
		}
		shmem.Unlock();

		// default commands
		for (uint32_t i = 0; i < rbt.Dof(); i++)
		{
			tau_default[i] = rbt.Kp(i) * (rbt.Home(i) - q[i]) - rbt.Kd(i) * dq[i];
		}

		// controller switching
		if (ptr->controller_to_hw)
		{
			double delta_time = (sim_time - ctrler_on_time);

			if (delta_time < kSwitchingTime)
			{
				gamma = delta_time / kSwitchingTime;
			}
			else
			{
				gamma = 1;
				ptr->hw_to_controller = 1;
			}
		}
		else
		{
			ctrler_on_time = sim_time;
		}

		// blend controller
		for (uint32_t i = 0; i < rbt.Dof(); i++)
		{
			rbt.SetDesiredTau(sim.get(), i, (1 - gamma) * tau_default[i] + gamma * tau_J_d[i]);
		}

		// if ((sim_time - prev_time) > 0.25)
		// {
		// 	prev_time = sim_time;
		// 	// print time
		// 	// printf("%.3f\n", sim_time);
		// 	for (int i = 0; i < rbt.Dof(); i++)
		// 	{
		// 		printf("%.3f\t", shm->tau[i]);
		// 	}
		// 	printf("\n");
		// }

		// step simulation
		sim->stepSimulation();

		// calculate next shot
		timer.WaitForTick();
	}

	std::cout << "Delete sim and exit" << std::endl;
	sim->disconnect();

	return 0;
}
