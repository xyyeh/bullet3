
include_directories( 
		${BULLET_PHYSICS_SOURCE_DIR}/src
		${BULLET_PHYSICS_SOURCE_DIR}/examples
		${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory
  )
  
set(SRCS
  main.cpp
	b3RobotSimulatorClientAPI.cpp
	b3RobotSimulatorClientAPI.h	
	robot.cpp
  robot.h
  shm_sem.h
  shm_sem.cc
	../ExampleBrowser/InProcessExampleBrowser.cpp
	../SharedMemory/GraphicsServerExample.cpp
	../SharedMemory/RemoteGUIHelper.cpp
	../SharedMemory/PhysicsServerExample.cpp
	../SharedMemory/PhysicsServerExampleBullet2.cpp
	../SharedMemory/SharedMemoryInProcessPhysicsC_API.cpp		
)	

add_executable(RobotSimulatorO2 ${SRCS})

target_link_libraries(RobotSimulatorO2 pthread rt BulletRobotics BulletExampleBrowserLib BulletFileLoader BulletWorldImporter BulletSoftBody BulletDynamics BulletCollision BulletInverseDynamicsUtils BulletInverseDynamics LinearMath OpenGLWindow gwen Bullet3Common)

