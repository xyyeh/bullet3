SUBDIRS( HelloWorld BasicDemo)
IF(BUILD_BULLET3)
	SUBDIRS( ExampleBrowser RobotSimulator RobotSimulatorShm RobotSimulatorStewart SharedMemory ThirdPartyLibs/Gwen ThirdPartyLibs/BussIK ThirdPartyLibs/clsocket OpenGLWindow TwoJoint )
ENDIF()

IF(BUILD_PYBULLET)
	SUBDIRS(pybullet)
ENDIF(BUILD_PYBULLET)
