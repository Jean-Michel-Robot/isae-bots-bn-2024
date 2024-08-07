#ifndef _SETUP_LOOP
#define _SETUP_LOOP

// forward declarations
class ROS;
class OdosPosition;
class LinearTrajectory;
class RotationTrajectory;
class Asserv;
class BrSMWrapper;

// pointers to the classes, accessible from anywhere in the src code
extern ROS* p_ros;
extern OdosPosition* p_odos;

extern Asserv *p_asserv;
extern BrSMWrapper *p_sm;

#endif
