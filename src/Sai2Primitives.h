#include "tasks/JointTask.h"
#include "tasks/PositionTask.h"
#include "tasks/OrientationTask.h"
#include "tasks/PosOriTask.h"
#include "tasks/SupportAndConstraintsTask.h"
#include "tasks/TwoHandTwoRobotsTask.h"
#include "primitives/RedundantArmMotion.h"
#include "primitives/SurfaceSurfaceAlignment.h"
#include "primitives/DualArmObjectMotion.h"
#ifdef USING_OTG
#include "trajectory_generation/OTG.h"
#include "trajectory_generation/OTG_ori.h"
#include "trajectory_generation/OTG_posori.h"
#endif