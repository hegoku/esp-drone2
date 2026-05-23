#include <math.h>
#include "flight/control.h"
#include "controllers/pid.h"
#include "misc/config.h"
#include "clocksource/clocksource.h"
#include "flight/flight.h"
#include "math/math.h"
#include "controllers/attitude_euler_controller.h"
#include "controllers/attitude_q_controller.h"
#include "sdkconfig.h"

struct pid_data angle_rate_pid[3];

void init_control()
{
#ifdef CONFIG_ANGLE_PID_ALGORITHM_EULER_PID
	init_attitude_euler_controller();
#elif defined(CONFIG_ANGLE_PID_ALGORITHM_Q_PID)
	init_attitude_q_controller();
#endif
}

void control_update()
{
#ifdef CONFIG_ANGLE_PID_ALGORITHM_EULER_PID
	attitude_euler_controller_update();
#elif defined(CONFIG_ANGLE_PID_ALGORITHM_Q_PID)
	attitude_q_controller_update();
#endif
}