#define PWM_FREQUENCY			500
#define PWM_RESOLUTION_BITS		11
#define PWM_MAX_VALUE			((1 << PWM_RESOLUTION_BITS) - 1)
#define PWM_MIN_VALUE			(PWM_MAX_VALUE / 2)

#define PIN_MOTOR_FL			25
#define PIN_MOTOR_FR			26
#define PIN_MOTOR_BL			16
#define PIN_MOTOR_BR			19

#define CHANNEL_MOTOR_FL		0
#define CHANNEL_MOTOR_FR		1
#define CHANNEL_MOTOR_BL		2
#define CHANNEL_MOTOR_BR		3

struct ThrottleOutput {
int motor_FL;
int motor_FR;
int motor_BL;
int motor_BR;
};