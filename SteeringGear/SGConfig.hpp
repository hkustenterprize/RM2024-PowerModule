#pragma once

namespace SGChassis
{
namespace SGConfig
{
// ============================================   //
// Chassis body Parameters                        //
// ============================================   //

#define SG_WHEEL_DIAMETER 0.105f
#define SG_CHASSIS_LENGTH 0.206f

// ============================================   //
// Wheels and Steering Gears Parameters           //
// ============================================   //

/*->Right Front Wheel<-*/
// Zero encoder
#define SG_RF_ZERO_ENCODER 2020
// PID config
#define SG_RF_STEER_KP 3.5f
#define SG_RF_STEER_KI 2.0f
#define SG_RF_STEER_KD 0.15f
#define SG_RF_STEER_OUTPUT_MAX 3.0f
#define SG_RF_STEER_INTEGRAL_MAX 0.3f
#define SG_RF_STEER_ALPHA 0.1f

#define SG_RF_WHEEL_KP 0.3f
#define SG_RF_WHEEL_KI 0.3f
#define SG_RF_WHEEL_KD 0.0f
#define SG_RF_WHEEL_OUTPUT_MAX 10.0f
#define SG_RF_WHEEL_INTEGRAL_MAX 10.0f
#define SG_RF_WHEEL_ALPHA 0.2f

/*->Left Front Wheel<-*/
// Zero encoder
#define SG_LF_ZERO_ENCODER 2059
// PID config
#define SG_LF_STEER_KP 3.5f
#define SG_LF_STEER_KI 2.0f
#define SG_LF_STEER_KD 0.15f
#define SG_LF_STEER_OUTPUT_MAX 3.0f
#define SG_LF_STEER_INTEGRAL_MAX 0.3f
#define SG_LF_STEER_ALPHA 0.1f

#define SG_LF_WHEEL_KP 0.3f
#define SG_LF_WHEEL_KI 0.3f
#define SG_LF_WHEEL_KD 0.0f
#define SG_LF_WHEEL_OUTPUT_MAX 10.0f
#define SG_LF_WHEEL_INTEGRAL_MAX 10.0f
#define SG_LF_WHEEL_ALPHA 0.2f

/*->Left Back Wheel<-*/

// Zero encoder
#define SG_LB_ZERO_ENCODER 4056
// PID config
#define SG_LB_STEER_KP 3.5f
#define SG_LB_STEER_KI 2.0f
#define SG_LB_STEER_KD 0.15f
#define SG_LB_STEER_OUTPUT_MAX 3.0f
#define SG_LB_STEER_INTEGRAL_MAX 0.3f
#define SG_LB_STEER_ALPHA 0.1f

#define SG_LB_WHEEL_KP 0.3f
#define SG_LB_WHEEL_KI 0.3f
#define SG_LB_WHEEL_KD 0.0f
#define SG_LB_WHEEL_OUTPUT_MAX 10.0f
#define SG_LB_WHEEL_INTEGRAL_MAX 10.0f
#define SG_LB_WHEEL_ALPHA 0.2f

/*->Right Back Wheel<-*/

// Zero encoder
#define SG_RB_ZERO_ENCODER 5431

// PID config
#define SG_RB_STEER_KP 3.50f
#define SG_RB_STEER_KI 2.0f
#define SG_RB_STEER_KD 0.15f
#define SG_RB_STEER_OUTPUT_MAX 3.0f
#define SG_RB_STEER_INTEGRAL_MAX 0.3f
#define SG_RB_STEER_ALPHA 0.1f

#define SG_RB_WHEEL_KP 0.3f
#define SG_RB_WHEEL_KI 0.3f
#define SG_RB_WHEEL_KD 0.0f
#define SG_RB_WHEEL_OUTPUT_MAX 10.0f
#define SG_RB_WHEEL_INTEGRAL_MAX 10.0f
#define SG_RB_WHEEL_ALPHA 0.2f

// ============================================   //
// Remote controller Parameters                   //
// ============================================   //

#define SG_RC_MAX_LINEAR_VEL 3.6f
#define SG_RC_MAX_ANGULAR_VEL 15.0f

// ============================================   //
// Power Controller Parameters                    //
// ============================================   //
#define SG_ENERGY_LOOP_KP 50.0f
#define SG_ENERGY_LOOP_KD 0.2f
#define MAX_CAP_POWER_OUT 300.0f

}  // namespace SGConfig
}  // namespace SGChassis