#include "ChassisController.hpp"

#if USE_CHASSIS_CONTROLLER

#include "AbstractFeedbackMotor.hpp"
#include "FreeRTOS.h"
#include "IMU.hpp"
#include "PID.hpp"
#include "Quaternion.hpp"
#include "assert.h"
#include "math.h"
#include "task.h"

#if USE_POWER_CONTROLLER
#include "PowerController.hpp"
#endif

using namespace Core::Control;
using namespace Core::Drivers;

namespace Core
{
namespace Control
{
namespace Chassis
{
#define ABS(x) ((x > 0) ? x : -x)
#define SQRT2 1.41421356237f
// #define PI 3.1415926f
#define RPM_TO_ANGULAR_SPEED_DIV (30.0f / PI)

#define CHASSIS_CLAMP(x, min, max) ((x < min) ? min : (x > max) ? max : x)
#define SIGN(x, y) (((x < 0 && y < 0) || (x > 0 && y > 0)) ? 1 : -1)

inline float maxAccClamp(float velocity, float lastVelocity, float maxAcc, float timeDiff)
{
    float currentAcc = (velocity - lastVelocity) / timeDiff;
    return (currentAcc < -maxAcc) ? lastVelocity - timeDiff * maxAcc : (currentAcc > maxAcc) ? lastVelocity + timeDiff * maxAcc : velocity;
}

ChassisConfigs::ChassisConfigs(AbstractFeedbackMotor *pMotorRf,
                               AbstractFeedbackMotor *pMotorLf,
                               AbstractFeedbackMotor *pMotorLb,
                               AbstractFeedbackMotor *pMotorRb,

                               PID *pPIDRf,
                               PID *pPIDLf,
                               PID *pPIDLb,
                               PID *pPIDRb,

                               const ChassisPropertyConfig &property,
                               const float maxAcceleration_,
                               const float maxAngularAccelertion_,
                               const Dynamics &maxSpeed_)
    : chassisPropertyConfig(property),
      chassisMotors(ChassisMotorsData<AbstractFeedbackMotor *>(pMotorRf, pMotorLf, pMotorLb, pMotorRb)),
      chassisPIDs(ChassisMotorsData<PID *>(pPIDRf, pPIDLf, pPIDLb, pPIDRb)),
      lastUpdate(xTaskGetTickCount()),
      maxAcceleration(maxAcceleration_),
      maxAngularAcceleration(maxAngularAccelertion_),
      maxSpeed(maxSpeed_)
{
}

ChassisController::ChassisController(const ChassisConfigs &chassisConfig_)
    : wheelsTargetAv(ChassisConfigs::ChassisMotorsData<float>(0.0f, 0.0f, 0.0f, 0.0f)),
      wheelsOutput(ChassisConfigs::ChassisMotorsData<float>(0.0f, 0.0f, 0.0f, 0.0f)),
      chassisConfigs(chassisConfig_),

      rawTargetVelocity(ChassisConfigs::Dynamics()),
      lastRawTargetVelocity(ChassisConfigs::Dynamics()),
      targetVelocity(ChassisConfigs::Dynamics()),
      lastTargetVelocity(ChassisConfigs::Dynamics()),
      measuredVelocity(ChassisConfigs::Dynamics()),
      lastMeasuredVelocity(ChassisConfigs::Dynamics()),
      measuredAcceleration(ChassisConfigs::Dynamics()),
      measuredDisplacement(ChassisConfigs::Dynamics())
{
    diameters = chassisConfigs.chassisPropertyConfig.chassisWheelDiameters;
#if USE_MECANUM_CHASSIS
    sum      = chassisConfigs.chassisPropertyConfig.chassisFrontBackLength + chassisConfigs.chassisPropertyConfig.chassisRightLeftLength;
    wFactor  = -diameters / (4.0f * sum);
    xyFactor = diameters / (8.0f);
#else
    // clang-format off
    length    = chassisConfigs.chassisPropertyConfig.chassisWheelToCentreLength;
    wFactor   = -diameters / (8.0f * length);
    xyFactor  = diameters / (4.0f * SQRT2);
    // clang-format on
#endif
}

void ChassisController::settargetVelocity(float x, float y, float w)
{
    lastRawTargetVelocity.x = rawTargetVelocity.x;
    lastRawTargetVelocity.y = rawTargetVelocity.y;
    lastRawTargetVelocity.w = rawTargetVelocity.w;
    rawTargetVelocity.x     = x;
    rawTargetVelocity.y     = y;
    rawTargetVelocity.w     = w;
    targetVelocity.x        = CHASSIS_CLAMP(x, -chassisConfigs.maxSpeed.x, chassisConfigs.maxSpeed.x);
    targetVelocity.y        = CHASSIS_CLAMP(y, -chassisConfigs.maxSpeed.y, chassisConfigs.maxSpeed.y);
    targetVelocity.w        = CHASSIS_CLAMP(w, -chassisConfigs.maxSpeed.w, chassisConfigs.maxSpeed.w);
}

void ChassisController::settargetVelocity(float speeds[3]) { this->settargetVelocity(speeds[0], speeds[1], speeds[2]); }

void ChassisController::setTargetPercentageSpeed(float vxPercentage, float vyPercentage, float vwPercentage)
{
    this->settargetVelocity(
        vxPercentage * chassisConfigs.maxSpeed.x, vyPercentage * chassisConfigs.maxSpeed.y, vwPercentage * chassisConfigs.maxSpeed.w);
}

const ChassisConfigs::Dynamics &ChassisController::getMeasuredAccleration(void) { return this->measuredAcceleration; }

const ChassisConfigs::Dynamics &ChassisController::getMeasuredVelocity(void) { return this->measuredVelocity; }

const ChassisConfigs::Dynamics &ChassisController::getMeasuredDisplacement(void) { return this->measuredDisplacement; }

void ChassisController::setMaxVelocity(const ChassisConfigs::Dynamics &newMaxSpeeds) { this->chassisConfigs.maxSpeed = newMaxSpeeds; }

void ChassisController::setMaxAngularAcceleration(const float maxAngularAcceleration)
{
    this->chassisConfigs.maxAngularAcceleration = maxAngularAcceleration;
}

void ChassisController::setMaxAcceleration(const float maxAcceleration) { this->chassisConfigs.maxAcceleration = maxAcceleration; }

void ChassisController::update()
{
    solveMeasuredDynamics();

/**
 * @brief Calculate target RPM
 */
#if USE_MECANUM_CHASSIS
    wheelsTargetAv[0] = (-targetVelocity.y - targetVelocity.x - targetVelocity.w * (sum / 2.0f)) * 2.0f / diameters;
    wheelsTargetAv[1] = (targetVelocity.y - targetVelocity.x - targetVelocity.w * (sum / 2.0f)) * 2.0f / diameters;
    wheelsTargetAv[2] = (targetVelocity.y + targetVelocity.x - targetVelocity.w * (sum / 2.0f)) * 2.0f / diameters;
    wheelsTargetAv[3] = (-targetVelocity.y + targetVelocity.x - targetVelocity.w * (sum / 2.0f)) * 2.0f / diameters;
#else
    wheelsTargetAv[0] = (-SQRT2 / 2.0f * targetVelocity.x - SQRT2 / 2.0f * targetVelocity.y - targetVelocity.w * length) * 2.0f / diameters;
    wheelsTargetAv[1] = (-SQRT2 / 2.0f * targetVelocity.x + SQRT2 / 2.0f * targetVelocity.y - targetVelocity.w * length) * 2.0f / diameters;
    wheelsTargetAv[2] = (SQRT2 / 2.0f * targetVelocity.x + SQRT2 / 2.0f * targetVelocity.y - targetVelocity.w * length) * 2.0f / diameters;
    wheelsTargetAv[3] = (SQRT2 / 2.0f * targetVelocity.x - SQRT2 / 2.0f * targetVelocity.y - targetVelocity.w * length) * 2.0f / diameters;
#endif

    /**
     * @brief Calculate power and do power limitation
     */
    float pidOutput[4];
#if USE_POWER_CONTROLLER

    static Power::PowerObj objs[4];
    static Power::PowerObj *pObjs[4] = {&objs[0], &objs[1], &objs[2], &objs[3]};
    // Update Power Objects
    for (int i = 0; i < 4; i++)
    {
        objs[i].curAv     = currentAv[i];
        objs[i].setAv     = wheelsTargetAv[i];
        objs[i].pidOutput = pidOutput[i] = (*chassisConfigs.chassisPIDs[i])(wheelsTargetAv[i], currentAv[i]);
        objs[i].pidMaxOutput             = chassisConfigs.chassisPIDs[i]->getParam().outputMax;
    }
    float *controlledOutput = Power::getControlledOutput(pObjs);
    for (int i = 0; i < 4; i++)
    {
        wheelsOutput[i] = controlledOutput[i];
    }
#else
    for (int i = 0; i < 4; i++)
    {
        pidOutput[i]    = (*chassisConfigs.chassisPIDs[i])(wheelsTargetAv[i], currentAv[i]);
        wheelsOutput[i] = pidOutput[i];
    }
#endif
    for (int i = 0; i < 4; i++)
    {
        chassisConfigs.chassisMotors[i]->setOutput(wheelsOutput[i]);
    }
}

void ChassisController::solveMeasuredDynamics()
{
    /**
     * @brief Update tick count
     */
    float nowTick  = xTaskGetTickCount();
    float timeDiff = nowTick - this->chassisConfigs.lastUpdate;

    this->chassisConfigs.lastUpdate = nowTick;

    /**
     * @brief Clamp the velocity
     */

    /**
     * @brief Update last measured speeds
     */
    lastMeasuredVelocity.w = measuredVelocity.w;
    lastMeasuredVelocity.y = measuredVelocity.y;
    lastMeasuredVelocity.x = measuredVelocity.x;
    measuredVelocity.w     = 0.0f;
    for (int i = 0; i < 4; i++)
        currentAv[i] = chassisConfigs.chassisMotors[i]->getRPMFeedback() / RPM_TO_ANGULAR_SPEED_DIV;
#if USE_MECANUM_CHASSIS
    measuredVelocity.x = (currentAv[2] + currentAv[3] - currentAv[1] - currentAv[0]) * xyFactor;
#else
    measuredVelocity.x = -(currentAv[2] + currentAv[3] - currentAv[0] - currentAv[1]) * xyFactor;
#endif
    measuredVelocity.y = (currentAv[1] + currentAv[2] - currentAv[0] - currentAv[3]) * xyFactor;
    measuredVelocity.w = (currentAv[0] + currentAv[1] + currentAv[2] + currentAv[3]) * wFactor;

    /**
     * @brief Get displacements data
     */
    measuredDisplacement.w += (measuredVelocity.w * timeDiff / (float)(configTICK_RATE_HZ));
    measuredDisplacement.y += (measuredVelocity.y * timeDiff / (float)(configTICK_RATE_HZ));
    measuredDisplacement.x += (measuredVelocity.x * timeDiff / (float)(configTICK_RATE_HZ));

    /**
     * @brief Get acceleration data
     */
    measuredAcceleration.w = (measuredVelocity.w - lastMeasuredVelocity.w) / (timeDiff / (float)(configTICK_RATE_HZ));
    measuredAcceleration.y = (measuredVelocity.y - lastMeasuredVelocity.y) / (timeDiff / (float)(configTICK_RATE_HZ));
    measuredAcceleration.x = (measuredVelocity.x - lastMeasuredVelocity.x) / (timeDiff / (float)(configTICK_RATE_HZ));
}

void ChassisController::stop(void)
{
    solveMeasuredDynamics();
    for (int i = 0; i < 4; i++)
    {
        chassisConfigs.chassisPIDs[i]->reset();
        chassisConfigs.chassisMotors[i]->setOutput(0);
    }
}

const ChassisConfigs &ChassisController::getConfig(void) { return this->chassisConfigs; }

}  // namespace Chassis
}  // namespace Control
}  // namespace Core

#undef ABS
#undef SQRT2
#undef PI
#undef RPM_TO_ANGULAR_SPEED_DIV
#undef CHASSIS_CLAMP
#undef SIGN

#endif