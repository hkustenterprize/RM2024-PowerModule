#include "SGChassisControl.hpp"

#include "FreeRTOS.h"
#include "RefereeSystemComm.hpp"
#include "RefereeSystemMessage.hpp"
#include "SGConfig.hpp"
#include "SuperCapaManager.hpp"
#include "task.h"

namespace SGChassis
{
namespace SGChassisControl
{

static const Core::Control::PID::Param powerPDParam(SG_ENERGY_LOOP_KP, 0.0f, SG_ENERGY_LOOP_KD, 0.0f, MAX_CAP_POWER_OUT, 0.0f, 0.0f, 0.2f, 100UL);
static Core::Control::PID powerPD_base(powerPDParam);  // ensure the system does not die
static Core::Control::PID powerPD_full(powerPDParam);  // ensure the capacitor's energy keeps under 90%
static Core::Communication::RefereeSystem::RefereeSystemMessageReceive<Core::Communication::RefereeSystem::RefereeRobotStatusMessageData>
    robotStatusMessageReceiver;
static Core::Communication::RefereeSystem::RefereeSystemMessageReceive<Core::Communication::RefereeSystem::RefereePowerHeatMessageData>
    powerHeatMessageReceiver;
static uint8_t LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL;
const Core::Control::SuperCapacitor::CapacitorStatus &capStatus = Core::Control::SuperCapacitor::getStatus();
float SteeringGearController::estimatedCapEnergy                = 0.0f;
float SteeringGearController::measuredPower                     = 0.0f;
float SteeringGearController::estimatedPower                    = 0.0f;
float SteeringGearController::refereeMaxPower                   = 45.0f;

SteeringGearController::SteeringGearController(const SGUtil::Quadruple<AbstractFeedbackMotor *> &steerMotor_,
                                               const SGUtil::Quadruple<Core::Control::PID *> &steerPID_,
                                               const SGUtil::Quadruple<AbstractFeedbackMotor *> &wheelMotor_,
                                               const SGUtil::Quadruple<Core::Control::PID *> &wheelPID_,
                                               const SteeringGearConfig &config_)
    : steerMotor(steerMotor_),
      steerPID(steerPID_),
      wheelMotor(wheelMotor_),
      wheelPID(wheelPID_),
      steerLimitator(0.741f, 0.005f, 12.98f, 7.8f / 2.0f, steerMotor_, steerPID_),
      wheelLimitator(0.214f, 0.132f, 3.47f, 7.8f / 2.0f, wheelMotor_, wheelPID_),
      config(config_),
      capEnabled(false),
      lastUpdateTick(xTaskGetTickCount())
{
}

const SteeringGearConfig::Dynamics &SteeringGearController::getMeasuredVelocity(void) { return this->measuredVelocity; }

void SteeringGearController::setTargetVelocity(float vx, float vy, float vw)
{
    taskENTER_CRITICAL();
    {
        this->targetVx = vx;
        this->targetVy = vy;
        this->targetVw = vw;
    }
    taskEXIT_CRITICAL();
}

void SteeringGearController::solveMeasuredDynamics()
{
    for (int i = 0; i < 4; i++)
    {
        currentAvX[i] = wheelsFeedbackAv[i] * cosf(steersFeedbackPosition[i]);
        currentAvY[i] = wheelsFeedbackAv[i] * sinf(steersFeedbackPosition[i]);
    }

    static const float cosAngle = cosf((float)(M_PI_4));

    measuredVelocity.x = (currentAvY[1] + currentAvY[3] + currentAvY[0] + currentAvY[2]) * (config.diameter / 2.0f) / 4.0f;
    measuredVelocity.y = (currentAvX[0] + currentAvX[2] + currentAvX[1] + currentAvX[3]) * (config.diameter / 2.0f) / 4.0f;
    measuredVelocity.w = (currentAvX[0] - currentAvX[2] - currentAvY[1] + currentAvY[3]) * (config.diameter / 2.0f) / 4.0f / cosAngle / 2.0f *
                         config.diameter / config.length;
}

void SteeringGearController::updatePowerStatus()
{
    // Update Power Status;
    static float MIN_MAXPOWER_CONFIGURED = 40.0f;

    TickType_t now = xTaskGetTickCount();

    estimatedPower = steerLimitator.getEstimatedPower() + wheelLimitator.getEstimatedPower();
    bool capValid  = capStatus.isConnected && (capStatus.capacitorRx.errorCode == 0) && (capStatus.capacitorTx.enableDCDC);
    if (!capValid)
    {
        // Judge whether the cap energy is used-up
        if (Core::Communication::RefereeSystem::isConnected() && powerHeatMessageReceiver.getData().bufferEnergy < MAX_POEWR_REFEREE_BUFF &&
            powerHeatMessageReceiver.getData().chassisPower > CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD)
        {
            measuredPower      = powerHeatMessageReceiver.getData().chassisPower;
            estimatedCapEnergy = 0.0f;
        }
        else
        {
            measuredPower = estimatedPower;
            if (powerHeatMessageReceiver.getData().chassisPower < MIN_MAXPOWER_CONFIGURED && powerHeatMessageReceiver.getData().bufferEnergy == 60U)
            {
                estimatedCapEnergy = 2100.0f;
            }
            else
            {
                estimatedCapEnergy += (powerHeatMessageReceiver.getData().chassisPower - estimatedPower) *
                                      static_cast<float>((now - lastUpdateTick) / configTICK_RATE_HZ);
                estimatedCapEnergy = Core::Utils::Math::clamp(estimatedCapEnergy, 0.0f, 2100.0f);
            }
        }
    }
    else
    {
        measuredPower      = capStatus.capacitorRx.chassisPower;
        estimatedCapEnergy = capStatus.capacitorRx.capEnergy / 255.0f * 2100.0f;
    }

    if (Core::Communication::RefereeSystem::isConnected())
    {
        refereeMaxPower = fmax(robotStatusMessageReceiver.getData().chassis_power_limit, CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD);
        if (robotStatusMessageReceiver.getData().robot_level > 10U)
            LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL = 1U;
        else
            LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL = fmax(1U, robotStatusMessageReceiver.getData().robot_level);
    }
    else
    {
        refereeMaxPower = InfantryChassisPowerLimit_HP_FIRST[LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL - 1U];
    }

    MIN_MAXPOWER_CONFIGURED = refereeMaxPower * 0.8f;

    float baseMaxPower, fullMaxPower, maxPower;
    if (capValid)
    {
        baseMaxPower = fmax(MIN_MAXPOWER_CONFIGURED, refereeMaxPower - powerPD_base(sqrtf(capBaseBuffSet), sqrtf(capStatus.capacitorRx.capEnergy)));
        fullMaxPower = fmax(MIN_MAXPOWER_CONFIGURED, refereeMaxPower - powerPD_full(sqrtf(capFullBuffSet), sqrtf(capStatus.capacitorRx.capEnergy)));
    }
    else
    {
        if (Core::Communication::RefereeSystem::isConnected())
        {
            baseMaxPower = fmax(MIN_MAXPOWER_CONFIGURED,
                                refereeMaxPower - powerPD_base(sqrtf(refereeBaseBuffSet), sqrtf(powerHeatMessageReceiver.getData().bufferEnergy)));
            fullMaxPower = fmax(MIN_MAXPOWER_CONFIGURED,
                                refereeMaxPower - powerPD_full(sqrtf(refereeFullBuffSet), sqrtf(powerHeatMessageReceiver.getData().bufferEnergy)));
        }
        else
        {
            baseMaxPower = refereeMaxPower * CAP_REFEREE_BOTH_GG_COE;
            fullMaxPower = refereeMaxPower * CAP_REFEREE_BOTH_GG_COE;
        }
    }
    // if (lowPowerCapChargingMode){
    //     maxPower = Core::Utils::Math::clamp(capEnabled ? refereeMaxPower + MAX_CAP_POWER_OUT : refereeMaxPower * 0.8f, fullMaxPower, baseMaxPower);
    // }
    // else{
    //     maxPower = Core::Utils::Math::clamp(capEnabled ? refereeMaxPower + MAX_CAP_POWER_OUT : refereeMaxPower, fullMaxPower, baseMaxPower);
    // }
    if (Core::Communication::RefereeSystem::isConnected() && capValid)
    {
        switch (capMode)
        {
        case 1:
            // if (lowPowerCapChargingMode && (getEstimatedCapEnergy() < 200.0f)){
            //     maxPower = Core::Utils::Math::clamp(capEnabled ? refereeMaxPower + MAX_CAP_POWER_OUT  : refereeMaxPower * 0.80f, fullMaxPower,
            //     baseMaxPower);
            // }
            if (refereeMaxPower <= 75.0f)
            {
                maxPower = Core::Utils::Math::clamp(capEnabled ? refereeMaxPower + MAX_CAP_POWER_OUT : 75.0f, fullMaxPower, baseMaxPower);
            }
            else
            {
                maxPower =
                    Core::Utils::Math::clamp(capEnabled ? refereeMaxPower + MAX_CAP_POWER_OUT : refereeMaxPower + 10.0f, fullMaxPower, baseMaxPower);
            }
            break;
        case 2:
            if ((getEstimatedCapEnergy() < 230.0f && SGUtil::floatEqual(this->targetVx, 0.0f) && SGUtil::floatEqual(this->targetVy, 0.0f)) ||
                (refereeMaxPower > 80.0f))
            {
                maxPower =
                    Core::Utils::Math::clamp(capEnabled ? refereeMaxPower + MAX_CAP_POWER_OUT : refereeMaxPower * 0.80f, fullMaxPower, baseMaxPower);
            }
            else
            {
                maxPower = Core::Utils::Math::clamp(capEnabled ? refereeMaxPower + MAX_CAP_POWER_OUT : refereeMaxPower, fullMaxPower, baseMaxPower);
            }
            break;
        case 3:
            maxPower = Core::Utils::Math::clamp(capEnabled ? refereeMaxPower + MAX_CAP_POWER_OUT : refereeMaxPower, fullMaxPower, baseMaxPower);
            break;
        default:
            maxPower = Core::Utils::Math::clamp(capEnabled ? refereeMaxPower + MAX_CAP_POWER_OUT : refereeMaxPower, fullMaxPower, baseMaxPower);
            break;
        }
    }
    else
    {
        maxPower = Core::Utils::Math::clamp(capEnabled ? refereeMaxPower + MAX_CAP_POWER_OUT : refereeMaxPower, fullMaxPower, baseMaxPower);
    }
    // maxPower = Core::Utils::Math::clamp(capEnabled ? refereeMaxPower + MAX_CAP_POWER_OUT : 75.0f, fullMaxPower, baseMaxPower);
    static const float ratio = 0.8f;
    maxPower                 = fmax(maxPower, MIN_MAXPOWER_CONFIGURED);

    steerLimitator.updateStatus(maxPower * ratio);
    wheelLimitator.updateStatus(maxPower - fmin(steerLimitator.getCommandPower(), maxPower * ratio));
}

void SteeringGearController::update()
{
    TickType_t now = xTaskGetTickCount();
    solveMeasuredDynamics();
    // Update feedback
    for (int i = 0; i < 4; ++i)
    {
        wheelsFeedbackAv[i] = SGUtil::rpm2rad(wheelMotor[i]->getRPMFeedback());

        float curPosition = steerMotor[i]->getPositionFeedback();

        if (fabs(curPosition - lastSteersFeedbackPosition[i]) > (float)(M_PI))
        {
            steersFeedbackPosition[i] = curPosition;
        }
        else
        {
            steersFeedbackPosition[i] = lastSteersFeedbackPosition[i] * 0.1f + curPosition * 0.9f;
        }

        lastSteersFeedbackPosition[i] = steersFeedbackPosition[i];
    }

    static const float angle    = (float)(M_PI_4);
    static const float cosAngle = cosf(angle);
    static const float sinAngle = sin(angle);

    float wheelsVw = targetVw * config.length / config.diameter * 2.0f;

    wheelsTargetAvX[0] = (targetVy + wheelsVw * cosAngle) / (config.diameter / 2.0f);
    wheelsTargetAvX[1] = (targetVy - wheelsVw * sinAngle) / (config.diameter / 2.0f);
    wheelsTargetAvX[2] = (targetVy - wheelsVw * cosAngle) / (config.diameter / 2.0f);
    wheelsTargetAvX[3] = (targetVy + wheelsVw * sinAngle) / (config.diameter / 2.0f);
    wheelsTargetAvY[0] = (targetVx - wheelsVw * sinAngle) / (config.diameter / 2.0f);
    wheelsTargetAvY[1] = (targetVx - wheelsVw * cosAngle) / (config.diameter / 2.0f);
    wheelsTargetAvY[2] = (targetVx + wheelsVw * sinAngle) / (config.diameter / 2.0f);
    wheelsTargetAvY[3] = (targetVx + wheelsVw * cosAngle) / (config.diameter / 2.0f);

    for (int i = 0; i < 4; ++i)
    {
        if (SGUtil::floatEqual(wheelsTargetAvX[i], wheelsTargetAvY[i], 0.0f))
        {
            steersError[i]    = SGUtil::clampSteerError(steersTargetPosition[i] - steersFeedbackPosition[i]);
            wheelsTargetAv[i] = 0.0f;
        }
        else
        {
            float target     = SGUtil::clampSteerError(atan2f(wheelsTargetAvY[i], wheelsTargetAvX[i]));
            float targets[2] = {target, SGUtil::clampSteerError(target + (float)M_PI)};
            float path[2]    = {SGUtil::clampSteerError(targets[0] - steersFeedbackPosition[i]),
                                SGUtil::clampSteerError(targets[1] - steersFeedbackPosition[i])};

            if (fabs(path[0]) < fabs(path[1]))
            {
                steersError[i]          = path[0];
                steersTargetPosition[i] = targets[0];
                wheelsTargetAv[i]       = sqrtf(wheelsTargetAvX[i] * wheelsTargetAvX[i] + wheelsTargetAvY[i] * wheelsTargetAvY[i]);
            }
            else
            {
                steersError[i]          = path[1];
                steersTargetPosition[i] = targets[1];
                wheelsTargetAv[i]       = -sqrtf(wheelsTargetAvX[i] * wheelsTargetAvX[i] + wheelsTargetAvY[i] * wheelsTargetAvY[i]);
            }
        }

        // filter the target position
        float targetPosition = steersTargetPosition[i];
        if (fabs(targetPosition - lastSteersTargetPosition[i]) > (float)(M_PI))
        {
            steersTargetPosition[i] = targetPosition;
        }
        else
        {
            steersTargetPosition[i] = lastSteersTargetPosition[i] * 0.8f + targetPosition * 0.2f;
        }

        lastSteersTargetPosition[i] = steersTargetPosition[i];
    }

    // PID and power controlt
    SGUtil::Quadruple<float> steerDecayCurrent;
    SGUtil::Quadruple<float> wheelDecayCurrent;
    for (int i = 0; i < 4; ++i)
    {
        // Update PID
        // if motor is connected, update pid, otherwise setoutput 0
        if (steerMotor[i]->isConnected())
        {
            steerDecayCurrent[i] = (*steerPID[i])(0, -steersError[i]);
            if (wheelMotor[i]->isConnected())
            {
                wheelDecayCurrent[i] = ((*wheelPID[i])(wheelsTargetAv[i], wheelsFeedbackAv[i]));
            }
            else
            {
                wheelDecayCurrent[i] = 0.0f;
                wheelPID[i]->reset();
            }
        }
        else
        {
            steerDecayCurrent[i] = 0.0f;
            wheelDecayCurrent[i] = 0.0f;
            steerPID[i]->reset();
            wheelPID[i]->reset();
        }
    }

    // update power status
    updatePowerStatus();

    steerLimitator.getDecayCurrent(steerDecayCurrent);
    wheelLimitator.getDecayCurrent(wheelDecayCurrent);

    for (int i = 0; i < 4; ++i)
    {
        steerMotor[i]->setCurrent(steerDecayCurrent[i]);
        wheelMotor[i]->setCurrent(wheelDecayCurrent[i]);
    }

    lastUpdateTick = now;
}

}  // namespace SGChassisControl
}  // namespace SGChassis