#include "PowerController.hpp"

#if USE_POWER_CONTROLLER

namespace Core
{
namespace Control
{
namespace Power
{

#define POWER_PD_KP 50.0f

Manager manager(Manager::Motors(nullptr, nullptr, nullptr, nullptr), Division::HERO);
PowerStatus powerStatus;
static uint8_t LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL;
static bool isCapEnergyOut                = false;
static uint16_t motorDisconnectCounter[4] = {0U, 0U, 0U, 0U};
static float MIN_MAXPOWER_CONFIGURED      = 30.0f;

static inline bool floatEqual(float a, float b) { return fabs(a - b) < 1e-5f; }

static inline float rpm2av(float rpm) { return rpm * (float)M_PI / 30.0f; }

static inline float av2rpm(float av) { return av * 30.0f / (float)M_PI; }

static inline void setErrorFlag(uint8_t &curFlag, Manager::ErrorFlags setFlag) { curFlag |= static_cast<uint8_t>(setFlag); }

static inline void clearErrorFlag(uint8_t &curFlag, Manager::ErrorFlags clearFlag) { curFlag &= (~static_cast<uint8_t>(clearFlag)); }

static inline bool isFlagged(uint8_t &curFlag, Manager::ErrorFlags flag) { return (curFlag & static_cast<uint8_t>(flag)) != 0; }

static inline bool isMotorConnected(const AbstractFeedbackMotor *motor) { return motor != nullptr && motor->getDisconnectCounter() < 10; }

static inline bool isAllMotorConnected()
{
    for (int i = 0; i < 4; i++)
        if (not isMotorConnected(manager.motors[i]))
            return false;
    return true;
}

Manager::Manager(
    const Motors &motors_, const Division division_, RLSEnabled rlsEnabled_, const float k1_, const float k2_, const float k3_, const float lambda_)

    : rlsEnabled(rlsEnabled_),
      error(0UL),
      motors(motors_),
      division(division_),
      powerBuff(0.0f),
      fullBuffSet(0.0f),
      baseBuffSet(0.0f),
      fullMaxPower(0.0f),
      baseMaxPower(0.0f),
      powerUpperLimit(0.0f),
      refereeMaxPower(0.0f),
      userConfiguredMaxPower(0.0f),
      callback(nullptr),
      torqueConst(0.0f),
      k1(k1_),
      k2(k2_),
      k3(k3_),
      lastUpdateTick(0),
      rls(1e-5f, 0.99999f)
{
    configASSERT(k1_ >= 0);
    configASSERT(k2_ >= 0);
    configASSERT(k3_ >= 0);

    float initParams[2] = {k1_, k2_};
    rls.setParamVector(Matrixf<2, 1>(initParams));
}

static bool isInitialized;
StackType_t xPowerTaskStack[1024];
StaticTask_t uxPowerTaskTCB;

using namespace Core::Communication;
RefereeSystem::RefereeSystemMessageReceive<RefereeSystem::RefereePowerHeatMessageData> powerMessage;
RefereeSystem::RefereeSystemMessageReceive<RefereeSystem::RefereeRobotStatusMessageData> robotMessage;

#if USE_SUPER_CAPACITOR
const Core::Control::SuperCapacitor::CapacitorStatus &capStatus = Core::Control::SuperCapacitor::getStatus();
#endif

Core::Control::PID::Param powerPDParam(POWER_PD_KP, 0.0f, 0.2f, 0.0f, MAX_CAP_POWER_OUT, 0.0f, 0.0f, 0.2f, 100UL);
Core::Control::PID powerPD_base(powerPDParam);  // ensure the system does not die
Core::Control::PID powerPD_full(powerPDParam);  // ensure the capacitor's energy keeps under 90%

/**
 * @implements
 */
void setMaxPowerConfigured(float maxPower)
{
    manager.userConfiguredMaxPower = Utils::Math::clamp(maxPower, MIN_MAXPOWER_CONFIGURED, manager.powerUpperLimit);
}

void setMode(uint8_t mode) { setMaxPowerConfigured(mode == 1 ? manager.powerUpperLimit : manager.refereeMaxPower); }

void registerPowerCallbackFunc(float (*callback)(void)) { manager.callback = callback; }

/**
 * @implements
 */
void setRLSEnabled(uint8_t enable) { manager.rlsEnabled = static_cast<Manager::RLSEnabled>(enable); }

const volatile PowerStatus &getPowerStatus() { return powerStatus; }

float getLatestFeedbackJudgePowerLimit() { return manager.refereeMaxPower; }

/**
 * @implements
 */

float *getControlledOutput(PowerObj *objs[4])
{
    const float k0 = manager.torqueConst * manager.motors[0]->getCurrentLimit() /
                     manager.motors[0]->getOutputLimit();  // torque current rate of the motor, defined as Nm/Output

    static float newTorqueCurrent[4];

    float sumCmdPower = 0.0f;
    float cmdPower[4];

    float sumError = 0.0f;
    float error[4];

    float maxPower = Utils::Math::clamp(manager.userConfiguredMaxPower, manager.fullMaxPower, manager.baseMaxPower);

    float allocatablePower = maxPower;
    float sumPowerRequired = 0.0f;
#if USE_DEBUG
    static float newCmdPower;
#endif

    for (int i = 0; i < 4; i++)
    {
        if (isMotorConnected(manager.motors[i]))
        {
            PowerObj *p = objs[i];
            cmdPower[i] = p->pidOutput * k0 * p->curAv + fabs(p->curAv) * manager.k1 + p->pidOutput * k0 * p->pidOutput * k0 * manager.k2 +
                          manager.k3 / static_cast<float>(4);
            sumCmdPower += cmdPower[i];
            error[i] = fabs(p->setAv - p->curAv);
            if (floatEqual(cmdPower[i], 0.0f) || cmdPower[i] < 0.0f)
            {
                allocatablePower += -cmdPower[i];
            }
            else
            {
                sumError += error[i];
                sumPowerRequired += cmdPower[i];
            }
        }
        else if (motorDisconnectCounter[i] < 1000U)
        {
            cmdPower[i] = manager.motors[i]->getTorqueFeedback() * rpm2av(manager.motors[i]->getRPMFeedback()) +
                          fabs(rpm2av(manager.motors[i]->getRPMFeedback())) * manager.k1 +
                          manager.motors[i]->getTorqueFeedback() * manager.motors[i]->getTorqueFeedback() * manager.k2 + manager.k3 / 4.0f;
            error[i] = 0.0f;
        }
        else
        {
            cmdPower[i] = 0.0f;
            error[i]    = 0.0f;
        }
    }

    // update power status
    powerStatus.maxPowerLimited          = maxPower;
    powerStatus.sumPowerCmd_before_clamp = sumCmdPower;

    if (sumCmdPower > maxPower)
    {
        float errorConfidence;
        if (sumError > error_powerDistribution_set)
        {
            errorConfidence = 1.0f;
        }
        else if (sumError > prop_powerDistribution_set)
        {
            errorConfidence =
                Utils::Math::clamp((sumError - prop_powerDistribution_set) / (error_powerDistribution_set - prop_powerDistribution_set), 0.0f, 1.0f);
        }
        else
        {
            errorConfidence = 0.0f;
        }
        for (int i = 0; i < 4; i++)
        {
            PowerObj *p = objs[i];
            if (isMotorConnected(manager.motors[i]))
            {
                if (floatEqual(cmdPower[i], 0.0f) || cmdPower[i] < 0.0f)
                {
                    newTorqueCurrent[i] = p->pidOutput;
                    continue;
                }
                float powerWeight_Error = fabs(p->setAv - p->curAv) / sumError;
                float powerWeight_Prop  = cmdPower[i] / sumPowerRequired;
                float powerWeight       = errorConfidence * powerWeight_Error + (1.0f - errorConfidence) * powerWeight_Prop;
                float delta             = p->curAv * p->curAv -
                              4.0f * manager.k2 * (manager.k1 * fabs(p->curAv) + manager.k3 / static_cast<float>(4) - powerWeight * allocatablePower);
                if (floatEqual(delta, 0.0f))  // repeat roots
                {
                    newTorqueCurrent[i] = -p->curAv / (2.0f * manager.k2) / k0;
                }
                else if (delta > 0.0f)  // distinct roots
                {
                    newTorqueCurrent[i] = p->pidOutput > 0.0f ? (-p->curAv + sqrtf(delta)) / (2.0f * manager.k2) / k0
                                                              : (-p->curAv - sqrtf(delta)) / (2.0f * manager.k2) / k0;
                }
                else  // imaginary roots
                {
                    newTorqueCurrent[i] = -p->curAv / (2.0f * manager.k2) / k0;
                }
                newTorqueCurrent[i] = Utils::Math::clamp(newTorqueCurrent[i], p->pidMaxOutput);
            }
            else
            {
                newTorqueCurrent[i] = 0.0f;
            }
        }
    }
    else
    {
        for (int i = 0; i < 4; i++)
        {
            if (isMotorConnected(manager.motors[i]))
            {
                newTorqueCurrent[i] = objs[i]->pidOutput;
            }
            else
            {
                newTorqueCurrent[i] = 0.0f;
            }
        }
    }

#if USE_DEBUG
    newCmdPower = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        PowerObj *p = objs[i];
        newCmdPower += newTorqueCurrent[i] * k0 * p->curAv + fabs(p->curAv) * manager.k1 +
                       newTorqueCurrent[i] * k0 * newTorqueCurrent[i] * k0 * manager.k2 + manager.k3 / 4.0f;
    }
#endif

    return newTorqueCurrent;
}

static inline void setErrorFlag()
{
    /*Judge the error status*/
#if USE_SUPER_CAPACITOR
    if (not capStatus.isConnected || not capStatus.capacitorTx.enableDCDC || not capStatus.capacitorRx.errorCode == 0)
        setErrorFlag(manager.error, Manager::CAPDisConnect);
    else
        clearErrorFlag(manager.error, Manager::CAPDisConnect);
#else
    setErrorFlag(manager.error, Manager::CAPDisConnect);
#endif

    if (not RefereeSystem::isConnected())
        setErrorFlag(manager.error, Manager::RefereeDisConnect);
    else
        clearErrorFlag(manager.error, Manager::RefereeDisConnect);

    if (not isAllMotorConnected())
        setErrorFlag(manager.error, Manager::MotorDisconnect);
    else
        clearErrorFlag(manager.error, Manager::MotorDisconnect);
}

void powerDaemon [[noreturn]] (void *pvParam)
{
    static Matrixf<2, 1> samples;
    static Matrixf<2, 1> params;
    static float effectivePower = 0;

    manager.torqueConst = manager.motors[0]->getKA() * manager.motors[0]->getReductionRatio();
    isInitialized       = true;

    vTaskDelay(1000);

    manager.lastUpdateTick = xTaskGetTickCount();

    while (true)
    {
        setErrorFlag();
        TickType_t now = xTaskGetTickCount();

        // update rls state and check whether cap energy is out even when cap disconnect to utilize credible data from referee system for the rls
        // model
        // estimate the cap energy if cap disconnect
        // estimated cap energy = cap energy feedback when cap is connected
#if USE_SUPER_CAPACITOR
        // If super capacitor is disconnected from the circuit, disable the rls update
        if (isFlagged(manager.error, Manager::CAPDisConnect))
        {
            // Judge whether the cap energy is used-up
            if (not isFlagged(manager.error, Manager::RefereeDisConnect))
            {
                if (powerMessage.getData().bufferEnergy < MAX_POEWR_REFEREE_BUFF &&
                    powerMessage.getData().chassisPower > CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD)
                {
                    isCapEnergyOut             = true;
                    manager.estimatedCapEnergy = 0.0f;
                }
                else
                {
                    isCapEnergyOut     = false;
                    manager.rlsEnabled = Manager::Disable;
                    if (powerMessage.getData().chassisPower < MIN_MAXPOWER_CONFIGURED && powerMessage.getData().bufferEnergy == 60U)
                    {
                        manager.estimatedCapEnergy = 2100.0f;
                    }
                    else
                    {
                        manager.estimatedCapEnergy += (powerMessage.getData().chassisPower - manager.estimatedPower) *
                                                      static_cast<float>((now - manager.lastUpdateTick) / configTICK_RATE_HZ);
                        manager.estimatedCapEnergy = Utils::Math::clamp(manager.estimatedCapEnergy, 0.0f, 2100.0f);
                    }
                }
            }
            else
            {
                isCapEnergyOut     = false;
                manager.rlsEnabled = Manager::Disable;
                manager.estimatedCapEnergy += (CAP_OFFLINE_ENERGY_TARGET_POWER - manager.estimatedPower) *
                                              static_cast<float>((now - manager.lastUpdateTick) / configTICK_RATE_HZ);
                manager.estimatedCapEnergy = Utils::Math::clamp(manager.estimatedCapEnergy, 0.0f, 2100.0f);
            }
        }
        else
        {
            isCapEnergyOut             = false;
            manager.estimatedCapEnergy = capStatus.capacitorRx.capEnergy / 255.0f * 2100.0f;
        }
#else  // Only Use Referee System, disable the rls update if referee data is invalid
        if (isFlagged(manager.error, Manager::RefereeDisConnect))
        {
            manager.rlsEnabled = Manager::Disable;
        }
        isCapEnergyOut             = false;
        manager.estimatedCapEnergy = 0.0f
#endif

// Set the power buff and buff set based on the current state
// Take cap message as priority
// If disconnect from cap or disable the cap, then take the referee system's power buffer as feedback
// If referee system is disconnected, then we need to disable the energy loop and treat power loop conservatively
// When both cap and referee are disconnected, we disable the energy loop and therefore no need to update the powerBuff and buffSet
//
// Set the energy feedback based on the current error status
#if USE_SUPER_CAPACITOR
        if (not isFlagged(manager.error, Manager::CAPDisConnect))
            manager.powerBuff = capStatus.capacitorRx.capEnergy;
        else if (not isFlagged(manager.error, Manager::RefereeDisConnect))
            manager.powerBuff = powerMessage.getData().bufferEnergy;
#else
                                     if (not isFlagged(manager.error, Manager::RefereeDisConnect))
                                         manager.powerBuff = powerMessage.getData().bufferEnergy;
#endif
// Set the energy target based on the current error status
#if USE_SUPER_CAPACITOR
        // If the Super Capacitor is in the circuit
        if (not isFlagged(manager.error, Manager::CAPDisConnect))
        {
            manager.fullBuffSet = capFullBuffSet;
            manager.baseBuffSet = capBaseBuffSet;
        }
        else
        {
            // if referee data is not valid, we do not enable the energy loop, so that we do not have to update fullbuffset and basebuffset
            manager.fullBuffSet = refereeFullBuffSet;
            manager.baseBuffSet = refereeBaseBuffSet;
        }
#else
        // Only Use Referee System
        // if referee data is not valid, we do not enable the energy loop, so that we do not have to updating fullbuffset and basebuffset
        manager.fullBuffSet = refereeFullBuffSet;
        manager.baseBuffSet = refereeBaseBuffSet;
#endif

        // Update the referee maximum power limit and user configured power limit
        // If disconnected, then restore the last robot level and find corresponding chassis power limit
        if (not isFlagged(manager.error, Manager::RefereeDisConnect))
        {
            manager.refereeMaxPower = fmax(robotMessage.getData().chassis_power_limit, CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD);
            if (robotMessage.getData().robot_level > 10U)
                LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL = 1U;
            else
                LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL = fmax(1U, robotMessage.getData().robot_level);
#if USE_SUPER_CAPACITOR
            if (isFlagged(manager.error, Manager::CAPDisConnect))
                manager.powerUpperLimit = manager.refereeMaxPower + POWER_PD_KP * (sqrtf(refereeFullBuffSet) - sqrtf(refereeBaseBuffSet));
            else
                manager.powerUpperLimit = manager.refereeMaxPower + MAX_CAP_POWER_OUT;
#else
            manager.powerUpperLimit = manager.refereeMaxPower;
#endif
        }
        else
        {
            switch (manager.division)
            {
            case Division::HERO:
                manager.refereeMaxPower = HeroChassisPowerLimit_HP_FIRST[LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL - 1U];
                break;
            case Division::INFANTRY:
                manager.refereeMaxPower = InfantryChassisPowerLimit_HP_FIRST[LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL - 1U];
                break;
            case Division::SENTRY:
                manager.refereeMaxPower = SentryChassisPowerLimit;
                break;
            default:
                configASSERT(0) break;
            }
// Since we have less available feedback, we constrain the power conservatively
#if USE_SUPER_CAPACITOR
            if (isFlagged(manager.error, Manager::CAPDisConnect))
            {
                manager.powerUpperLimit = manager.refereeMaxPower * CAP_REFEREE_BOTH_GG_COE;
            }
            else
            {
                manager.powerUpperLimit = manager.refereeMaxPower + MAX_CAP_POWER_OUT;
            }
#else
            manager.powerUpperLimit = manager.refereeMaxPower * CAP_REFEREE_BOTH_GG_COE;
#endif
        }

        MIN_MAXPOWER_CONFIGURED = manager.refereeMaxPower * 0.8f;

        // energy loop
        // if cap and referee both gg, set the max power to latest power limit * 0.85 and disable energy loop
        // if referee gg, set the max power to latest power limit * 0.95, enable energy loop when cap energy out
        if (isFlagged(manager.error, Manager::CAPDisConnect) && isFlagged(manager.error, Manager::RefereeDisConnect))
        {
            manager.baseMaxPower = manager.fullMaxPower = manager.refereeMaxPower * CAP_REFEREE_BOTH_GG_COE;
            powerPD_base.reset();
            powerPD_full.reset();
        }
        else
        {
            manager.baseMaxPower =
                fmax(manager.refereeMaxPower - powerPD_base(sqrtf(manager.baseBuffSet), sqrtf(manager.powerBuff)), MIN_MAXPOWER_CONFIGURED);
            manager.fullMaxPower =
                fmax(manager.refereeMaxPower - powerPD_full(sqrtf(manager.fullBuffSet), sqrtf(manager.powerBuff)), MIN_MAXPOWER_CONFIGURED);
        }

        // if user has self defined power curve, use it
        if (manager.callback != nullptr)
            setMaxPowerConfigured(manager.callback());

        // Estimate the power based on the current model
        effectivePower = 0;
        samples[0][0]  = 0;
        samples[1][0]  = 0;
        for (int i = 0; i < 4; i++)
        {
            if (isMotorConnected(manager.motors[i]))
            {
                motorDisconnectCounter[i] = 0U;
            }
            else
            {
                motorDisconnectCounter[i]++;
            }
            if (motorDisconnectCounter[i] < 1000U)  // We consider motor that is just disconnected as still using power, by assuming the motor keep
                                                    // latest output and rpm by 1 second, otherwise it is not safe if we only use energy loop
            {
                effectivePower += manager.motors[i]->getTorqueFeedback() * rpm2av(manager.motors[i]->getRPMFeedback());
                samples[0][0] += fabsf(rpm2av(manager.motors[i]->getRPMFeedback()));
                samples[1][0] += manager.motors[i]->getTorqueFeedback() * manager.motors[i]->getTorqueFeedback();
            }
            else
            {
                motorDisconnectCounter[i] = 1000U;
            }
        }
        manager.estimatedPower = manager.k1 * samples[0][0] + manager.k2 * samples[1][0] + effectivePower + manager.k3;

// Get the measured power from cap
// If cap is disconnected, get measured power from referee feedback if cap energy is out
// Otherwise, set it to estimated power
#if USE_SUPER_CAPACITOR
        if (not isFlagged(manager.error, Manager::CAPDisConnect))
        {
            manager.measuredPower = capStatus.capacitorRx.chassisPower;
        }
        else if (not isFlagged(manager.error, Manager::RefereeDisConnect) && isCapEnergyOut)
        {
            // If the capacitor energy is used up, we could trust the data from the referee system
            manager.measuredPower = powerMessage.getData().chassisPower;
        }
        else
        {
            manager.measuredPower = manager.estimatedPower;
        }
#else
        if (not isFlagged(manager.error, Manager::RefereeDisConnect))
        {
            manager.measuredPower = powerMessage.getData().chassisPower;
        }
        else
        {
            manager.measuredPower = manager.estimatedPower;
        }
#endif

        // update power status
        powerStatus.userConfiguredMaxPower = manager.userConfiguredMaxPower;
        powerStatus.effectivePower         = effectivePower;
        powerStatus.powerLoss              = manager.measuredPower - effectivePower;
        powerStatus.efficiency             = Utils::Math::clamp(effectivePower / manager.measuredPower, 0.0f, 1.0f);
        powerStatus.estimatedCapEnergy     = static_cast<uint8_t>(manager.estimatedCapEnergy / 2100.0f * 255.0f);
        powerStatus.error                  = static_cast<Manager::ErrorFlags>(manager.error);

        // Update the RLS parameters AND
        // Add dead zone AND
        // The Referee System could not detect negative power, leading to failure of real measurement.
        // So use estimated power to evaluate this situtation
        if (manager.rlsEnabled == Manager::Enable && fabs(manager.measuredPower) > 5.0f &&
            not(isFlagged(manager.error, Manager::CAPDisConnect) && manager.estimatedPower < 0))
        {
            params     = manager.rls.update(samples, manager.measuredPower - effectivePower - manager.k3);
            manager.k1 = fmax(params[0][0], 1e-5f);  // In case the k1 diverge to negative number
            manager.k2 = fmax(params[1][0], 1e-5f);  // In case the k2 diverge to negative number
        }

        manager.lastUpdateTick = now;

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/**
 * @implements
 */
void init(const Manager &mana)
{
    if (isInitialized)
        return;
    manager = mana;

    // default value
    LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL = manager.division == Division::SENTRY ? 10U : 1U;
    MIN_MAXPOWER_CONFIGURED           = 30.0f;
    manager.powerUpperLimit           = CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD + MAX_CAP_POWER_OUT;

    Core::Communication::RefereeSystem::subscribeMessage(&powerMessage);
    Core::Communication::RefereeSystem::subscribeMessage(&robotMessage);

    xTaskCreateStatic(powerDaemon, "power", 1024, nullptr, 10, xPowerTaskStack, &uxPowerTaskTCB);
}

}  // namespace Power
}  // namespace Control
}  // namespace Core

#endif