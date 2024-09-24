#pragma once

#include "AbstractFeedbackMotor.hpp"
#include "PID.hpp"
#include "SGPowerLimitator.hpp"
#include "SGUtil.hpp"

namespace SGChassis
{
namespace SGChassisControl
{

constexpr static float refereeFullBuffSet          = 60.0f;
constexpr static float refereeBaseBuffSet          = 50.0f;
constexpr static float capFullBuffSet              = 230.0f;
constexpr static float capBaseBuffSet              = 30.0f;
constexpr static float error_powerDistribution_set = 20.0f;
constexpr static float prop_powerDistribution_set  = 15.0f;

// constexpr float MIN_MAXPOWER_CONFIGURED                   = 15.0f;
constexpr float MAX_CAP_POWER_OUT                         = 300.0f;
constexpr float CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD = 43.0f;
constexpr float CAP_OFFLINE_ENERGY_TARGET_POWER           = 37.0f;
constexpr float MAX_POEWR_REFEREE_BUFF                    = 60.0f;
constexpr float REFEREE_GG_COE                            = 0.95f;
constexpr float CAP_REFEREE_BOTH_GG_COE                   = 0.85f;

constexpr static uint8_t maxLevel                                     = 10U;
constexpr static uint8_t InfantryChassisPowerLimit_HP_FIRST[maxLevel] = {45U, 50U, 55U, 60U, 65U, 70U, 75U, 80U, 90U, 100U};

struct SteeringGearConfig
{
    float length;
    float diameter;
    struct Dynamics
    {
        float x;
        float y;
        float w;
    };
};

class SteeringGearController
{
   public:
    SteeringGearController() = delete;

    SteeringGearController(const SteeringGearController &) = delete;

    SteeringGearController(const SGUtil::Quadruple<AbstractFeedbackMotor *> &steerMotor_,
                           const SGUtil::Quadruple<Core::Control::PID *> &steerPID_,
                           const SGUtil::Quadruple<AbstractFeedbackMotor *> &wheelMotor_,
                           const SGUtil::Quadruple<Core::Control::PID *> &wheelPID_,
                           const SteeringGearConfig &config);

    void setTargetVelocity(float vx, float vy, float vw);

    void update();

    void setCapEnabled(bool enabled) { this->capEnabled = enabled; }

    void setLowPowerChargingEnabled(bool enabled) {this->lowPowerCapChargingMode = enabled;}

    void setCapMode(uint8_t mode) {this -> capMode = mode;}

    static uint8_t getLatestFeedbackJudgePowerLimit() { return refereeMaxPower; }

    static uint8_t getEstimatedCapEnergy() { return static_cast<uint8_t>(estimatedCapEnergy / 2100.0f * 255.0f); }

    const SteeringGearConfig::Dynamics &getMeasuredVelocity(void);

   private:
    void updatePowerStatus();
    void solveMeasuredDynamics();

    SGUtil::Quadruple<AbstractFeedbackMotor *> steerMotor;
    SGUtil::Quadruple<Core::Control::PID *> steerPID;
    SGUtil::Quadruple<AbstractFeedbackMotor *> wheelMotor;
    SGUtil::Quadruple<Core::Control::PID *> wheelPID;

    // Target velocity
    SGUtil::Quadruple<float> wheelsTargetAvX;
    SGUtil::Quadruple<float> wheelsTargetAvY;
    SGUtil::Quadruple<float> wheelsTargetAv;
    SGUtil::Quadruple<float> steersTargetPosition;
    SGUtil::Quadruple<float> lastSteersTargetPosition;

    // Current Feedback
    SGUtil::Quadruple<float> wheelsFeedbackAv;
    SGUtil::Quadruple<float> steersFeedbackPosition;
    SGUtil::Quadruple<float> lastSteersFeedbackPosition;

    SGUtil::Quadruple<float> steersError;
    SGPowerControl::PowerLimitator steerLimitator;
    SGPowerControl::PowerLimitator wheelLimitator;

    SteeringGearConfig config;

    float targetVx = 0, targetVy = 0, targetVw = 0;

    bool capEnabled;
    bool lowPowerCapChargingMode {false};
    uint8_t capMode;

    static float measuredPower;
    static float estimatedPower;
    static float refereeMaxPower;

    static float estimatedCapEnergy;

    TickType_t lastUpdateTick;

    float currentAvX[4];
    float currentAvY[4];

    SteeringGearConfig::Dynamics measuredVelocity;
};

}  // namespace SGChassisControl
}  // namespace SGChassis