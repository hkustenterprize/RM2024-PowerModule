#pragma once

#include "AbstractFeedbackMotor.hpp"
#include "PID.hpp"
#include "SGUtil.hpp"
namespace SGChassis
{
namespace SGPowerControl
{

class PowerLimitator
{
   public:
    PowerLimitator(float k0_,
                   float k1_,
                   float k2_,
                   float k3_,
                   const SGUtil::Quadruple<AbstractFeedbackMotor *> &motors_,
                   const SGUtil::Quadruple<Core::Control::PID *> &pids_);

    void getDecayCurrent(SGUtil::Quadruple<float> &decayCurrent);

    void updateStatus(float updatedMaxPower);

    float getCommandPower() const { return this->commandPower; }

    float getEstimatedPower() const { return this->estimatedPower; }

   private:
    // ============================================   //
    // Basic Model Parameters and Configuration       //
    // ============================================   //

    // P = τΩ + k1|Ω| + k2τ^2 + k3
    const float k0, k1, k2, k3;  // The model params
    // k0: Torque constant, Nm/A
    // k1: Friction constant
    // k2: Joules constant
    // k3: Constant heat loss

    SGUtil::Quadruple<AbstractFeedbackMotor *> motors;
    SGUtil::Quadruple<Core::Control::PID *> pids;

    // ============================= //
    // Quantities for power control  //
    // ============================= //

    float maxPower = 0;

    enum class PowerControlStatus
    {
        NOT_RESTRICTED,
        RESTRICTED
    } powerControlStatus = PowerControlStatus::NOT_RESTRICTED;

    // ============================= //
    // Power Estimator               //

    // static void powerDaemon(void *pvParam);

    // const bool isEstimatorEnabled;

    SGUtil::Quadruple<float> cmdPower;
    SGUtil::Quadruple<float> currentPower;

    float estimatedPower = 0;
    float commandPower   = 0;

    // ============================== //
    // Helper Function                //
    // ============================== //

    float current2Torque(float current) { return current * k0; }
};

}  // namespace SGPowerControl
}  // namespace SGChassis