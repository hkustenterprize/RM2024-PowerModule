#include "SGPowerLimitator.hpp"

#include "FreeRTOS.h"
#include "task.h"
namespace SGChassis
{
namespace SGPowerControl
{
    
PowerLimitator::PowerLimitator(float k0_,
                               float k1_,
                               float k2_,
                               float k3_,
                               const SGUtil::Quadruple<AbstractFeedbackMotor *> &motors_,
                               const SGUtil::Quadruple<Core::Control::PID *> &pids_)
    : k0(k0_), k1(k1_), k2(k2_), k3(k3_), motors(motors_), pids(pids_)
{
}

void PowerLimitator::updateStatus(float updatedMaxPower)
{
    this->maxPower = updatedMaxPower;

    // Calculate the estimated power
    float ePower = 0, cPower = 0;

    for (int i = 0; i < 4; ++i)
    {
        float w = SGUtil::rpm2rad(motors[i]->getRPMFeedback());
        float t = motors[i]->getTorqueFeedback();

        // Current Power
        this->currentPower[i] = t * w + this->k1 * fabs(w) + this->k2 * t * t + this->k3 / 4.0f;
        ePower += this->currentPower[i];

        // Command Power
        float ct          = current2Torque(pids[i]->getState().output);
        this->cmdPower[i] = ct * w + this->k1 * fabs(w) + ct * ct * this->k2 + this->k3 / 4.0f;
        cPower += this->cmdPower[i];
    }

    this->estimatedPower = ePower;
    this->commandPower   = cPower;


}

void PowerLimitator::getDecayCurrent(SGUtil::Quadruple<float> &decayCurrent)
{
    float allocatablePower = this->maxPower, powerSumRequired = 0;

    for (int i = 0; i < 4; i++)
    {
        if (this->cmdPower[i] > 0.0f)
        {
            powerSumRequired += this->cmdPower[i];
        }
        else
        {
            allocatablePower -= this->cmdPower[i];
        }
    }

    // Start to calculate the decay current, if the power is over the limit
    if (this->commandPower > this->maxPower)
    {
        this->powerControlStatus = PowerControlStatus::RESTRICTED;
        for (int i = 0; i < 4; i++)
        {
            if (SGUtil::floatEqual(cmdPower[i], 0.0f) || cmdPower[i] < 0.0f)
            {
                continue;
            }
            float curAv       = SGUtil::rpm2rad(motors[i]->getRPMFeedback());
            float powerWeight = cmdPower[i] / powerSumRequired;
            float delta       = curAv * curAv - 4.0f * k2 * (k1 * fabs(curAv) + k3 / 4.0f - powerWeight * allocatablePower);
            if (SGUtil::floatEqual(delta, 0.0f))  // repeat roots
            {
                decayCurrent[i] = -curAv / (2.0f * k2) / k0;
            }
            else if (delta > 0.0f)  // distinct roots
            {
                decayCurrent[i] = decayCurrent[i] > 0.0f ? (-curAv + sqrtf(delta)) / (2.0f * k2) / k0 : (-curAv - sqrtf(delta)) / (2.0f * k2) / k0;
            }
            else  // imaginary roots
            {
                decayCurrent[i] = -curAv / (2.0f * k2) / k0;
            }
        }
    }
    else
    {
        this->powerControlStatus = PowerControlStatus::NOT_RESTRICTED;
    }
}

}  // namespace SGPowerControl
}  // namespace SGChassis