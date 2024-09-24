#include "AnalyticalPowerManager.hpp"

#include "BalancedInfantryConfig.hpp"
#include "FreeRTOS.h"
#include "PID.hpp"
#include "RefereeMess.hpp"
#include "SuperCapaManager.hpp"
#include "math.h"
#include "task.h"

namespace Chassis
{
namespace AnalyticalPowerManager
{
using namespace Core::Control;

const TickType_t xTimeInterval = pdMS_TO_TICKS(SYSTEM_TIME_INTERVAL);  // ms

const SuperCapacitor::CapacitorStatus &capStatus = Core::Control::SuperCapacitor::getStatus();

const volatile RefereeMess::RefereeData &refereeDataCopy = RefereeMess::getRefereeData();

Manager manager;

PID::Param powerPDParam(50.0f, 0.0f, 0.20f, 0.0f, 300.0f, 0.0f, 0.0f, 0.2f, 100UL);  // 电容能量环
PID powerPD(powerPDParam);

static inline bool floatEqual(float a, float b) { return fabs(a - b) < 1e-5f; }

void Manager::setMode(MODE currentMode) { manager.mode = currentMode; }

float Manager::getDecayUspeed() { return manager.decayUspeed; }
float Manager::getDecayUyaw() { return manager.decayUyaw; }
float Manager::getConfiguredMaxPower() { return manager.configuredMaxPower; }

void Manager::update(float chassisTargetSpeed,
                     float chassisCurrentSpeed,
                     float Uspeed,
                     float Uyaw,
                     float wLeftWheel,
                     float leftUelse,
                     float wRightWheel,
                     float rightUelse)
{
    manager.chassisTargetSpeed  = chassisTargetSpeed;
    manager.chassisCurrentSpeed = chassisCurrentSpeed;
    manager.Uspeed              = Uspeed;
    manager.Uyaw                = Uyaw;
    manager.wLeftWheel          = wLeftWheel;
    manager.leftUelse           = leftUelse;
    manager.wRightWheel         = wRightWheel;
    manager.rightUelse          = rightUelse;

    if (floatEqual(Uyaw, 0.0f))
    {
        if (Uspeed * Uyaw > 0.0f)
            manager.K = 10000.0f;
        else
            manager.K = -10000.0f;
    }
    else
        manager.K = Uspeed / Uyaw;

    float leftTotalTorque  = Uspeed - Uyaw + leftUelse;
    float rightTotalTorque = Uspeed + Uyaw + rightUelse;

    manager.cmdPower = (manager.k1 * (ABS(wLeftWheel) + ABS(wRightWheel))) + (manager.k2 * (powf(leftTotalTorque, 2) + powf(rightTotalTorque, 2))) +
                       manager.k3 + (wLeftWheel * leftTotalTorque) + (wRightWheel * rightTotalTorque);

    if (manager.cmdPower <= manager.configuredMaxPower)
    {
        manager.restrictedUspeed = manager.Uspeed;
        manager.restrictedUyaw   = manager.Uyaw;

        manager.decayUspeed = 1.0f;
        manager.decayUyaw   = 1.0f;
    }
    else
    {
        float speedError = manager.chassisTargetSpeed - manager.chassisCurrentSpeed;

        if (ABS(speedError) > 0.5f && ABS(manager.chassisCurrentSpeed) > 0.5f &&
            ((manager.chassisTargetSpeed * manager.chassisCurrentSpeed < 0.0f) || (manager.chassisTargetSpeed >= 0.0f && speedError < 0.0f) ||
             (manager.chassisTargetSpeed <= 0.0f && speedError > 0.0f)))
        {
            manager.decayUspeed = manager.decayUspeed * 0.97f + 1.0f * 0.03f;
            manager.decayUyaw   = manager.decayUyaw * 0.97f + 1.0f * 0.03f;
        }
        else
        {
            // calculate the restricted Uspeed and Uyaw
            manager.A = (manager.k2 * (2.0f * powf(manager.K, 2) + 2.0f));
            manager.B = (2.0f * manager.k2 * (manager.K - 1.0f) * manager.leftUelse) + (2.0f * manager.k2 * (manager.K + 1.0f) * manager.rightUelse) +
                        (manager.wLeftWheel * (manager.K - 1.0f)) + (manager.wRightWheel * (manager.K + 1.0f));
            manager.C = (manager.wLeftWheel * manager.leftUelse) + (manager.wRightWheel * manager.rightUelse) +
                        (manager.k1 * (ABS(manager.wLeftWheel) + ABS(manager.wRightWheel))) +
                        (manager.k2 * (powf(manager.leftUelse, 2) + powf(manager.rightUelse, 2))) + manager.k3 - manager.configuredMaxPower;

            manager.Delta = powf(manager.B, 2) - 4 * manager.A * manager.C;

            if (floatEqual(manager.Delta, 0.0f))  // repeat roots
            {
                float tempUyaw = (-manager.B) / (2.0f * manager.A);

                if (tempUyaw * manager.Uyaw > 0.0f)
                    manager.restrictedUyaw = tempUyaw;
                else
                    manager.restrictedUyaw = 0.0f;

                manager.restrictedUspeed = manager.K * manager.restrictedUyaw;
            }
            else if (manager.Delta > 0.0f)  // distinct roots
            {
                float tempUyaw1 = (-manager.B - sqrtf(manager.Delta)) / (2.0f * manager.A);
                float tempUyaw2 = (-manager.B + sqrtf(manager.Delta)) / (2.0f * manager.A);

                if (tempUyaw1 * manager.Uyaw > 0.0f && tempUyaw2 * manager.Uyaw > 0.0f)
                {
                    if (manager.Uyaw > 0.0f)
                        manager.restrictedUyaw = fmax(tempUyaw1, tempUyaw2);
                    else
                        manager.restrictedUyaw = fmin(tempUyaw1, tempUyaw2);
                }
                else if (tempUyaw1 * manager.Uyaw > 0.0f)
                    manager.restrictedUyaw = tempUyaw1;
                else if (tempUyaw2 * manager.Uyaw > 0.0f)
                    manager.restrictedUyaw = tempUyaw2;
                else
                    manager.restrictedUyaw = 0.0f;

                manager.restrictedUspeed = manager.K * manager.restrictedUyaw;
            }
            else if (manager.Delta < 0.0f)  // imaginary roots
            {
                float tempUyaw = (-manager.B) / (2.0f * manager.A);

                if (tempUyaw * manager.Uyaw > 0.0f)
                    manager.restrictedUyaw = tempUyaw;
                else
                    manager.restrictedUyaw = 0.0f;

                manager.restrictedUspeed = manager.K * manager.restrictedUyaw;
            }

            float tempDecayUspeed = manager.restrictedUspeed / Uspeed;
            float tempDecayUyaw   = manager.restrictedUyaw / Uyaw;

            tempDecayUspeed = CLAMP(tempDecayUspeed, 0.01f, 1.00f);
            tempDecayUyaw   = CLAMP(tempDecayUyaw, 0.01f, 1.00f);

            manager.decayUspeed = tempDecayUspeed * 0.1f + manager.decayUspeed * 0.9f;
            manager.decayUyaw   = tempDecayUyaw * 0.1f + manager.decayUyaw * 0.9f;
        }
    }

    float restrictedLeftTotalTorque  = manager.restrictedUspeed - manager.restrictedUyaw + leftUelse;
    float restrictedRightTotalTorque = manager.restrictedUspeed + manager.restrictedUyaw + rightUelse;

    manager.restrictedPower = (manager.k1 * (ABS(wLeftWheel) + ABS(wRightWheel))) +
                              (manager.k2 * (powf(restrictedLeftTotalTorque, 2) + powf(restrictedRightTotalTorque, 2))) + manager.k3 +
                              (wLeftWheel * restrictedLeftTotalTorque) + (wRightWheel * restrictedRightTotalTorque);
}

void powerControllTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    while (true)
    {
        /* update the current power status */
        manager.chassisPower    = capStatus.capacitorRx.chassisPower;
        manager.powerBuff       = capStatus.capacitorRx.capEnergy;
        manager.refereeMaxPower = refereeDataCopy.chassis_power_limit;

        // cap power loop
        if (capStatus.isConnected)
        {
            if (manager.mode == Manager::MODE::BATTERY)
            {
                manager.buffSet = float(255 * 0.8f);
            }
            else
            {
                manager.buffSet = float(255 * 0.4f);
            }

            manager.configuredMaxPower = manager.refereeMaxPower - powerPD(sqrtf(manager.buffSet), sqrtf(manager.powerBuff));
            manager.configuredMaxPower = fmax(manager.configuredMaxPower, 23.0f);
        }
        else
            manager.configuredMaxPower = manager.refereeMaxPower;

        // manager.configuredMaxPower = 60.0f;

        vTaskDelayUntil(&xLastWakeTime, xTimeInterval);
    }
}

StackType_t powerControllTaskStack[128];
StaticTask_t powerControllTaskTCB;
void init() { xTaskCreateStatic(powerControllTask, "PowerControll", 128, nullptr, 10, powerControllTaskStack, &powerControllTaskTCB); }

}  // namespace AnalyticalPowerManager
}  // namespace Chassis