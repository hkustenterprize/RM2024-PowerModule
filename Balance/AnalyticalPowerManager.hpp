#pragma once

#include "../Core/Inc/AppConfig.h"
#include "main.h"

namespace Chassis
{
namespace AnalyticalPowerManager
{

class Manager
{
   public:
    enum MODE : uint8_t
    {
        BATTERY = 0,
        CAP
    };

    static void setMode(MODE currentMode);

    static void update(float chassisTargetSpeed,
                       float chassisCurrentSpeed,
                       float Uspeed,
                       float Uyaw,
                       float wLeftWheel,
                       float leftUelse,
                       float wRightWheel,
                       float rightUelse);

    static float getDecayUspeed();

    static float getDecayUyaw();

    static float getConfiguredMaxPower();

    friend void powerControllTask(void *pvParameters);

   private:
    enum ErrorFlags
    {
        MotorDisconnect   = 1U,
        RefereeDisConnect = 2U,
        CAPDisConnect     = 4U
    };

    float refereeMaxPower;     // 裁判系统最大上限功率
    float configuredMaxPower;  // 能量环最大允许功率

    float powerBuff;  // 当前电容剩余能量
    float buffSet;    // 期望剩余能量

    float chassisPower;  // 当前底盘功率
    float cmdPower;      // 当前控制功率
    float restrictedPower;

    float wLeftWheel;   // 左轮角速度 rad/s
    float wRightWheel;  // 左轮角速度 rad/s

    float chassisTargetSpeed;   // 目标车速
    float chassisCurrentSpeed;  // 当前车速

    float leftUelse;   // Uleg + Upitch
    float rightUelse;  // Uleg + Upitch

    float Uspeed;
    float Uyaw;

    float restrictedUspeed;
    float restrictedUyaw;

    float decayUspeed;
    float decayUyaw;

    float A;
    float B;
    float C;
    float Delta;

    const float k1 = 0.2f;
    const float k2 = 2.8f;
    const float k3 = 3.21f;

    float K;  // Uspeed = K * Uyaw

    MODE mode;
};

void init();

}  // namespace AnalyticalPowerManager
}  // namespace Chassis