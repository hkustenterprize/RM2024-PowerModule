/**
 * @file ChassisController.hpp
 * @author GUO, Zilin RM2023
 * @brief The first version of chassis controller, coupled with power controller;
 * @version 1.0
 * @date 2023-5-31
 * @example
 * @copyright Copyright (c) 2023
 */

#pragma once
#include "AppConfig.h"

#if USE_CHASSIS_CONTROLLER

#include "AbstractFeedbackMotor.hpp"
#include "PID.hpp"

#if (!defined USE_OMNI_CHASSIS) && (!defined USE_MECANUM_CHASSIS)
#define USE_MECANUM_CHASSIS
#endif

using namespace Core::Control;
namespace Core
{
namespace Control
{
namespace Chassis
{
/**
 * @brief The frame of chassis controller
 *
 * The coordinate system for chassis
 *           ^ Y
 * 1         |        0
 *           |
 *           |
 * ----------|----------> X
 *           |
 *           |
 * 2         |        3
 * vw is positive for anti-clockwise rotate
 */

/**
 * @brief  Structure that stores the measured property datas of the controled chassis
 * @remark Use Macro USE_OMNI_CHASSIS or USE_MECANUM_CHASSIS to determine the compiling data member
 * @param chassisWheelDiameters         The diameters of the wheels (unit : m)
 * @if USE_MECANUM_CHASSIS
 * @param chassisRightLeftLength        The distance between where the right motor touches the ground and where left motor at the same side touches
 * the ground
 * @param chassisFrontBackLength        The distance between where the front motor touches the ground and where the back motor at the same side
 * touches the ground
 * @else USE_OMNI_CHASSIS
 * @remark This configuration assumes that the chassis are circle
 * @param chassisWheelToCentreLength    The distance between where every motor touches the ground and the chassis centre.
 */
struct ChassisPropertyConfig
{
    ChassisPropertyConfig() = delete;
    float chassisWheelDiameters;

#if USE_MECANUM_CHASSIS
    float chassisRightLeftLength;
    float chassisFrontBackLength;
    ChassisPropertyConfig(float wheelDiameters, float fbLength, float rlLength)
        : chassisWheelDiameters(wheelDiameters), chassisRightLeftLength(rlLength), chassisFrontBackLength(fbLength)
    {
    }
#else
    float chassisWheelToCentreLength;
    ChassisPropertyConfig(float wheelDiameters, float wheelToCentre)
        : chassisWheelDiameters(wheelDiameters), chassisWheelToCentreLength(wheelToCentre)
    {
    }
#endif
};

/**
 * @brief Main chassis configurations data sheets, which is used to intialize the controller object
 */
class ChassisConfigs
{
   public:
    /**
     * @brief Chassis Dynamics
     * @param x
     * @param y
     * @param w
     */
    struct Dynamics
    {
        float x;
        float y;
        float w;
    };
    /**
     * @attention In case of construction without explicit datas
     */
    ChassisConfigs() = delete;

    /**
     * @brief Constructor function
     * @param motors                    The pointer data of chassis motors driver, assuming they are all inherited from AbstractFeedbackMotors
     * @param pids                      The pointer data package of PID
     * @param property                  The physical property datas from the controlled chassis
     * @param maxAcceleration_          The configurated maximum acceleration rate of ax, ay
     * @param maxAngularAcceleration_   The configurated maximum angular accleration rate of aw
     * @param maxSpeed_                 The configurated maximum speed of vx, vy, vy
     */
    ChassisConfigs(AbstractFeedbackMotor *pMotorRf,
                   AbstractFeedbackMotor *pMotorLf,
                   AbstractFeedbackMotor *pMotorLb,
                   AbstractFeedbackMotor *pMotorRb,

                   PID *pPIDRf,
                   PID *pPIDLf,
                   PID *pPIDLb,
                   PID *pPIDRb,

                   const ChassisPropertyConfig &property,
                   const float maxAcceleration_,
                   const float maxAngularAcceleration_,
                   const Dynamics &maxSpeed_);

    /**
     * @remark Ensures only the controller object with its given function could visit the motors data.
     */
    friend class ChassisController;

   private:
    /**
     * @brief Chassis motors relevant datas standard Structure
     * @tparam T    The type of the motor datas (e.g. AbstractFeedbackMotor, PID)
     * @note This is a container structure
     * @param motorRf  Motors installed on the right front postition
     * @param motorLf  Motors installed on the left front position
     * @param motorLb  Motors installed on the left back position
     * @param motorRb  Motors installed on the right back position
     */
    template <typename T>
    struct ChassisMotorsData
    {
        T motorRf;
        T motorLf;
        T motorLb;
        T motorRb;

        /**
         * @attention In case of construction without explicit datas
         */
        ChassisMotorsData() = delete;
        ChassisMotorsData(T motorRf_, T motorLf_, T motorLb_, T motorRb_) : motorRf(motorRf_), motorLf(motorLf_), motorLb(motorLb_), motorRb(motorRb_)
        {
        }

        /**
         * @brief To visit the motors at particular sides in convenience using index
         */
        const T &operator[](uint32_t index) const { return (index == 0) ? motorRf : (index == 1) ? motorLf : (index == 2) ? motorLb : motorRb; }
        T &operator[](uint32_t index) { return (index == 0) ? motorRf : (index == 1) ? motorLf : (index == 2) ? motorLb : motorRb; }

        /**
         * @brief Get the pointer of the first element
         * @return const T*
         */
        T *getArray(void) { return &motorRf; }
    } __attribute__((aligned(sizeof(T))));

    const ChassisPropertyConfig chassisPropertyConfig;
    ChassisMotorsData<AbstractFeedbackMotor *> chassisMotors;
    ChassisMotorsData<PID *> chassisPIDs;

    /**
     * @brief Last update tick
     * @remark Base on API xCreateTickCount(), don't use HAL_GetTick()
     */
    TickType_t lastUpdate;

    /**
     * @brief Max output acceleration
     */
    float maxAcceleration;
    float maxAngularAcceleration;

    Dynamics maxSpeed;
};

/**
 * @brief Chassis Controller object
 * @remark Assuming only construct one obejct
 */
class ChassisController
{
   public:
    /**
     * @attention In case of construction without explicit datas
     */
    ChassisController() = delete;
    /**
     * @brief Constructor function
     * @param ChassisConfigs Main chassis Configuration
     */
    ChassisController(const ChassisConfigs &chassisConfigs_);

    /**
     * @brief Set current target speed (unit: m/s, rad/s)
     * @param vx Velocity in x-axis on the moving plane
     * @param vy Velocity in y-axis on the moving plane
     * @param w  Angular velocity on the moving plane
     */
    void settargetVelocity(float vx, float vy, float vw);

    /**
     * @brief Set current target Speed (uint: m/s, rad/s)
     * @param Dynamics
     * @remark Dynamics[0]->vx, Dynamics[1]->vy, Dynamics[2]->w
     */
    void settargetVelocity(float Dynamics[3]);

    /**
     * @brief Set current target command velocity in percentage of the configurated maximum velocity
     * @param vxPercentage The percentage velocity in x-axis on the moving plane
     * @param vyPercentage The percentage velocity in y-axis on the moving plane
     * @param vwPercentage The percentage angular velocity on the moving plane
     * @note  These should fall in the range of [0, 1]
     */
    void setTargetPercentageSpeed(float vxPercentage, float vyPercentage, float vwPercentage);

    /**
     * @brief Update Every tick to calculate and set the PID output on the motors
     * @brief Also, this function will read RPM from 4 wheels and calculate dynamics data.
     *
     * @remark Define macro USE_POWER_CONTROLLER to enable power controller limitation
     */
    void update(void);

    /**
     * @brief The user can stop all the motors and reset PID in case of emergency
     */
    void stop(void);

    /**
     * @brief The user can only get chassis configurations object from this API, but unable to modify them.
     * @return const ChassisConfigs&
     */
    const ChassisConfigs &getConfig(void);

    /**
     * @brief The user can get chassis measured velocity from this API, but unable to modify it
     * @return const ChassisConfigs::Dynamics &
     */
    const ChassisConfigs::Dynamics &getMeasuredVelocity(void);

    /**
     * @brief The user can get chassis measured acceleration from this API, but unable to modify it
     * @return const ChassisConfigs::Dynamics &
     */
    const ChassisConfigs::Dynamics &getMeasuredAccleration(void);

    /**
     * @brief The user can get chassis measured displacement from this API, but unable to modify it
     * @return const ChassisConfigs::Dynamcis &
     */
    const ChassisConfigs::Dynamics &getMeasuredDisplacement(void);

    /**
     * @brief The user is able to modify configurated maximum speed by this API
     * @param newMaxSpeed The new maximum configurated speeds
     */
    void setMaxVelocity(const ChassisConfigs::Dynamics &newMaxSpeed);

    /**
     * @brief The user is able to modify configurated maximum linear acceleration rate by this API
     * @param newMaxiAcceleration   The new maximum configurated acceleration rate of ax, ay
     */
    void setMaxAcceleration(const float newMaxAcceleration);

    /**
     * @brief The user is able to modify configurated maximum linear acceleration rate by this API
     * @param newMaxiAnuglarAcceleration   The new maximum configurated acceleration rate of aw
     */
    void setMaxAngularAcceleration(const float newMaxAngularAcceleration);

   private:
    void solveMeasuredDynamics(void);
    float currentAv[4];
    float diameters;
#if USE_MECANUM_CHASSIS
    float sum;
#else
    float length;
#endif
    float wFactor;
    float xyFactor;
    ChassisConfigs::ChassisMotorsData<float> wheelsTargetAv;
    ChassisConfigs::ChassisMotorsData<float> wheelsOutput;
    ChassisConfigs chassisConfigs;
    /**
     * @brief Chassis Dynamics datas
     */
    ChassisConfigs::Dynamics rawTargetVelocity;
    ChassisConfigs::Dynamics lastRawTargetVelocity;
    ChassisConfigs::Dynamics targetVelocity;
    ChassisConfigs::Dynamics lastTargetVelocity;
    ChassisConfigs::Dynamics measuredVelocity;
    ChassisConfigs::Dynamics lastMeasuredVelocity;
    ChassisConfigs::Dynamics measuredAcceleration;
    ChassisConfigs::Dynamics measuredDisplacement;
};

}  // namespace Chassis
}  // namespace Control
}  // namespace Core
#endif
