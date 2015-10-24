/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "build_config.h"
#include "platform.h"
#include "debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/navigation_rewrite.h"
#include "flight/navigation_rewrite_private.h"

#include "config/runtime_config.h"
#include "config/config.h"


#if defined(NAV)

/*-----------------------------------------------------------
 * Altitude controller
 *-----------------------------------------------------------*/
void setupFixedWingAltitudeController(void)
{
    // TODO
}

void resetFixedWingAltitudeController()
{
    navPidReset(&posControl.pids.fw_alt);
    posControl.rcAdjustment[PITCH] = 0;
}

static void updateAltitudeTargetFromRCInput_FW(void)
{
    // In some cases pilot has no control over flight direction
    if (!navCanAdjustAltitudeFromRCInput()) {
        posControl.flags.isAdjustingAltitude = false;
        return;
    }

    int16_t rcAdjustment = applyDeadband(rcCommand[PITCH], posControl.navConfig->alt_hold_deadband);

    if (rcAdjustment) {
        // set velocity proportional to stick movement
        float rcClimbRate = -rcAdjustment * posControl.navConfig->max_manual_climb_rate / (500.0f - posControl.navConfig->alt_hold_deadband);
        updateAltitudeTargetFromClimbRate(rcClimbRate);
        posControl.flags.isAdjustingAltitude = true;
    }
    else {
        posControl.flags.isAdjustingAltitude = false;
    }
}

// Position to velocity controller for Z axis
static void updateAltitudeVelocityAndPitchController_FW(uint32_t deltaMicros)
{
    static float velzFilterState;

    // On a fixed wing we might not have a reliable climb rate source (if no BARO available), so we can't apply PID controller to
    // velocity error. We use PID controller on altitude error and calculate desired pitch angle from desired climb rate and forward velocity

    float altitudeError = posControl.desiredState.pos.V.Z - posControl.actualState.pos.V.Z;
    float forwardVelocity = sqrtf(sq(posControl.actualState.vel.V.X) + sq(posControl.actualState.vel.V.Y));
    forwardVelocity = MAX(forwardVelocity, 300.0f);   // Limit min velocity for PID controller at about 10 km/h

    // Calculate max climb rate from current forward velocity and maximum pitch angle
    float maxVelocityZ = forwardVelocity * NAV_ROLL_PITCH_MAX_FW_TAN;

    posControl.desiredState.vel.V.Z = navPidApply2(altitudeError, US2S(deltaMicros), &posControl.pids.fw_alt, -maxVelocityZ, maxVelocityZ);
    posControl.desiredState.vel.V.Z = navApplyFilter(posControl.desiredState.vel.V.Z, NAV_FW_VEL_CUTOFF_FREQENCY_HZ, US2S(deltaMicros), &velzFilterState);

    // Calculate pitch angle (plane should be trimmed to horizontal flight with PITCH=0
    posControl.rcAdjustment[PITCH] = atan2_approx(posControl.desiredState.vel.V.Z, forwardVelocity) / RADX100;
    posControl.rcAdjustment[PITCH] = constrain(posControl.rcAdjustment[PITCH], -NAV_ROLL_PITCH_MAX_FW, NAV_ROLL_PITCH_MAX_FW) * 0.1f;

#if defined(NAV_BLACKBOX)
    navDesiredVelocity[Z] = constrain(lrintf(posControl.desiredState.vel.V.Z), -32678, 32767);
    navLatestPositionError[Z] = constrain(lrintf(altitudeError), -32678, 32767);
    navTargetPosition[Z] = constrain(lrintf(posControl.desiredState.pos.V.Z), -32678, 32767);
#endif
}

void applyFixedWingAltitudeController(uint32_t currentTime)
{
    static navigationTimer_t targetPositionUpdateTimer; // Occurs @ POSITION_TARGET_UPDATE_RATE_HZ
    static uint32_t previousTimePositionUpdate;         // Occurs @ altitude sensor update rate (max MAX_ALTITUDE_UPDATE_RATE_HZ)
    static uint32_t previousTimeUpdate;                 // Occurs @ looptime rate

    uint32_t deltaMicros = currentTime - previousTimeUpdate;
    previousTimeUpdate = currentTime;

    // If last time Z-controller was called is too far in the past - ignore it (likely restarting altitude controller)
    if (deltaMicros > HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
        resetTimer(&targetPositionUpdateTimer, currentTime);
        previousTimeUpdate = currentTime;
        previousTimePositionUpdate = currentTime;
        resetFixedWingAltitudeController();
        return;
    }

    if (posControl.flags.hasValidPositionSensor) {
        // Update altitude target from RC input or RTL controller
        if (updateTimer(&targetPositionUpdateTimer, HZ2US(POSITION_TARGET_UPDATE_RATE_HZ), currentTime)) {
            if (navShouldApplyAutonomousLandingLogic()) {
                // Gradually reduce descent speed depending on actual altitude.
                if (posControl.actualState.pos.V.Z > (posControl.homePosition.pos.V.Z + 5000)) {
                    updateAltitudeTargetFromClimbRate(-300.0f);
                }
                else if (posControl.actualState.pos.V.Z > (posControl.homePosition.pos.V.Z + 1000)) {
                    updateAltitudeTargetFromClimbRate(-100.0f);
                }
                else {
                    updateAltitudeTargetFromClimbRate(-50.0f);
                }
            }

            updateAltitudeTargetFromRCInput_FW();
        }

        // If we have an update on vertical position data - update velocity and accel targets
        if (posControl.flags.verticalPositionNewData) {
            uint32_t deltaMicrosPositionUpdate = currentTime - previousTimePositionUpdate;
            previousTimePositionUpdate = currentTime;

            // Check if last correction was too log ago - ignore this update
            if (deltaMicrosPositionUpdate < HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
                updateAltitudeVelocityAndPitchController_FW(deltaMicrosPositionUpdate);
            }
            else {
                // due to some glitch position update has not occurred in time, reset altitude controller
                resetFixedWingAltitudeController();
            }

            // Indicate that information is no longer usable
            posControl.flags.verticalPositionNewData = 0;
        }

        // Set rcCommand to the desired PITCH angle target
        rcCommand[PITCH] = leanAngleToRcCommand(posControl.rcAdjustment[PITCH]);
    }
    else {
        // No valid altitude sensor data, don't adjust pitch automatically, rcCommand[PITCH] is passed through to PID controller
    }
}

/*-----------------------------------------------------------
 * Calculate rcAdjustment for YAW
 *-----------------------------------------------------------*/
void applyFixedWingHeadingController(void)
{
    if (posControl.flags.headingNewData) {
#if defined(NAV_BLACKBOX)
        navDesiredHeading = constrain(lrintf(posControl.desiredState.yaw), -32678, 32767);
#endif

        // TODO

        // Indicate that information is no longer usable
        posControl.flags.headingNewData = 0;
    }

    // Control yaw by NAV PID
    //rcCommand[YAW] = constrain(posControl.rcAdjustment[YAW], -500, 500);
}

void resetFixedWingHeadingController(void)
{
    // TODO
}

/*-----------------------------------------------------------
 * XY-position controller for multicopter aircraft
 *-----------------------------------------------------------*/
t_fp_vector virtualDesiredPosition;

void resetFixedWingPositionController(void)
{
    virtualDesiredPosition.V.X = 0;
    virtualDesiredPosition.V.Y = 0;
    virtualDesiredPosition.V.Z = 0;

    navPidReset(&posControl.pids.fw_nav);
    posControl.rcAdjustment[ROLL] = 0;
}

static void calculateVirtualPositionTarget_FW(float trackingPeriod)
{
    float posErrorX = posControl.desiredState.pos.V.X - posControl.actualState.pos.V.X;
    float posErrorY = posControl.desiredState.pos.V.Y - posControl.actualState.pos.V.Y;

    float distanceToActualTarget = sqrtf(sq(posErrorX) + sq(posErrorY));
    float forwardVelocity = sqrtf(sq(posControl.actualState.vel.V.X) + sq(posControl.actualState.vel.V.Y));

    // Limit minimum forward velocity to 1 m/s
    forwardVelocity = MAX(forwardVelocity, 100.0f);
    float trackingDistance = trackingPeriod * forwardVelocity;

    // TODO: This code will need to be changed to function properly in waypoint mode

    // Calculate virtual position for straight movement
    #define TAN_5DEG    0.087489f
    if (distanceToActualTarget > (posControl.navConfig->waypoint_radius / TAN_5DEG)) {
        // If angular visibility of a waypoint is less than 10deg, don't calculate circular loiter, go straight to the target
        virtualDesiredPosition.V.X = posControl.actualState.pos.V.X + posErrorX * (trackingDistance / distanceToActualTarget);
        virtualDesiredPosition.V.Y = posControl.actualState.pos.V.Y + posErrorY * (trackingDistance / distanceToActualTarget);
    }
    else {
        // We are closing in on a waypoint, calculate circular loiter
        // Find a point on our circular loiter path closest to the airplane, calculate angle from waypoint venter to airplane
        float loiterTargetAngle = atan2_approx(-posErrorY, -posErrorX) + (trackingDistance / posControl.navConfig->waypoint_radius);
        float loiterTargetX = posControl.desiredState.pos.V.X + posControl.navConfig->waypoint_radius * cos_approx(loiterTargetAngle);
        float loiterTargetY = posControl.desiredState.pos.V.Y + posControl.navConfig->waypoint_radius * sin_approx(loiterTargetAngle);

        // We have temporary loiter target. Recalculate distance and position error
        posErrorX = loiterTargetX - posControl.actualState.pos.V.X;
        posErrorY = loiterTargetY - posControl.actualState.pos.V.Y;
        distanceToActualTarget = sqrtf(sq(posErrorX) + sq(posErrorY));

        // Calculate cirtual waypoint at some close distance in front of the plain
        virtualDesiredPosition.V.X = posControl.actualState.pos.V.X + posErrorX * (trackingDistance / distanceToActualTarget);
        virtualDesiredPosition.V.Y = posControl.actualState.pos.V.Y + posErrorY * (trackingDistance / distanceToActualTarget);
    }

    // Shift position according to pilot's ROLL input (up to max_manual_speed velocity)
    if (navCanAdjustHorizontalVelocityAndAttitudeFromRCInput()) {
        int16_t rcPitchAdjustment = applyDeadband(rcCommand[PITCH], posControl.navConfig->pos_hold_deadband);
        int16_t rcRollAdjustment = applyDeadband(rcCommand[ROLL], posControl.navConfig->pos_hold_deadband);

        if (rcPitchAdjustment || rcRollAdjustment) {
            float rcShiftX = rcPitchAdjustment * posControl.navConfig->max_manual_speed / (500.0f - posControl.navConfig->pos_hold_deadband) * trackingPeriod;
            float rcShiftY = rcRollAdjustment * posControl.navConfig->max_manual_speed / (500.0f - posControl.navConfig->pos_hold_deadband) * trackingPeriod;

            // Calculate rotation coefficients
            float sinYaw = sin_approx(posControl.actualState.yaw * RADX100);
            float cosYaw = cos_approx(posControl.actualState.yaw * RADX100);

            // Rotate this target shift from body frame to to earth frame and apply to position target
            virtualDesiredPosition.V.X += rcShiftX * cosYaw - rcShiftY * sinYaw;
            virtualDesiredPosition.V.Y += rcShiftX * sinYaw + rcShiftY * cosYaw;

            posControl.flags.isAdjustingPosition = true;
        }
        else {
            posControl.flags.isAdjustingPosition = false;
        }
    }
    else {
        posControl.flags.isAdjustingPosition = false;
    }
}

static void updatePositionHeadingController_FW(uint32_t deltaMicros)
{
    // We have virtual position target, calculate heading error
    int32_t virtualTargetBearing = calculateBearingToDestination(&virtualDesiredPosition);

    // Calculate NAV heading error
    int32_t headingError = wrap_18000(posControl.actualState.yaw - virtualTargetBearing);

    // Forced turn direction
    if (ABS(headingError) > 9000) {
        headingError = 9000;
    }

    // Input error in (deg*100), output pitch angle (deg*100)
    float rollAdjustment = navPidApply2(headingError, US2S(deltaMicros), &posControl.pids.fw_nav, -NAV_ROLL_PITCH_MAX_FW, NAV_ROLL_PITCH_MAX_FW);

    // Convert rollAdjustment to decidegrees (rcAdjustment holds decidegrees)
    posControl.rcAdjustment[ROLL] = constrain(rollAdjustment, -NAV_ROLL_PITCH_MAX_FW, NAV_ROLL_PITCH_MAX_FW) * 0.1f;
}

void applyFixedWingPositionController(uint32_t currentTime)
{
    static uint32_t previousTimePositionUpdate;         // Occurs @ GPS update rate
    static uint32_t previousTimeUpdate;                 // Occurs @ looptime rate

    uint32_t deltaMicros = currentTime - previousTimeUpdate;
    previousTimeUpdate = currentTime;

    // If last position update was too long in the past - ignore it (likely restarting altitude controller)
    if (deltaMicros > HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
        previousTimeUpdate = currentTime;
        previousTimePositionUpdate = currentTime;
        resetFixedWingPositionController();
        return;
    }

    // Apply controller only if position source is valid. In absence of valid pos sensor (GPS loss), we'd stick in forced ANGLE mode
    if (posControl.flags.hasValidPositionSensor) {
        // If we have new position - update velocity and acceleration controllers
        if (posControl.flags.horizontalPositionNewData) {
            uint32_t deltaMicrosPositionUpdate = currentTime - previousTimePositionUpdate;
            previousTimePositionUpdate = currentTime;

            if (deltaMicrosPositionUpdate < HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
                // Calculate virtual position target at a distance of forwardVelocity * HZ2S(POSITION_TARGET_UPDATE_RATE_HZ)
                // Account for pilot's roll input (move position target left/right at max of max_manual_speed)
                // POSITION_TARGET_UPDATE_RATE_HZ should be chosen keeping in mind that position target shouldn't be reached until next pos update occurs
                // FIXME: verify the above
                calculateVirtualPositionTarget_FW(posControl.navConfig->fw_nav_period_ms * 1e-3f);

                updatePositionHeadingController_FW(deltaMicrosPositionUpdate);
            }
            else {
                resetFixedWingPositionController();
            }

            // Indicate that information is no longer usable
            posControl.flags.horizontalPositionNewData = 0;
        }

        rcCommand[ROLL] = leanAngleToRcCommand(posControl.rcAdjustment[ROLL]);
    }
    else {
        // No valid pos sensor data, don't adjust pitch automatically, rcCommand[ROLL] is passed through to PID controller
    }
}

/*-----------------------------------------------------------
 * FixedWing land detector
 *-----------------------------------------------------------*/
bool isFixedWingLandingDetected(uint32_t * landingTimer)
{
    uint32_t currentTime = micros();

    // TODO

    *landingTimer = currentTime;
    return false;
}

/*-----------------------------------------------------------
 * FixedWing emergency landing
 *-----------------------------------------------------------*/
void applyFixedWingEmergencyLandingController(void)
{
    // TODO
}

/*-----------------------------------------------------------
 * Fixed-wing-specific automatic parameter update
 *-----------------------------------------------------------*/
void updateFixedWingSpecificData(uint32_t currentTime)
{
    UNUSED(currentTime);
}

#endif  // NAV
