#include "accelmotor.h"
#include "math.h"

/**
 * @brief init a accelstepper
 */
void accelStepperInit(AccelStepper *stepperx)
{
    int i;
    stepperx->_currentPos       = 0;
    stepperx->_targetPos        = 0;
    stepperx->_speed            = 0.0;
    stepperx->_maxSpeed         = 0.0;
    stepperx->_acceleration     = 0.0;
    stepperx->_sqrt_twoa        = 1.0;
    stepperx->_stepInterval     = 0;
    stepperx->_minPulseWidth    = 1;
    stepperx->_enablePin        = 0xff;
    stepperx->_lastStepTime     = 0;
    stepperx->_enableInverted   = false;
    stepperx->_constanSpeedFlag = false;

    // new vals
    stepperx->_n         = 0;
    stepperx->_c0        = 0.0;
    stepperx->_cn        = 0.0;
    stepperx->_cmin      = 1.0;
    stepperx->_direction = DIRECTION_CCW;
    stepperx->_stopFlag  = 0;

    for (i = 0; i < 4; i++)
        stepperx->_pinInverted[i] = 0;
}

unsigned long computeNewSpeed(AccelStepper *stepperx)
{
    long distanceTo  = stepperx->_targetPos - stepperx->_currentPos;                                     // +ve is clockwise from curent location
    long stepsToStop = (long)((stepperx->_speed * stepperx->_speed) / (2.0f * stepperx->_acceleration)); // Equation 16

    if (distanceTo == 0 && stepsToStop <= 1) {
        // We are at the target and its time to stop
        stepperx->_stepInterval = 0;
        stepperx->_speed        = 0.0;
        stepperx->_n            = 0;
        return stepperx->_stepInterval;
    }

    if (distanceTo > 0) {
        // We are anticlockwise from the target
        // Need to go clockwise from here, maybe decelerate now
        if (stepperx->_n > 0) {
            // Currently accelerating, need to decel now? Or maybe going the wrong way?
            if ((stepsToStop >= distanceTo) || stepperx->_direction == DIRECTION_CCW)
                stepperx->_n = -stepsToStop; // Start deceleration
        } else if (stepperx->_n < 0) {
            // Currently decelerating, need to accel again?
            if ((stepsToStop < distanceTo) && stepperx->_direction == DIRECTION_CW)
                stepperx->_n = -stepperx->_n; // Start accceleration
        }
    } else if (distanceTo < 0) {
        // We are clockwise from the target
        // Need to go anticlockwise from here, maybe decelerate now
        if (stepperx->_n > 0) {
            // Currently accelerating, need to decel now? Or maybe going the wrong way?
            if ((stepsToStop >= -distanceTo) || stepperx->_direction == DIRECTION_CW)
                stepperx->_n = -stepsToStop; // Start deceleration
        } else if (stepperx->_n < 0) {
            // Currently decelerating, need to accel again?
            if ((stepsToStop < -distanceTo) && stepperx->_direction == DIRECTION_CCW)
                stepperx->_n = -stepperx->_n; // Start accceleration
        }
    }

    // Need to accelerate or decelerate
    if (stepperx->_n == 0) {
        // First step from stopped
        stepperx->_cn        = stepperx->_c0;
        stepperx->_direction = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW;
    } else {
        // Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
        stepperx->_cn = stepperx->_cn - ((2.0f * stepperx->_cn) / ((4.0f * stepperx->_n) + 1)); // Equation 13
        stepperx->_cn = (stepperx->_cn > stepperx->_cmin) ? stepperx->_cn : stepperx->_cmin;
    }
    stepperx->_n++;
    stepperx->_stepInterval = stepperx->_cn;
    stepperx->_speed        = 1000000.0f / stepperx->_cn;
    if (stepperx->_direction == DIRECTION_CCW)
        stepperx->_speed = -stepperx->_speed;

#if 0
   Serial.println(_speed);
   Serial.println(_acceleration);
   Serial.println(_cn);
   Serial.println(_c0);
   Serial.println(_n);
   Serial.println(_stepInterval);
   Serial.println(distanceTo);
   Serial.println(stepsToStop);
   Serial.println("-----");
#endif
    return stepperx->_stepInterval;
}

void setAcceleration(AccelStepper *stepperx, float acceleration)
{

    if (acceleration == 0.0f)
        return;
    if (acceleration < 0.0f)
        acceleration = -acceleration;
    if (stepperx->_acceleration != acceleration) {
        // Recompute _n per Equation 17
        stepperx->_n = stepperx->_n * (stepperx->_acceleration / acceleration);

        // New c0 per Equation 7, with correction per Equation 15
        stepperx->_c0           = 0.676f * sqrt(2.0f / acceleration) * 1000000.0f; // Equation 15
        stepperx->_acceleration = acceleration;
        computeNewSpeed(stepperx);
    }
}

void setMaxSpeed(AccelStepper *stepperx, float speed)
{
    if (speed < 0.0f)
        speed = -speed;
    if (stepperx->_maxSpeed != speed) {
        stepperx->_maxSpeed = speed;
        stepperx->_cmin     = 1000000.0f / speed;

        // Recompute _n from current speed and adjust speed if accelerating or cruising
        if (stepperx->_n > 0) {
            stepperx->_n = (long)((stepperx->_speed * stepperx->_speed) / (2.0f * stepperx->_acceleration)); // Equation 16
            computeNewSpeed(stepperx);
        }
    }
}

void moveTo(AccelStepper *stepperx, long absolute)
{
    stepperx->_constanSpeedFlag = false;
    if (stepperx->_targetPos != absolute) {
        stepperx->_targetPos = absolute;
    }
}

void stopMotor(AccelStepper *stepperx)
{
    stepperx->_stopFlag = 1;
}

void MoveConstantSpeed(AccelStepper *stepperx, long constantInterval, long absolute)
{
    stepperx->_constanSpeedFlag = true;
    if (stepperx->_targetPos != absolute) {

        stepperx->_targetPos    = absolute;
        stepperx->_stepInterval = constantInterval;
        if (stepperx->_targetPos > stepperx->_currentPos) {
            stepperx->_direction = true;
        } else {
            stepperx->_direction = false;
        }
    }
}
