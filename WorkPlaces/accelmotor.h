#ifndef __accelmotor_H_
#define __accelmotor_H_

#include "stdbool.h"
#include "main.h"

// 单位为mm
#define K_4_25    0.04f
#define K_17_1200 0.00177f
#define K_1_88    0.0011f

typedef struct {
    // Number of pins on the stepper motor. Permits 2 or 4. 2 pins is a
    // bipolar, and 4 pins is a unipolar.
    uint8_t _interface; // 0, 1, 2, 4, 8, See MotorInterfaceType

    // Arduino pin number assignments for the 2 or 4 pins required to interface to the
    // stepper motor or driver
    uint8_t _pin[4];

    // Whether the _pins is inverted or not
    uint8_t _pinInverted[4];

    // The current absolution position in steps.
    long _currentPos; // Steps

    // The target position in steps. The AccelStepper library will move the
    // motor from the _currentPos to the _targetPos, taking into account the
    // max speed, acceleration and deceleration
    long _targetPos; // Steps

    // The current motos speed in steps per second
    // Positive is clockwise
    float _speed; // Steps per second

    // The maximum permitted speed in steps per second. Must be > 0.
    float _maxSpeed;

    // The acceleration to use to accelerate or decelerate the motor in steps
    // per second per second. Must be > 0
    float _acceleration;
    float _sqrt_twoa; // Precomputed sqrt(2*_acceleration)

    // The last step time in microseconds
    unsigned long _lastStepTime;

    // The minimum allowed pulse width in microseconds
    unsigned int _minPulseWidth;

    // Is the direction pin inverted?
    // bool           _dirInverted;     // Moved to _pinInverted[1]

    // Is the step pin inverted?
    // bool           _stepInverted;    // Moved to _pinInverted[0]

    // Is the enable pin inverted?
    bool _enableInverted;

    // Enable pin for stepper driver, or 0xFF if unused.
    uint8_t _enablePin;

    // The step counter for speed calculations
    long _n;

    // Initial step size in microseconds
    float _c0;

    // Last step size in microseconds
    float _cn;

    // Min step size in microseconds based on maxSpeed
    float _cmin; // at max speed

    // Current direction motor is spinning in
    // Protected because some peoples subclasses need it to be so
    bool _direction; // 1 == CW

    // The current interval between steps in microseconds.
    // 0 means the motor is currently stopped with _speed == 0
    unsigned long _stepInterval;

    bool _stopFlag;

    bool _constanSpeedFlag;

} AccelStepper;

typedef enum {
    DIRECTION_CCW = 0, //< Counter-Clockwise
    DIRECTION_CW  = 1  //< Clockwise
} Direction;

void accelStepperInit(AccelStepper *stepperx);
unsigned long computeNewSpeed(AccelStepper *stepperx);
void setAcceleration(AccelStepper *stepperx, float acceleration);
void setMaxSpeed(AccelStepper *stepperx, float speed);
void moveTo(AccelStepper *stepperx, long absolute);
void stopMotor(AccelStepper *stepperx);
void MoveConstantSpeed(AccelStepper *stepperx, long constantInterval, long absolute);

#endif
