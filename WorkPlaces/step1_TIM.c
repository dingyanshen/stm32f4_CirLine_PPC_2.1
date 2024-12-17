#include "step1_TIM.h"
#include "accelmotor.h"
#include "delay.h"
#include "usart.h"
#include "tim.h"

#define A_Forward1step()         \
    {                            \
        step1_dir_High;          \
        stepper_A._currentPos++; \
        delay_us(1);             \
        step1_step_High;         \
        delay_us(1);             \
        step1_step_Low;          \
    }

#define A_Reverse1step()         \
    {                            \
        step1_dir_Low;           \
        stepper_A._currentPos--; \
        delay_us(1);             \
        step1_step_High;         \
        delay_us(1);             \
        step1_step_Low;          \
    }

#define B_Forward1step()         \
    {                            \
        step2_dir_High;          \
        stepper_B._currentPos++; \
        delay_us(1);             \
        step2_step_High;         \
        delay_us(1);             \
        step2_step_Low;          \
    }

#define B_Reverse1step()         \
    {                            \
        step2_dir_Low;           \
        stepper_B._currentPos--; \
        delay_us(1);             \
        step2_step_High;         \
        delay_us(1);             \
        step2_step_Low;          \
    }

#define Z_Forward1step()         \
    {                            \
        step3_dir_High;          \
        stepper_Z._currentPos++; \
        delay_us(1);             \
        step3_step_High;         \
        delay_us(1);             \
        step3_step_Low;          \
    }

#define Z_Reverse1step()         \
    {                            \
        step3_dir_Low;           \
        stepper_Z._currentPos--; \
        delay_us(1);             \
        step3_step_High;         \
        delay_us(1);             \
        step3_step_Low;          \
    }

#define S_Forward1step()         \
    {                            \
        step4_dir_High;          \
        stepper_S._currentPos++; \
        delay_us(1);             \
        step4_step_High;         \
        delay_us(1);             \
        step4_step_Low;          \
    }

#define S_Reverse1step()         \
    {                            \
        step4_dir_Low;           \
        stepper_S._currentPos--; \
        delay_us(1);             \
        step4_step_High;         \
        delay_us(1);             \
        step4_step_Low;          \
    }

#define Y_Forward1step()  \
    {                     \
        B_Forward1step(); \
        A_Reverse1step(); \
    }

#define Y_Reverse1step()  \
    {                     \
        A_Forward1step(); \
        B_Reverse1step(); \
    }

#define X_Forward1step()  \
    {                     \
        A_Forward1step(); \
        B_Forward1step(); \
    }

#define X_Reverse1step()  \
    {                     \
        A_Reverse1step(); \
        B_Reverse1step(); \
    }

/**
 * 运动模式
 * 0 : x y z R stop
 * 1 : xy line
 * 2 : z line
 * 3 : S line
 * 4 : XYZ line
 * 5 : XY Circle
 */
uint8_t RunMode = 0;

/**
 * 逐点直线插补法相关变量
 */
extern float f_coreae, f_corebe;                   // aob坐标系下的目标点
extern long l_stepperAdis, l_stepperBdis;          // aob坐标系下 电机脉冲数的目标点
extern long l_stepperAe, l_stepperBe, l_stepperZe; // aob坐标系下 原点平移后的目标点
extern long l_Eij;                                 // 逐点插补下的剩余步数
extern long l_Fij, l_Fijk[3];                      // 方向公式  Fij = Xe * yj - xi * Ye
extern long l_e[3];                                // 三个方向均匀变化的均匀函数
extern uint8_t u8_quadrantSeial;                   // 三维空间内运动的卦限

/**
 * 逐点xy圆周插补相关变量
 */
extern long l_Fij_XYCircle;               // XY在想象中的虚拟坐标中的误差函数
extern long l_Xi_XYCircle, l_Yi_XYCircle; // XY的虚拟坐标
extern uint8_t Seial_XYCircle;            // XY的所处象限

/**
 * @brief 进行步进电机的进给
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    switch (RunMode) {
        case 0:
            step1_TIM->ARR = 999;
            break;
        case 1:
            XYLineIhandler();
            break;
        case 2:
            ZLineIhandler();
            break;
        case 3:
            SLineIhandler();
            break;
        case 4:
            XYZLineIhandler();
            break;
        case 5:
            XYCircleIhandler();
            break;
        default:
            break;
    }
}

/**
 * @brief move xy to xy
 */
void XYLineIhandler(void)
{
    if (stepper_XY._stopFlag) {
        stepper_XY._targetPos    = 0;
        stepper_XY._currentPos   = 0;
        stepper_XY._stepInterval = 0;
        stepper_XY._speed        = 0;
        stepper_XY._n            = 0;
        stepper_XY._stopFlag     = 0;

        stepper_A._targetPos    = 0;
        stepper_A._currentPos   = 0;
        stepper_A._stepInterval = 0;
        stepper_A._speed        = 0;
        stepper_A._n            = 0;
        stepper_A._stopFlag     = 0;

        stepper_B._targetPos    = 0;
        stepper_B._currentPos   = 0;
        stepper_B._stepInterval = 0;
        stepper_B._speed        = 0;
        stepper_B._n            = 0;
        stepper_B._stopFlag     = 0;

        RunMode = 0;
    }
    if (stepper_XY._targetPos > stepper_XY._currentPos) {
        computeNewSpeed(&stepper_XY);
        step1_TIM->ARR = stepper_XY._stepInterval + 10;
        switch (u8_quadrantSeial) {
            case 1: // 第一象限
                if (l_Fij >= 0) {
                    A_Forward1step();
                    l_Fij = l_Fij - l_stepperBe;

                } else {
                    B_Forward1step();
                    l_Fij = l_Fij + l_stepperAe;
                }
                break;
            case 2: // 第二象限

                if (l_Fij >= 0) {
                    B_Forward1step();
                    l_Fij = l_Fij + l_stepperAe;
                } else {
                    A_Reverse1step();
                    l_Fij = l_Fij + l_stepperBe;
                }
                break;
            case 3:
                if (l_Fij >= 0) {
                    A_Reverse1step();
                    l_Fij = l_Fij + l_stepperBe;
                } else {
                    B_Reverse1step();
                    l_Fij = l_Fij - l_stepperAe;
                }
                break;
            case 4:
                if (l_Fij >= 0) {
                    B_Reverse1step();
                    l_Fij = l_Fij - l_stepperAe;
                } else {
                    A_Forward1step();
                    l_Fij = l_Fij - l_stepperBe;
                }
                break;
            default:
                break;
        }
        stepper_XY._currentPos++;
    } else {
        RunMode = 0;
    }
}

/**
 * @brief move xyz to xyz
 */
void XYZLineIhandler(void)
{
    if (stepper_XY._stopFlag) {
        stepper_XYZ._targetPos    = 0;
        stepper_XYZ._currentPos   = 0;
        stepper_XYZ._stepInterval = 0;
        stepper_XYZ._speed        = 0;
        stepper_XYZ._n            = 0;
        stepper_XYZ._stopFlag     = 0;

        stepper_A._targetPos    = 0;
        stepper_A._currentPos   = 0;
        stepper_A._stepInterval = 0;
        stepper_A._speed        = 0;
        stepper_A._n            = 0;
        stepper_A._stopFlag     = 0;

        stepper_B._targetPos    = 0;
        stepper_B._currentPos   = 0;
        stepper_B._stepInterval = 0;
        stepper_B._speed        = 0;
        stepper_B._n            = 0;
        stepper_B._stopFlag     = 0;

        RunMode = 0;
    }
    if (stepper_Z._stopFlag) {
        stepper_XYZ._targetPos    = 0;
        stepper_XYZ._currentPos   = 0;
        stepper_XYZ._stepInterval = 0;
        stepper_XYZ._speed        = 0;
        stepper_XYZ._n            = 0;
        stepper_XYZ._stopFlag     = 0;

        stepper_Z._targetPos    = 0;
        stepper_Z._currentPos   = 0;
        stepper_Z._stepInterval = 0;
        stepper_Z._speed        = 0;
        stepper_Z._n            = 0;
        stepper_Z._stopFlag     = 0;

        RunMode = 0;
    }
    if (stepper_XYZ._targetPos > stepper_XYZ._currentPos) {

        computeNewSpeed(&stepper_XYZ);
        step1_TIM->ARR = stepper_XYZ._stepInterval + 10;
        switch (u8_quadrantSeial) {
            case 1: // 第一象限
                if (l_Fijk[0] <= l_Fijk[1] && l_Fijk[0] <= l_Fijk[2]) {
                    A_Forward1step();
                    l_Fijk[0] = l_Fijk[0] + l_e[1] + l_e[2];
                    l_Fijk[1] = l_Fijk[1] - l_e[1];
                    l_Fijk[2] = l_Fijk[2] - l_e[2];

                } else if (l_Fijk[1] <= l_Fijk[0] && l_Fijk[1] <= l_Fijk[2]) {
                    B_Forward1step();
                    l_Fijk[0] = l_Fijk[0] - l_e[0];
                    l_Fijk[1] = l_Fijk[1] + l_e[0] + l_e[2];
                    l_Fijk[2] = l_Fijk[2] - l_e[2];

                } else {
                    Z_Forward1step();
                    l_Fijk[0] = l_Fijk[0] - l_e[0];
                    l_Fijk[1] = l_Fijk[1] - l_e[1];
                    l_Fijk[2] = l_Fijk[2] + l_e[0] + l_e[1];
                }
                break;
            case 2: // 第二象限
                if (l_Fijk[0] <= l_Fijk[1] && l_Fijk[0] <= l_Fijk[2]) {
                    A_Reverse1step();
                    l_Fijk[0] = l_Fijk[0] + l_e[1] + l_e[2];
                    l_Fijk[1] = l_Fijk[1] - l_e[1];
                    l_Fijk[2] = l_Fijk[2] - l_e[2];

                } else if (l_Fijk[1] <= l_Fijk[0] && l_Fijk[1] <= l_Fijk[2]) {
                    B_Forward1step();
                    l_Fijk[0] = l_Fijk[0] - l_e[0];
                    l_Fijk[1] = l_Fijk[1] + l_e[0] + l_e[2];
                    l_Fijk[2] = l_Fijk[2] - l_e[2];

                } else {
                    Z_Forward1step();
                    l_Fijk[0] = l_Fijk[0] - l_e[0];
                    l_Fijk[1] = l_Fijk[1] - l_e[1];
                    l_Fijk[2] = l_Fijk[2] + l_e[0] + l_e[1];
                }
                break;
            case 3:
                if (l_Fijk[0] <= l_Fijk[1] && l_Fijk[0] <= l_Fijk[2]) {
                    A_Reverse1step();
                    l_Fijk[0] = l_Fijk[0] + l_e[1] + l_e[2];
                    l_Fijk[1] = l_Fijk[1] - l_e[1];
                    l_Fijk[2] = l_Fijk[2] - l_e[2];

                } else if (l_Fijk[1] <= l_Fijk[0] && l_Fijk[1] <= l_Fijk[2]) {
                    B_Reverse1step();
                    l_Fijk[0] = l_Fijk[0] - l_e[0];
                    l_Fijk[1] = l_Fijk[1] + l_e[0] + l_e[2];
                    l_Fijk[2] = l_Fijk[2] - l_e[2];

                } else {
                    Z_Forward1step();
                    l_Fijk[0] = l_Fijk[0] - l_e[0];
                    l_Fijk[1] = l_Fijk[1] - l_e[1];
                    l_Fijk[2] = l_Fijk[2] + l_e[0] + l_e[1];
                }
                break;
            case 4:
                if (l_Fijk[0] <= l_Fijk[1] && l_Fijk[0] <= l_Fijk[2]) {
                    A_Forward1step();
                    l_Fijk[0] = l_Fijk[0] + l_e[1] + l_e[2];
                    l_Fijk[1] = l_Fijk[1] - l_e[1];
                    l_Fijk[2] = l_Fijk[2] - l_e[2];

                } else if (l_Fijk[1] <= l_Fijk[0] && l_Fijk[1] <= l_Fijk[2]) {
                    B_Reverse1step();
                    l_Fijk[0] = l_Fijk[0] - l_e[0];
                    l_Fijk[1] = l_Fijk[1] + l_e[0] + l_e[2];
                    l_Fijk[2] = l_Fijk[2] - l_e[2];

                } else {
                    Z_Forward1step();
                    l_Fijk[0] = l_Fijk[0] - l_e[0];
                    l_Fijk[1] = l_Fijk[1] - l_e[1];
                    l_Fijk[2] = l_Fijk[2] + l_e[0] + l_e[1];
                }
                break;
            case 5:
                if (l_Fijk[0] <= l_Fijk[1] && l_Fijk[0] <= l_Fijk[2]) {
                    A_Forward1step();
                    l_Fijk[0] = l_Fijk[0] + l_e[1] + l_e[2];
                    l_Fijk[1] = l_Fijk[1] - l_e[1];
                    l_Fijk[2] = l_Fijk[2] - l_e[2];

                } else if (l_Fijk[1] <= l_Fijk[0] && l_Fijk[1] <= l_Fijk[2]) {
                    B_Forward1step();
                    l_Fijk[0] = l_Fijk[0] - l_e[0];
                    l_Fijk[1] = l_Fijk[1] + l_e[0] + l_e[2];
                    l_Fijk[2] = l_Fijk[2] - l_e[2];

                } else {
                    Z_Reverse1step();
                    l_Fijk[0] = l_Fijk[0] - l_e[0];
                    l_Fijk[1] = l_Fijk[1] - l_e[1];
                    l_Fijk[2] = l_Fijk[2] + l_e[0] + l_e[1];
                }
                break;
            case 6:
                if (l_Fijk[0] <= l_Fijk[1] && l_Fijk[0] <= l_Fijk[2]) {
                    A_Reverse1step();
                    l_Fijk[0] = l_Fijk[0] + l_e[1] + l_e[2];
                    l_Fijk[1] = l_Fijk[1] - l_e[1];
                    l_Fijk[2] = l_Fijk[2] - l_e[2];

                } else if (l_Fijk[1] <= l_Fijk[0] && l_Fijk[1] <= l_Fijk[2]) {
                    B_Forward1step();
                    l_Fijk[0] = l_Fijk[0] - l_e[0];
                    l_Fijk[1] = l_Fijk[1] + l_e[0] + l_e[2];
                    l_Fijk[2] = l_Fijk[2] - l_e[2];

                } else {
                    Z_Reverse1step();
                    l_Fijk[0] = l_Fijk[0] - l_e[0];
                    l_Fijk[1] = l_Fijk[1] - l_e[1];
                    l_Fijk[2] = l_Fijk[2] + l_e[0] + l_e[1];
                }
                break;
            case 7:
                if (l_Fijk[0] <= l_Fijk[1] && l_Fijk[0] <= l_Fijk[2]) {
                    A_Reverse1step();
                    l_Fijk[0] = l_Fijk[0] + l_e[1] + l_e[2];
                    l_Fijk[1] = l_Fijk[1] - l_e[1];
                    l_Fijk[2] = l_Fijk[2] - l_e[2];

                } else if (l_Fijk[1] <= l_Fijk[0] && l_Fijk[1] <= l_Fijk[2]) {
                    B_Reverse1step();
                    l_Fijk[0] = l_Fijk[0] - l_e[0];
                    l_Fijk[1] = l_Fijk[1] + l_e[0] + l_e[2];
                    l_Fijk[2] = l_Fijk[2] - l_e[2];

                } else {
                    Z_Reverse1step();
                    l_Fijk[0] = l_Fijk[0] - l_e[0];
                    l_Fijk[1] = l_Fijk[1] - l_e[1];
                    l_Fijk[2] = l_Fijk[2] + l_e[0] + l_e[1];
                }
                break;
            case 8:
                if (l_Fijk[0] <= l_Fijk[1] && l_Fijk[0] <= l_Fijk[2]) {
                    A_Forward1step();
                    l_Fijk[0] = l_Fijk[0] + l_e[1] + l_e[2];
                    l_Fijk[1] = l_Fijk[1] - l_e[1];
                    l_Fijk[2] = l_Fijk[2] - l_e[2];

                } else if (l_Fijk[1] <= l_Fijk[0] && l_Fijk[1] <= l_Fijk[2]) {
                    B_Reverse1step();
                    l_Fijk[0] = l_Fijk[0] - l_e[0];
                    l_Fijk[1] = l_Fijk[1] + l_e[0] + l_e[2];
                    l_Fijk[2] = l_Fijk[2] - l_e[2];

                } else {
                    Z_Reverse1step();
                    l_Fijk[0] = l_Fijk[0] - l_e[0];
                    l_Fijk[1] = l_Fijk[1] - l_e[1];
                    l_Fijk[2] = l_Fijk[2] + l_e[0] + l_e[1];
                }
                break;
            default:
                break;
        }
        stepper_XYZ._currentPos++;
    } else {
        RunMode = 0;
    }
}

/**
 * @brief move z to z
 */
void ZLineIhandler(void)
{
    if (stepper_Z._stopFlag) {
        stepper_Z._targetPos    = 0;
        stepper_Z._currentPos   = 0;
        stepper_Z._stepInterval = 0;
        stepper_Z._speed        = 0;
        stepper_Z._n            = 0;
        stepper_Z._stopFlag     = 0;
        RunMode                 = 0;
    }
    if (stepper_Z._targetPos != stepper_Z._currentPos) {
        computeNewSpeed(&stepper_Z);
        step1_TIM->ARR = stepper_Z._stepInterval + 10;
        if (stepper_Z._direction == 1) {
            Z_Forward1step();
        } else {
            Z_Reverse1step();
        }

    } else {
        RunMode = 0;
    }
}

/**
 * @brief move S to S
 */
void SLineIhandler(void)
{
    if (stepper_S._stopFlag) {
        stepper_S._targetPos    = 0;
        stepper_S._currentPos   = 0;
        stepper_S._stepInterval = 0;
        stepper_S._speed        = 0;
        stepper_S._n            = 0;
        stepper_S._stopFlag     = 0;
        RunMode                 = 0;
    }
    if (stepper_S._targetPos != stepper_S._currentPos) {
        computeNewSpeed(&stepper_S);
        step1_TIM->ARR = stepper_S._stepInterval + 10;
        if (stepper_S._direction == 1) {
            S_Forward1step();
        } else {
            S_Reverse1step();
        }
    } else {
        RunMode = 0;
    }
}

void Judge_Quadrant(void)
{
    if (l_Xi_XYCircle >= 0 && l_Yi_XYCircle < 0)
        Seial_XYCircle = 4;
    else if (l_Xi_XYCircle > 0 && l_Yi_XYCircle >= 0)
        Seial_XYCircle = 1;
    else if (l_Xi_XYCircle <= 0 && l_Yi_XYCircle > 0)
        Seial_XYCircle = 2;
    else
        Seial_XYCircle = 3;
}

void XYCircleIhandler(void)
{
    if (stepper_XY._stopFlag) {
        stepper_XY._targetPos    = 0;
        stepper_XY._currentPos   = 0;
        stepper_XY._stepInterval = 0;
        stepper_XY._speed        = 0;
        stepper_XY._n            = 0;
        stepper_XY._stopFlag     = 0;

        stepper_A._targetPos    = 0;
        stepper_A._currentPos   = 0;
        stepper_A._stepInterval = 0;
        stepper_A._speed        = 0;
        stepper_A._n            = 0;
        stepper_A._stopFlag     = 0;

        stepper_B._targetPos    = 0;
        stepper_B._currentPos   = 0;
        stepper_B._stepInterval = 0;
        stepper_B._speed        = 0;
        stepper_B._n            = 0;
        stepper_B._stopFlag     = 0;

        RunMode = 0;
    }
    if (stepper_XYCircle._currentPos != stepper_XYCircle._targetPos) {
        Judge_Quadrant();

        computeNewSpeed(&stepper_XYCircle);
        step1_TIM->ARR = stepper_XYCircle._stepInterval + 10;
        switch (Seial_XYCircle) {
            case 1:
                if (l_Fij_XYCircle <= 0) {
                    A_Forward1step();
                    l_Xi_XYCircle--;
                    l_Fij_XYCircle = l_Fij_XYCircle + 2 * l_Xi_XYCircle + 1;
                } else {
                    B_Forward1step();
                    l_Yi_XYCircle++;
                    l_Fij_XYCircle = l_Fij_XYCircle - 2 * l_Yi_XYCircle + 1;
                }
                break;

            case 2:
                if (l_Fij_XYCircle <= 0) {
                    A_Forward1step();
                    l_Xi_XYCircle--;
                    l_Fij_XYCircle = l_Fij_XYCircle - 2 * l_Xi_XYCircle + 1;
                } else {
                    B_Reverse1step();
                    l_Yi_XYCircle--;
                    l_Fij_XYCircle = l_Fij_XYCircle - 2 * l_Yi_XYCircle + 1;
                }
                break;

            case 3:
                if (l_Fij_XYCircle <= 0) {
                    A_Reverse1step();
                    l_Xi_XYCircle++;
                    l_Fij_XYCircle = l_Fij_XYCircle - 2 * l_Xi_XYCircle + 1;
                } else {
                    B_Reverse1step();
                    l_Yi_XYCircle--;
                    l_Fij_XYCircle = l_Fij_XYCircle + 2 * l_Yi_XYCircle + 1;
                }
                break;

            case 4:
                if (l_Fij_XYCircle <= 0) {
                    A_Reverse1step();
                    l_Xi_XYCircle++;
                    l_Fij_XYCircle = l_Fij_XYCircle + 2 * l_Xi_XYCircle + 1;
                } else {
                    B_Forward1step();
                    l_Yi_XYCircle++;
                    l_Fij_XYCircle = l_Fij_XYCircle + 2 * l_Yi_XYCircle + 1;
                }
                break;

            default:
                break;
        }
        stepper_XYCircle._currentPos++;
    } else {
        RunMode = 0;
    }
}
