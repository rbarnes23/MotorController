/**
 * @ Author: Richard Barnes
 * @ Create Time: 2022-02-12 09:59:40
 * @ Modified by: Richard Barnes
 * @ Modified time: 2022-02-13 01:46:05
 * @ Description:
 */


#ifdef __MOTORCONTROLLER_H__
#define __MOTORCONTROLLER_H__
#endif

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

//#define DEBUG true
// vra and vrb are used to adjust individual motor speeds on the fly
struct motorDrive
{
    int8_t ldir;
    uint16_t lspeed;
    uint16_t vra;
    int8_t rdir;
    uint16_t rspeed;
    uint16_t vrb;
};

struct PWM
{
    float left_voltage;
    float right_voltage;
};

//enum BoardType{NODEMCU32S,TEENSY40};


class Motor
{
public:
    motorDrive calculateMotorSpeeds(uint16_t pwm_X, uint16_t pwm_Y, uint8_t resolution = 8, uint8_t min_left = 45, uint8_t min_right = 45);
    motorDrive calculateTurn(int16_t nJoyX, int16_t nJoyY, uint16_t dead_zone);
    float fmap(float sensorValue, float sensorMin, float sensorMax, float outMin, float outMax);
protected:
private:
};

class Motor_BTS7960
{
public:
    // Constructor
    // Variables

    // Methods
    void init(uint8_t motor_no, uint8_t motor_EN_L_pin, uint8_t motor_EN_R_pin, uint8_t motor_PWM_L_pin, uint8_t motor_PWM_R_pin, uint16_t frequency, uint16_t resolution, int flip_dir);
    void drive(int8_t dir, uint16_t motorSpeed);
    void motorEnable(int pin, int hilo);
    void stop();

protected:
private:
    // Variables
    int8_t _last_direction;
    int8_t _flip_dir = 1;
    uint8_t _motor_EN_L_pin;
    uint8_t _motor_EN_R_pin;
    uint8_t _motor_PWM_L_pin;
    uint8_t _motor_PWM_R_pin;
    int16_t _frequency;
    int16_t _resolution;
    uint8_t _motor_no;
    uint8_t _board_no;

    // Setting PWM properties
    uint8_t _pwmChannel_LEFT;
    uint8_t _pwmChannel_RIGHT;
};

class Motor_BRUSHLESS_400W
{
public:
    // Constructor

    // Variables

    // Methods
    void init(uint8_t motor_no, uint8_t motor_EN_L_pin, uint8_t motor_EN_R_pin, uint8_t motor_PWM_L_pin, uint8_t motor_PWM_R_pin, uint16_t frequency, uint16_t resolution, int flip_dir);
    void drive(int8_t dir, uint16_t motorSpeed);
    void motorEnable(int pin, int hilo);
    void stop();
    void readEncoder();

protected:
private:
    // Variables
    int8_t _flip_dir = 1;
    uint8_t _motor_EN_L_pin;
    uint8_t _motor_EN_R_pin;
    uint8_t _motor_PWM_L_pin;
    uint8_t _motor_PWM_R_pin;
    int16_t _frequency;
    int16_t _resolution;
    uint8_t _motor_no;
    uint8_t _board_no;

    // Setting PWM properties
    uint8_t _pwmChannel_LEFT;
    uint8_t _pwmChannel_RIGHT;

    // RIO RAND BRUSHLESS 6 to 60 volt
    int8_t _last_direction;
    uint8_t _motor_BRAKE_pin;
    uint8_t _motor_DIRECTION_pin;
    uint8_t _motor_PWM_pin;
    uint8_t _encoder_pin;
    volatile int posi;
    // Methods
};

class Motor_ODRIVE
{
public:
    // Constructor
    Motor_ODRIVE(uint8_t motor_no, uint8_t motor_EN_L_pin, uint8_t motor_EN_R_pin, uint8_t motor_PWM_L_pin, uint8_t motor_PWM_R_pin, uint16_t frequency, uint16_t resolution, int flip_dir);
    // Variables

    // Methods
    void drive(int8_t dir, uint16_t motorSpeed);
    void motorEnable(int pin, int hilo);
    void stop();
    void readEncoder();

protected:
private:
    // Variables
    int8_t _flip_dir = 1;
    uint8_t _motor_EN_L_pin;
    uint8_t _motor_EN_R_pin;
    uint8_t _motor_PWM_L_pin;
    uint8_t _motor_PWM_R_pin;
    int16_t _frequency;
    int16_t _resolution;
    uint8_t _motor_no;

    // Setting PWM properties
    uint8_t _pwmChannel_LEFT;
    uint8_t _pwmChannel_RIGHT;

    // RIO RAND BRUSHLESS 6 to 60 volt
    int8_t _last_direction;
    uint8_t _motor_BRAKE_pin;
    uint8_t _motor_DIRECTION_pin;
    uint8_t _motor_PWM_pin;
    uint8_t _encoder_pin;
    volatile int posi;
    // Methods
};
