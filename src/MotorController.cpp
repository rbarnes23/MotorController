/**
 * @ Author: Richard Barnes
 * @ Create Time: 2022-02-12 09:57:42
 * @ Modified by: Richard Barnes
 * @ Modified time: 2022-02-16 10:16:36
 * @ Description:
 */

#include "MotorController.h"

// Begin Motor Class

void Motor_BTS7960::init(uint8_t motor_no, uint8_t motor_EN_L_pin, uint8_t motor_EN_R_pin, uint8_t motor_PWM_L_pin, uint8_t motor_PWM_R_pin, uint16_t frequency, uint16_t resolution, int flip_dir)
{
    // Anything you need when instantiating your object goes here
    _motor_no = motor_no;
    _flip_dir = flip_dir;
    _frequency = frequency;
    _resolution = resolution;
    _motor_EN_L_pin = motor_EN_L_pin;   // enable direction pin Forward
    _motor_EN_R_pin = motor_EN_R_pin;   // enable direction pin Reverse
    _motor_PWM_L_pin = motor_PWM_L_pin; // set PWM Forward
    _motor_PWM_R_pin = motor_PWM_R_pin; // set PWM Reverse
    // Set mode of pin i.e Signal Output or input
    pinMode(_motor_EN_L_pin, OUTPUT);
    pinMode(_motor_EN_R_pin, OUTPUT);
    pinMode(_motor_PWM_L_pin, OUTPUT);
    pinMode(_motor_PWM_R_pin, OUTPUT);
    // There are 16 channels avail 0 to 15.  BTS7960 requires 2 channels for each controller
    _pwmChannel_LEFT = _motor_no * 2;
    _pwmChannel_RIGHT = (_motor_no * 2) + 1;
#ifdef ESP32M
    ledcSetup(_pwmChannel_LEFT, _frequency, _resolution);
    ledcSetup(_pwmChannel_RIGHT, _frequency, _resolution);
    ledcAttachPin(_motor_PWM_L_pin, _pwmChannel_LEFT);
    ledcAttachPin(_motor_PWM_R_pin, _pwmChannel_RIGHT);
#else
    analogWriteResolution(_resolution); // analogWrite value 0 to 4095, or 4096 for high
    analogWriteFrequency(_motor_PWM_L_pin, _frequency);
    analogWriteFrequency(_motor_PWM_R_pin, _frequency);

#endif
#ifdef DEBUG
    Serial.println("motor\t" + String(_motor_no) + "\t" + String(_motor_EN_L_pin) + "\t" + String(_motor_EN_R_pin) + "\t" + String(_motor_PWM_L_pin) + "\t" + String(_motor_PWM_R_pin));
#endif
}

void Motor_BTS7960::drive(int8_t dir, uint16_t motorSpeed)
{
    dir *= _flip_dir;
    if (dir != _last_direction) // Brake before changing direction
    {
        _last_direction = dir;
    }
    // Enable the motors if moving else disable them
    // Serial.println("DIR: " + String(dir));
    // Enable pins if not stopped
    if (dir != 0)
    {
        digitalWrite(_motor_EN_L_pin, HIGH);
        digitalWrite(_motor_EN_R_pin, HIGH);
    }
    else
    {
        digitalWrite(_motor_EN_L_pin, LOW);
        digitalWrite(_motor_EN_R_pin, LOW);
    }

    // Set pwm for Brushless RioRand 400W
    switch (dir)
    {
    case 1:
        // FORWARD
        // Set the speed of each motor
#ifdef ESP32M
        ledcWrite(_pwmChannel_LEFT, motorSpeed);
        ledcWrite(_pwmChannel_RIGHT, 0);
#else
        analogWrite(_motor_PWM_L_pin, motorSpeed);
        analogWrite(_motor_PWM_R_pin, 0);
#endif

#ifdef DEBUG
        Serial.println("MF " + String(_motor_no) + ": " + String(motorSpeed));
#endif
        break;
    case -1:
        // BACKWARDS
        // Set the speed of each motor
#ifdef ESP32M
        ledcWrite(_pwmChannel_LEFT, 0);
        ledcWrite(_pwmChannel_RIGHT, motorSpeed);
#else
        analogWrite(_motor_PWM_R_pin, motorSpeed);
        analogWrite(_motor_PWM_L_pin, 0);
#endif

#ifdef DEBUG
        Serial.println("MB " + String(_motor_no) + ": " + String(motorSpeed));
#endif
        break;
    default: // STOP
        this->stop();
        break;
    }
}

void Motor_BTS7960::stop()
{

#ifdef ESP32M
    ledcWrite(_pwmChannel_LEFT, 0);
    ledcWrite(_pwmChannel_RIGHT, 0);
#else
    analogWrite(_motor_PWM_L_pin, 0);
    analogWrite(_motor_PWM_R_pin, 0);
#endif

#ifdef DEBUG
    Serial.println("STOP " + String(_motor_no));
#endif
}

// Functions main purpose has to do with Ardupilots Brushed with Relay mode
//  Instead of changing the pwm value for each motor, enable or disable the motor direction on BTS7960
void Motor_BTS7960::motorEnable(int pin, int hilo)
{
    digitalWrite(pin, hilo);
}

// Begin Brushless 400W Rio Rand Motor Class
void Motor_BRUSHLESS_400W::init(uint8_t motor_no, uint8_t motor_BRAKE_pin, uint8_t motor_DIRECTION_pin, uint8_t motor_PWM_pin, uint8_t motor_ENCODER_pin, uint16_t frequency, uint16_t resolution, int flip_dir)
{
    // Anything you need when instantiating your object goes here
    _motor_no = motor_no;
    _flip_dir = flip_dir;
    _frequency = frequency;
    _resolution = resolution;

    _motor_BRAKE_pin = motor_BRAKE_pin;         // BRAKE
    _motor_DIRECTION_pin = motor_DIRECTION_pin; // DIRECTION
    _motor_PWM_pin = motor_PWM_pin;             // Motor PWM
    _motor_ENCODER_pin = motor_ENCODER_pin;     // ENCODER
    pinMode(_motor_BRAKE_pin, OUTPUT);
    pinMode(_motor_DIRECTION_pin, OUTPUT);
    pinMode(_motor_PWM_pin, OUTPUT);
    // pinMode(_encoder_pin, INPUT_PULLDOWN);
    // attachInterrupt(digitalPinToInterrupt(_encoder_pin), readEncoder(), RISING);
    // readEncoder();

#ifdef ESP32M
    ledcSetup(_motor_no, _frequency, _resolution); // There are 16 channels avail 0 to 15
    ledcAttachPin(_motor_PWM_pin, _motor_no);
#else
    analogWriteResolution(_resolution); // analogWrite value 0 to 4095, or 4096 for high
    analogWriteFrequency(_motor_PWM_pin, _frequency);
#endif

#ifdef DEBUG
    Serial.println("motor\t" + String(_motor_no) + "\t" + String(_motor_BRAKE_pin) + "\t" + String(_motor_DIRECTION_pin) + "\t" + String(_motor_PWM_pin) + "\t" + String(_encoder_pin));
#endif
}

void Motor_BRUSHLESS_400W::drive(int8_t dir, uint16_t motorSpeed)
{
    dir *= _flip_dir;
    if (dir != _last_direction) // Brake before changing direction
    {
#ifdef ESP32M
        ledcWrite(_motor_no, 0);
#else
        analogWrite(_motor_PWM_pin, 0);
#endif

        digitalWrite(_motor_BRAKE_pin, HIGH);
        delay(100);
        _last_direction = dir;
    }
    // Enable the motors if moving else disable them
    // Serial.println("DIR: " + String(dir));
    // Enable pins if not stopped
    if (dir != 0)
    {
        digitalWrite(_motor_BRAKE_pin, LOW);
    }
    else
    {
#ifdef ESP32M
        ledcWrite(_motor_no, 0);
#else
        analogWrite(_motor_PWM_pin, 0);
#endif
        digitalWrite(_motor_BRAKE_pin, HIGH);
    }

    // Set pwm for Brushless RioRand 400W
    switch (dir)
    {
    case 1:
        // FORWARD
        // Set the speed of each motor

        digitalWrite(_motor_DIRECTION_pin, HIGH);
#ifdef ESP32M
        ledcWrite(_motor_no, motorSpeed);
#else
        analogWrite(_motor_PWM_pin, motorSpeed);
#endif

#ifdef DEBUG
        Serial.println("MF " + String(_motor_no) + ": " + String(motorSpeed));
#endif
        break;
    case -1:
        // BACKWARDS
        // Set the speed of each motor
        digitalWrite(_motor_DIRECTION_pin, LOW);
#ifdef ESP32M
        ledcWrite(_motor_no, motorSpeed);
#else
        analogWrite(_motor_PWM_pin, motorSpeed);
#endif

#ifdef DEBUG
        Serial.println("MB " + String(_motor_no) + ": " + String(motorSpeed));
#endif
        break;
    default: // STOP
        this->stop();
        break;
    }
}

void Motor_BRUSHLESS_400W::stop()
{
#ifdef ESP32M
    ledcWrite(_motor_no, 0);
#else
    analogWrite(_motor_PWM_pin, 0);
#endif

#ifdef DEBUG
    Serial.println("STOP " + String(_motor_no));
#endif
}

// Functions main purpose has to do with Ardupilots Brushed with Relay mode
//  Instead of changing the pwm value for each motor, enable or disable the motor direction on BTS7960
void Motor_BRUSHLESS_400W::motorEnable(int pin, int hilo)
{
    digitalWrite(pin, hilo);
}

void Motor_BRUSHLESS_400W::readEncoder()
{
    int b = digitalRead(_motor_ENCODER_pin);
    if (b > 0)
    {
        _posi++;
    }
    else
    {
        _posi--;
    }
}

motorDrive Motor::calculateMotorSpeeds(uint16_t pwm_X, uint16_t pwm_Y, uint8_t resolution, uint8_t min_left, uint8_t min_right)
{
    // OUTPUTS
    int nMotMixL; // Motor (left)  mixed output           (-128..+127)
    int nMotMixR; // Motor (right) mixed output           (-128..+127)

    // CONFIG
    // - fPivYLimt  : The threshold at which the pivot action starts
    //                This threshold is measured in units on the Y-axis
    //                away from the X-axis (Y=0). A greater value will assign
    //                more of the joystick's range to pivot actions.
    //                Allowable range: (0..+127)
    float fPivYLimit = 32.0;

    // TEMP VARIABLES
    float nMotPremixL; // Motor (left)  premixed output        (-128..+127)
    float nMotPremixR; // Motor (right) premixed output        (-128..+127)
    int nPivSpeed;     // Pivot Speed                          (-128..+127)
    float fPivScale;   // Balance scale b/w drive and pivot    (   0..1   )

    // Map the X and Y axis to positive and negative numbers -  X for direction and Y for speed
    int x = map(pwm_X, 1000, 2000, -128, 127); // map our speed to 0-255 range
    int y = map(pwm_Y, 1000, 2000, -128, 127); // map our speed to 0-255 range

    // Calculate Drive Turn output due to Joystick X input
    if (y >= 0)
    {
        // Forward
        nMotPremixL = (x >= 0) ? 127.0 : (127.0 + x);
        nMotPremixR = (x >= 0) ? (127.0 - x) : 127.0;
    }
    else
    {
        // Reverse
        nMotPremixL = (x >= 0) ? (127.0 - x) : 127.0;
        nMotPremixR = (x >= 0) ? 127.0 : (127.0 + x);
    }

    // Scale Drive output due to Joystick Y input (throttle)
    nMotPremixL = nMotPremixL * y / 128.0;
    nMotPremixR = nMotPremixR * y / 128.0;

    // Now calculate pivot amount
    // - Strength of pivot (nPivSpeed) based on Joystick X input
    // - Blending of pivot vs drive (fPivScale) based on Joystick Y input
    nPivSpeed = x;
    fPivScale = (abs(y) > fPivYLimit) ? 0.0 : (1.0 - abs(y) / fPivYLimit);

    // Calculate final mix of Drive and Pivot
    nMotMixL = (1.0 - fPivScale) * nMotPremixL + fPivScale * (nPivSpeed);
    nMotMixR = (1.0 - fPivScale) * nMotPremixR + fPivScale * (-nPivSpeed);

    // Make absolutely sure they are in range
    nMotMixL = constrain(nMotMixL, -128, 127);
    nMotMixR = constrain(nMotMixR, -128, 127);

    // Set Motor Direction
    motorDrive md;
    md.vra = min_left;
    md.vrb = min_right;
    if (nMotMixL > 0)
    { // Min speed on left motor is 45 and right motor is 55
        // md.lspeed = map(nMotMixL, 1, 128, min_left, 255);
        md.lspeed = map(nMotMixL, 1, 128, 0, 255);
        md.ldir = 1;
    }
    else if (nMotMixL < 0)
    {
        // md.lspeed = map(abs(nMotMixL), 1, 128, min_left, 255);
        md.lspeed = map(abs(nMotMixL), 1, 128, 0, 255);
        md.ldir = -1;
    }
    else
    {
        md.ldir = 0;
        md.lspeed = 0;
    }

    if (nMotMixR > 0)
    {
        // md.rspeed = map(nMotMixR, 1, 128, min_right, 255);
        md.rspeed = map(nMotMixR, 1, 128, 0, 255);
        md.rdir = 1;
    }
    else if (nMotMixR < 0)
    {
        // md.rspeed = map(abs(nMotMixR), 1, 128, min_right, 255);
        md.rspeed = map(abs(nMotMixR), 1, 128, 0, 255);
        md.rdir = -1;
    }
    else
    {
        md.rspeed = 0;
        md.rdir = 0;
    }
    // Make sure speed is zero without any adjustments
    if (md.lspeed == 0)
        md.vra = 0;
    if (md.rspeed == 0)
        md.vrb = 0;
    md.lspeed = constrain(md.lspeed, md.vra, 255);
    md.rspeed = constrain(md.rspeed, md.vrb, 255);

    return md;
}

motorDrive Motor::calculateTurn(int16_t nJoyX, int16_t nJoyY, uint16_t dead_zone)
{
    // Differential Steering Joystick Algorithm
    // ========================================
    //   by Calvin Hass
    //   https://www.impulseadventure.com/elec/robot-differential-steering.html
    //   https://github.com/ImpulseAdventure/
    //
    // Converts a single dual-axis joystick into a differential
    // drive motor control, with support for both drive, turn
    // and pivot operations.
    //

    // Using inputs from Flysky Controller range of 1000 to 2000
    nJoyX = nJoyX - 1000;
    nJoyX = map(nJoyX, 0, 1000, -128, 127);
    nJoyY = nJoyY - 1000;
    nJoyY = map(nJoyY, 0, 1000, -128, 127);

    // INPUTS
    // int     nJoyX;              // Joystick X input                     (-128..+127)
    // int     nJoyY;              // Joystick Y input                     (-128..+127)

    // OUTPUTS
    int nMotMixL; // Motor (left)  mixed output           (-128..+127)
    int nMotMixR; // Motor (right) mixed output           (-128..+127)

    // CONFIG
    // - fPivYLimt  : The threshold at which the pivot action starts
    //                This threshold is measured in units on the Y-axis
    //                away from the X-axis (Y=0). A greater value will assign
    //                more of the joystick's range to pivot actions.
    //                Allowable range: (0..+127)
    float fPivYLimit = 32.0;

    // TEMP VARIABLES
    float nMotPremixL; // Motor (left)  premixed output        (-128..+127)
    float nMotPremixR; // Motor (right) premixed output        (-128..+127)
    int nPivSpeed;     // Pivot Speed                          (-128..+127)
    float fPivScale;   // Balance scale b/w drive and pivot    (   0..1   )

    // Calculate Drive Turn output due to Joystick X input
    if (nJoyY >= 0)
    {
        // Forward
        nMotPremixL = (nJoyX >= 0) ? 127.0 : (127.0 + nJoyX);
        nMotPremixR = (nJoyX >= 0) ? (127.0 - nJoyX) : 127.0;
    }
    else
    {
        // Reverse
        nMotPremixL = (nJoyX >= 0) ? (127.0 - nJoyX) : 127.0;
        nMotPremixR = (nJoyX >= 0) ? 127.0 : (127.0 + nJoyX);
    }

    // Scale Drive output due to Joystick Y input (throttle)
    nMotPremixL = nMotPremixL * nJoyY / 128.0;
    nMotPremixR = nMotPremixR * nJoyY / 128.0;

    // Serial.println("NL" + String(nMotPremixL)+ '\t'+ "NR" + String(nMotPremixR));

    // Now calculate pivot amount
    // - Strength of pivot (nPivSpeed) based on Joystick X input
    // - Blending of pivot vs drive (fPivScale) based on Joystick Y input
    nPivSpeed = nJoyX;
    fPivScale = (abs(nJoyY) > fPivYLimit) ? 0.0 : (1.0 - abs(nJoyY) / fPivYLimit);

    // Calculate final mix of Drive and Pivot
    nMotMixL = (1.0 - fPivScale) * nMotPremixL + fPivScale * (nPivSpeed);
    nMotMixR = (1.0 - fPivScale) * nMotPremixR + fPivScale * (-nPivSpeed);

    // Make absolutely sure they are in range
    nMotMixL = constrain(nMotMixL, -128, 127);
    nMotMixR = constrain(nMotMixR, -128, 127);

    // Serial.println("NL" + String(nMotMixL)+ '\t'+ "NR" + String(nMotMixR));

    // Set Motor Direction
    motorDrive md;
    (dead_zone == 0) ? dead_zone = 1 : dead_zone;

    if (nMotMixL > 0)
    {
        md.rspeed = map(nMotMixL, 1, 128, dead_zone, 255); // replace the 1's with dead_zone
        md.rdir = 1;
    }
    else if (nMotMixL < 0)
    {
        md.rspeed = map(abs(nMotMixL), 1, 128, dead_zone, 255); // replace the 1's with dead_zone
        md.rdir = -1;
    }
    else
    {
        md.rdir = 0;
        md.rspeed = 0;
    }

    if (nMotMixR > 0)
    {
        md.lspeed = map(nMotMixR, 1, 128, dead_zone, 255); // replace the 1's with dead_zone
        md.ldir = 1;
    }
    else if (nMotMixL < 0)
    {
        md.lspeed = map(abs(nMotMixR), 1, 128, dead_zone, 255); // replace the 1's with dead_zone
        md.ldir = -1;
    }
    else
    {
        md.lspeed = 0;
        md.ldir = 0;
    }

    return md;

    /*KEEP THIS FOR NOW - ODRIVE
    Serial.println("NL" + String(nMotMixL));
    Serial.println("NR" + String(nMotMixR));
    // Convert to Motor PWM range
    struct PWM pwm;
    pwm.left_voltage = fmap(nMotMixL, -128, 127, min_voltage, max_voltage);
    pwm.right_voltage = fmap(nMotMixR, -128, 127, min_voltage, max_voltage);
    //Remove the dead zone
    ((pwm.left_voltage > 0 and pwm.left_voltage < dead_zone) || (pwm.left_voltage < 0 && pwm.left_voltage > (-1 * dead_zone))) ? pwm.left_voltage = 0 : pwm.left_voltage;
    ((pwm.right_voltage > 0 and pwm.right_voltage < dead_zone) || (pwm.right_voltage < 0 && pwm.right_voltage > (-1 * dead_zone))) ? pwm.right_voltage = 0 : pwm.right_voltage;
    return pwm;
    //odrive.SetVelocity(1, -pwm.left_voltage);
    //odrive.SetVelocity(0, pwm.right_voltage);
  */
}

// Map floating point data
float Motor::fmap(float sensorValue, float sensorMin, float sensorMax, float outMin, float outMax)
{
    return (sensorValue - sensorMin) * (outMax - outMin) / (sensorMax - sensorMin) + outMin;
}
