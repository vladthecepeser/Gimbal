// Base class
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "AS5600.h"
#include <string>

using namespace std;

class FOCMotor
{
private:

    AS5600 encoder;

    // Pins
    int aPhase;
    int bPhase;
    int cPhase;
    int directionPin;

    // Motor control parameters
    string role;

    //Computation
    float pastTheta = 0.0f;
    float currentTheta = 0.0f;
    bool angleInitialized = false;
    const float pi = 3.14159265358979323846f;

    int V_quadrature;
    int V_beta;
    int V_alpha;

    int V_a;
    int V_b;
    int V_c;

public:
    FOCMotor(const char& role, 
        int aPhase, 
        int bPhase, 
        int cPhase, 
        int directionPin,
        TwoWire* bus = &Wire)
        : role(role), 
        aPhase(aPhase), 
        bPhase(bPhase), 
        cPhase(cPhase), 
        directionPin(directionPin),
        encoder(bus)

    {
        encoder.begin(directionPin);  //  set direction pin.
        encoder.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
    }

    string getRole() const
    {
        return role;
    }

    void updateEncoder() //Expected runtime to be at most 500 us or 2kHz
    {                   
        const float rawTheta = encoder.rawAngle() * AS5600_RAW_TO_RADIANS;

        if (encoder.lastError() != AS5600_OK) {
            return;
        }

        if (!angleInitialized) {
            pastTheta = rawTheta;
            currentTheta = rawTheta;
            angleInitialized = true;
            return;
        }

        float difference = rawTheta - pastTheta;
        const float twoPi = 2.0f * pi;
        if (difference > pi) {
            difference -= twoPi;
        } else if (difference < -pi) {
            difference += twoPi;
        }

        currentTheta += difference;
        pastTheta = rawTheta;
        
        return;

    }

    void updateFOC()
    {
    }

    void applySVPWM()
    {
    }

    float getTheta(){
        return(currentTheta);
    }
};



class PID
{
private:
    int ki;
    int kp;
    int kd;

    int voltageQuadrature;

public:
    PID(int kp, int ki, int kd)
        : kp(kp), ki(ki), kd(kd)
    {
    }

    void setI(int ki){this->ki = ki;
    }
    
    void setP(int kp){this->kp = kp;
    }

    void setD(int kd){this->kd = kd;
    }   

    void updatePID(int error, int pastError, int dt)
    {
        voltageQuadrature = kp*error + ki*(1/2)*dt*(pastError + error) + kd*(pastError - error)/dt;
    }

    int getVoltageQuadrature(){
        return voltageQuadrature;
    }

};


class Gyro
{
private:
    MPU6050 mpu;
    //MPU6050 mpu(0x69); //Use for AD0 high
    //MPU6050 mpu(0x68, &Wire1); //Use for AD0 low, but 2nd Wire (TWI/I2C) object.

    int gyroInterruptPin;
    bool DMPReady;
    uint8_t devStatus;
    uint8_t FIFOBuffer[64];
public:
    Gyro(int interruptPin,
        uint8_t address,
        void (*interruptHandler)(),
        int XGyroOffset,
        int YGyroOffset,
        int ZGyroOffset,
        int XAccelOffset,
        int YAccelOffset,
        int ZAccelOffset,
        TwoWire* bus = &Wire)
        : mpu(address, bus),
          gyroInterruptPin(interruptPin),
          DMPReady(false)
    {
        mpu.initialize();
        pinMode(gyroInterruptPin, INPUT);
        if(mpu.testConnection() == false){
            Serial.println("MPU6050 connection failed");
            while(true);
        }
        devStatus = mpu.dmpInitialize();

        mpu.setXGyroOffset(XGyroOffset);
        mpu.setYGyroOffset(YGyroOffset);
        mpu.setZGyroOffset(ZGyroOffset);
        mpu.setXAccelOffset(XAccelOffset);
        mpu.setYAccelOffset(YAccelOffset);
        mpu.setZAccelOffset(ZAccelOffset);

        if (devStatus == 0) {
            mpu.CalibrateAccel(6);  
            mpu.CalibrateGyro(6);
            mpu.setDMPEnabled(true);
            pinMode(gyroInterruptPin, INPUT);
            attachInterrupt(digitalPinToInterrupt(gyroInterruptPin), interruptHandler, RISING);
            DMPReady = true;
        } 
        else {
            Serial.print(F("DMP Initialization failed (code ")); 
            Serial.print(devStatus);
            Serial.println(F(")"));
        }
    }

    bool getQuaternion(Quaternion& quaternion) {
        if (!mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
            return false;
        }

        mpu.dmpGetQuaternion(&quaternion, FIFOBuffer);
        return true;
    }

    bool isDmpPacketReady() {
        return DMPReady &&
               mpu.getFIFOCount() >= mpu.dmpGetFIFOPacketSize();
    }

};