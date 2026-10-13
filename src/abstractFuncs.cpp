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

    void (*setPhaseA)(unsigned int);
    void (*setPhaseB)(unsigned int);
    void (*setPhaseC)(unsigned int);

    // Pins
    int directionPin;

    // Motor control parameters
    const char role;
    const int polePairCount;
    float elecAngleOffset; // Electrical angle offset for calibration
    const float mechAngleOffset  = 0.0f; // Mechanical angle offset for calibration

    //Computation
    float pastTheta = 0.0f;
    float theta = 0.0f;
    float currentCumulativeTheta = 0.0f;
    bool angleInitialized = false;
    float electricalAngle;
    static const float pi = 3.14159265358979323846f;
    static const float twoPi = 2.0f * pi;
    static const float invSqrt3 = 0.57735027f;
    static const float sqrt3Over2 = 0.866025403f;
    const float electModulo;

public:
    FOCMotor(const char& Role,
        const int PolePairCount,
        void (*APhase)(unsigned int),
        void (*BPhase)(unsigned int),   
        void (*CPhase)(unsigned int),
        int DirectionPin,
        TwoWire* Bus = &Wire)
        : role(Role), 
        polePairCount(PolePairCount),
        setPhaseA(APhase), 
        setPhaseB(BPhase), 
        setPhaseC(CPhase), 
        directionPin(DirectionPin),
        encoder(Bus),
        electModulo(2.0f * pi / static_cast<float>(polePairCount))

    {
        encoder.begin(directionPin);  //  set direction pin.
        encoder.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.

    }

    char getRole() const
    {
        return role;
    }

    void updatePosition() //Expected runtime to be at most 500 us or 2kHz
    {                   
        const float measuredTheta = encoder.rawAngle() * AS5600_RAW_TO_RADIANS;

        if (encoder.lastError() != AS5600_OK) {
            return;
        }

        theta = measuredTheta;
        if (!angleInitialized) {
            pastTheta = measuredTheta;
            currentCumulativeTheta = measuredTheta;
            angleInitialized = true;
            return;
        }

        float difference = measuredTheta - pastTheta;
        if (difference > pi) {
            difference -= twoPi;
        } else if (difference < -pi) {
            difference += twoPi;
        }

        pastTheta = measuredTheta;
        currentCumulativeTheta += difference;
    
        return;

    }

    void updateFOC(float desiredVoltage) //Expected runtime to be at most 500 us or 2kHz
    { //expected input range -1 to 1; sign selects torque direction

        electricalAngle = theta - elecAngleOffset;
        while (electricalAngle >= electModulo) {
            electricalAngle -= electModulo;
        }
        while (electricalAngle < 0.0f) {
            electricalAngle += electModulo;
        }

        electricalAngle *= static_cast<float>(polePairCount);

        if (desiredVoltage < -1.0f) desiredVoltage = -1.0f;
        if (desiredVoltage > 1.0f)  desiredVoltage = 1.0f;

        const float vQuadrature = (desiredVoltage * 3400.0f) * invSqrt3;
        const float vBeta = vQuadrature * cosf(electricalAngle);
        const float vAlpha = -vQuadrature * sinf(electricalAngle);

        //Below is SVPWM. Use common mode injection. 
        //More computationally efficient than sector-dwelling and vector lookups.
        const float vA = vAlpha;
        const float vB = (-0.5f * vAlpha) + (sqrt3Over2 * vBeta);
        const float vC = (-0.5f * vAlpha) - (sqrt3Over2 * vBeta);

        const float maxPhase = fmaxf(vA, fmaxf(vB, vC));
        const float minPhase = fminf(vA, fminf(vB, vC));
        const float offset = -(maxPhase + minPhase) / 2.0f;

        const float midpoint = 1700.0f;
        const float ccrMax = 3400.0f;

        const float dutyA = (vA + offset + midpoint < 0.0f) ? 0.0f : (vA + offset + midpoint > ccrMax) ? ccrMax : (vA + offset + midpoint);
        const float dutyB = (vB + offset + midpoint < 0.0f) ? 0.0f : (vB + offset + midpoint > ccrMax) ? ccrMax : (vB + offset + midpoint);
        const float dutyC = (vC + offset + midpoint < 0.0f) ? 0.0f : (vC + offset + midpoint > ccrMax) ? ccrMax : (vC + offset + midpoint);

        setPhaseA(static_cast<unsigned int>(dutyA));
        setPhaseB(static_cast<unsigned int>(dutyB));
        setPhaseC(static_cast<unsigned int>(dutyC));

        return;
    }
    
    void initPosition()
    {
        setPhaseA(static_cast<unsigned int>(1700));
        setPhaseB(static_cast<unsigned int>(3400));
        setPhaseC(static_cast<unsigned int>(0));
    }
    
    void initElecAngle()
    {
        updatePosition();

        electricalAngle = theta;
        while (electricalAngle >= electModulo) {
            electricalAngle -= electModulo;
        }
        while (electricalAngle < 0.0f) {
            electricalAngle += electModulo;
        }
        elecAngleOffset = electricalAngle;
    }

    float getPostition(){
        return(currentCumulativeTheta);
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

    //pin
    int gyroInterruptPin;

    //Management/Call Stuff
    bool DMPReady;
    uint8_t devStatus;
    uint8_t FIFOBuffer[64];

    //Offsets
    int xGyroOffset;
    int yGyroOffset;
    int zGyroOffset;
    int xAccelOffset;
    int yAccelOffset;
    int zAccelOffset;
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
        DMPReady(false),
        xGyroOffset(XGyroOffset),
        yGyroOffset(YGyroOffset),
        zGyroOffset(ZGyroOffset),
        xAccelOffset(XAccelOffset),
        yAccelOffset(YAccelOffset),
        zAccelOffset(ZAccelOffset)
    {
        mpu.initialize();
        pinMode(gyroInterruptPin, INPUT);
        if(mpu.testConnection() == false){
            Serial.println("MPU6050 connection failed");
            while(true);
        }
        devStatus = mpu.dmpInitialize();

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