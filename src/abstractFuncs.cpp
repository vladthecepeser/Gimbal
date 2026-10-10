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
    const char role;
    const int polePairCount;
    const float elecAngleOffset;
    const float mechAngleOffset; 

    //Computation
    float pastTheta = 0.0f;
    float theta = 0.0f;
    float currentCumulativeTheta = 0.0f;
    bool angleInitialized = false;
    float electricalAngle;
    const float pi = 3.14159265358979323846f;
    const float twoPi = 2.0f * pi;
    const float electModulo = 2.0f * pi / polePairCount;

    int V_quadrature;
    int V_beta;
    int V_alpha;

    int V_a;
    int V_b;
    int V_c;

public:
    FOCMotor(const char& Role, 
        const int PolePairCount,
        const float ElecAngleOffset,
        const float MechAngleOffset,
        int APhase, 
        int BPhase, 
        int CPhase, 
        int DirectionPin,
        TwoWire* Bus = &Wire)
        : role(Role), 
        polePairCount(PolePairCount),
        elecAngleOffset(ElecAngleOffset),
        mechAngleOffset(MechAngleOffset),
        aPhase(APhase), 
        bPhase(BPhase), 
        cPhase(CPhase), 
        directionPin(DirectionPin),
        encoder(Bus)

    {
        encoder.begin(directionPin);  //  set direction pin.
        encoder.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
    }

    string getRole() const
    {
        return role;
    }

    void updatePosition() //Expected runtime to be at most 500 us or 2kHz
    {                   
        const float theta = encoder.rawAngle() * AS5600_RAW_TO_RADIANS;

        if (encoder.lastError() != AS5600_OK) {
            return;
        }

        if (!angleInitialized) {
            currentCumulativeTheta = theta;
            angleInitialized = true;
            return;
        }

        float difference = theta - pastTheta;
        if (difference > pi) {
            difference -= twoPi;
        } else if (difference < -pi) {
            difference += twoPi;
        }

        pastTheta = theta;
        currentCumulativeTheta += difference;
    
        return;

    }

    void updateFOC(int desiredTorque) //Expected runtime to be at most 500 us or 2kHz
    {
        electricalAngle = theta - elecAngleOffset;
        while (electricalAngle >= electModulo) {
            electricalAngle -= electModulo;
        }
        while (electricalAngle < 0.0f) {
            electricalAngle += electModulo;
        }

        const float sinAngle = sin(electricalAngle);
        const float cosAngle = cos(electricalAngle);

        V_quadrature = desiredTorque;
        V_beta = V_quadrature * sinAngle;
        V_alpha = V_quadrature * cosAngle;

        V_a = V_alpha;
        V_b = (-0.5f * V_alpha) + (sqrt(3.0f) / 2.0f * V_beta);
        V_c = (-0.5f * V_alpha) - (sqrt(3.0f) / 2.0f * V_beta);


        return;
    }
    

    //Use common mode injection. More computationally efficient than
    void applySVPWM()
    {
        
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