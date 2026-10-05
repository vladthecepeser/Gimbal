// Base class
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <vector>

using namespace std;

class FOCMotor
{
private:
    // Motor pins
    int aPhase;
    int bPhase;
    int cPhase;
    int SDAPin;
    int SCLPin;

    // Motor control parameters
    string role;

    int theta;
    int V_quadrature;
    int V_beta;
    int V_alpha;

    int V_a;
    int V_b;
    int V_c;

public:
    FOCMotor(const string& role, int aPhase, int bPhase, int cPhase, int SDAPin, int SCLPin)
        : role(role), aPhase(aPhase), bPhase(bPhase), cPhase(cPhase), SDAPin(SDAPin), SCLPin(SCLPin)
    {
    }

    string getRole() const
    {
        return role;
    }

    void updateEncoder()
    {
    }

    void updateFOC()
    {
    }

    void applySVPWM()
    {
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
        voltageQuadrature = kp*error + ki*(1/2)*dt*(pasterror + error) + kd*(pastError - error)/dt;
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
    Quaternion q;
public:
    Gyro(int gyroInterruptPin) : gyroInterruptPin(gyroInterruptPin), DMPReady(false) {
        mpu.initialize();
        pinMode(gyroInterruptPin, INPUT);
        if(mpu.testConnection() == false){
            Serial.println("MPU6050 connection failed");
            while(true);
        }
        devStatus = mpu.dmpInitialize();

        mpu.setXGyroOffset(0);
        mpu.setYGyroOffset(0);
        mpu.setZGyroOffset(0);
        mpu.setXAccelOffset(0);
        mpu.setYAccelOffset(0);
        mpu.setZAccelOffset(0);

        if (devStatus == 0) {
            mpu.CalibrateAccel(6);  
            mpu.CalibrateGyro(6);
            mpu.setDMPEnabled(true);
            pinMode(gyroInterruptPin, INPUT);
            attachInterrupt(digitalPinToInterrupt(gyroInterruptPin), fifoISR, RISING);
            DMPReady = true;
        } 
        else {
            Serial.print(F("DMP Initialization failed (code ")); 
            Serial.print(devStatus);
            Serial.println(F(")"));
        }
    }

    Quaternion getQuaternion() {
        if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 
            /* Display Quaternion values in easy matrix form: [w, x, y, z] */
            mpu.dmpGetQuaternion(&q, FIFOBuffer);
            return q;
        }
        else return -1; ////////////////////!!!!!!!!!!!!!!!!!!!CHECKCHECKCHEKC!!
    }


};


void loop() {
    if (!DMPReady) {
        print("GYRO NOT WORKING!!! --- DMP FAILURE!!!");
        return;
    }
    
    /* Read a packet from FIFO */
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 

    /* Display Quaternion values in easy matrix form: [w, x, y, z] */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    Serial.print("quat\t");
    Serial.print(q.w);
    Serial.print("\t");
    Serial.print(q.x);
    Serial.print("\t");
    Serial.print(q.y);
    Serial.print("\t");
    Serial.println(q.z);
    }
}