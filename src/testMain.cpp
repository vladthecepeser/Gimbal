#include "abstractFuncs.cpp"
#include "basicFuncs.cpp"
#include "GimbalFuncs.h"
#include "arduino.h"
#include <HardwareTimer.h> 

//HardwareTimer.h requires ~0.5 us during each timer interrupt for 40-80 clock
//cycles of abstraction computing. However, it is portable across almost all STMs.

using namespace std;

//pins 
int wireSDA = 18;
int wireSCl = 19;
int wire_1SDA = 20;
int wire_1SCL = 21;
int baseInterruptPin = 17;
int cameraInterruptPin = 16;
constexpr uint8_t baseGyroAddress = 0x68;
constexpr uint8_t cameraGyroAddress = 0x69;

//Motor specifics
const int polePairCount = 7;

//Offsets
int baseXGyroOffset = 0;
int baseYGyroOffset = 0;
int baseZGyroOffset = 0;
int baseXAccelOffset = 0;
int baseYAccelOffset = 0;
int baseZAccelOffset = 0;

int cameraXGyroOffset = 0;
int cameraYGyroOffset = 0;
int cameraZGyroOffset = 0;
int cameraXAccelOffset = 0;
int cameraYAccelOffset = 0;
int cameraZAccelOffset = 0;


//ISR flags
volatile bool tick2500Hz = false;
volatile bool baseFifoInterrupt = false;
volatile bool cameraFifoInterrupt = false;

//Global Vars
Quaternion qBase;
Quaternion qcamera;

Gyro* baseGyro = nullptr;
Gyro* cameraGyro = nullptr;

TwoWire i2cGyro(PB7, PA15);
TwoWire i2cYaw(PF0, PC4);
TwoWire i2cRoll(PC11, PA8);
TwoWire i2cPitch(PC7, PC6);

void timer2500HzCallback() {
    tick2500Hz = true;
}

void baseGyroISR() {
    baseFifoInterrupt = true;
}

void cameraGyroISR() {
    cameraFifoInterrupt = true;
}

HardwareTimer timer2500Hz(TIM3);

void FOC_Control_2500Hz()
{
    for (int i = 0; i < 3; i++)
    {
        Motors[i].updatePosition();  //gets angle and calculates electrical angle
                                    //Realistically only updates every 200 uS
        Motors[i].updateFOC();      //includes inverse_park and SVPWM calculation

        Motors[i].applySVPWM();     //applies the calculated SVPWM to the motor
    }

    //Code separated to make class easier to change
        //* encoder can be easily changed
        //* PWM pins easily changed
        //* FOC algorithm should mostly remain intact
}

void PID_Loop()
{
    for (int i = 0; i < 3; i++)
    {
        errors[i] = desiredAngle[i] - baseGyro->getAngle(i);

        PIDS[i].updatePID(errors[i], pastErrors[i], dt);
    }

    PastErrors = errors;
}

  
void setup()
{
    TIM1_PWM_Init();
    TIM8_PWM_Init();
    TIM20_PWM_Init();

    FOCMotor Motors[]
    
    Motors[0] = FOCMotor("Pitch Motor", aPhasePitch, bPhasePitch, cPhasePitch, SDAPitch, SCLPitch); //pitch motor
    Motors[1] = FOCMotor("Roll Motor", aPhaseRoll, bPhaseRoll, cPhaseRoll, SDARoll, SCLRoll); //roll
    Motors[2] = FOCMotor("Yaw Motor", aPhaseYaw, bPhaseYaw, cPhaseYaw, SDAYaw, SCLYaw); //yaw

    PID PIDS[]

    PIDS[0] = PID(kpPitch, kiPitch, kdPitch); //pitch PID
    PIDS[1] = PID(kpRoll, kiRoll, kdRoll); //roll PID
    PIDS[2] = PID(kpYaw, kiYaw, kdYaw); //yaw PID

    timer2500Hz.setOverflow(2500, HERTZ_FORMAT);
    timer2500Hz.attachInterrupt(timer2500HzCallback);
    timer2500Hz.resume();

    i2cGyro.begin();
    i2cGyro.setClock(400000);
    i2cYaw.begin();
    i2cYaw.setClock(400000);
    i2cRoll.begin();
    i2cRoll.setClock(400000);
    i2cPitch.begin();
    i2cPitch.setClock(400000);

    Serial.begin(115200); //115200 is required for Teapot Demo output
    while (!Serial);

    static Gyro baseGyroInstance(
        baseInterruptPin,
        baseGyroAddress,
        baseGyroISR,
        baseXGyroOffset,
        baseYGyroOffset,
        baseZGyroOffset,
        baseXAccelOffset,
        baseYAccelOffset,
        baseZAccelOffset,
        &i2cGyro);
    static Gyro cameraGyroInstance(
        cameraInterruptPin,
        cameraGyroAddress,
        cameraGyroISR,
        cameraXGyroOffset,
        cameraYGyroOffset,
        cameraZGyroOffset,
        cameraXAccelOffset,
        cameraYAccelOffset,
        cameraZAccelOffset,
        &i2cGyro);

    baseGyro = &baseGyroInstance;
    cameraGyro = &cameraGyroInstance;
}

void loop()
{
    noInterrupts();
    const bool bothGyroInterrupts = baseFifoInterrupt && cameraFifoInterrupt;
    if (bothGyroInterrupts) {
        baseFifoInterrupt = false;
        cameraFifoInterrupt = false;
    }
    interrupts();

    if (bothGyroInterrupts) {
        const bool bothDmpPacketsReady =
            baseGyro->isDmpPacketReady() &&
            cameraGyro->isDmpPacketReady();

        if (bothDmpPacketsReady && baseGyro->getQuaternion(qBase) && cameraGyro->getQuaternion(qcamera)) {
            PID_Loop();
        } else {
            baseFifoInterrupt = true;
            cameraFifoInterrupt = true;
        }
    }

    if (tick2500Hz) {
        tick2500Hz = false;
        FOC_Control_2500Hz();
    }
}
