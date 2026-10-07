#include "GimbalFuncs.cpp"
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
int wire_1SDA = 21;
int baseInterruptPin = 17;


//Offsets
int baseXGyroOffset = 0;
int baseYGyroOffset = 0;
int baseZGyroOffset = 0;
int baseXAccelOffset = 0;
int baseYAccelOffset = 0;
int baseZAccelOffset = 0;


volatile bool tick500Hz = false;
volatile bool tick2500Hz = false;


void timer500HzCallback() {
    tick500Hz = true;
}

void timer2500HzCallback() {
    tick2500Hz = true;
}

HardwareTimer timer500Hz(TIM2);
HardwareTimer timer2500Hz(TIM3);

void FOC_Control_2500Hz()
{
    for (int i = 0; i < 3; i++)
    {
        Motors[i].updateEncoder();  //gets angle and calculates electrical angle
                                    //Realistically only updates every 200 uS

        Motors[i].updateFOC();      //includes inverse_park and SVPWM calculation

        Motors[i].applySVPWM();     //applies the calculated SVPWM to the motor
    }

    //Code separated to make class easier to change
        //* encoder can be easily changed
        //* PWM pins easily changed
        //* FOC algorithm should mostly remain intact
}

void PID_Loop_500Hz()
{
    Quaternion q;
    
    if (baseGyro.dmpGetCurrentFIFOPacket(baseGyro.FIFOBuffer)) {
            baseGyro.dmpGetQuaternion(&q, baseGyro.FIFOBuffer);
            baseGyro.dmpGetGravity(&gravity, &q);
    }

    for (int i = 0; i < 3; i++)
    {

        errors[i] = desiredAngle[i] - Gyro.getAngle(i);

        PIDS[i].updatePID(errors[i], pastErrors[i], dt);
    }

    PastErrors = errors;
}

  
void setup()
{

    FOCMotor Motors[]
    
    Motors[0] = FOCMotor("Pitch Motor", aPhasePitch, bPhasePitch, cPhasePitch, SDAPitch, SCLPitch); //pitch motor
    Motors[1] = FOCMotor("Roll Motor", aPhaseRoll, bPhaseRoll, cPhaseRoll, SDARoll, SCLRoll); //roll
    Motors[2] = FOCMotor("Yaw Motor", aPhaseYaw, bPhaseYaw, cPhaseYaw, SDAYaw, SCLYaw); //yaw

    PID PIDS[]

    PIDS[0] = PID(kpPitch, kiPitch, kdPitch); //pitch PID
    PIDS[1] = PID(kpRoll, kiRoll, kdRoll); //roll PID
    PIDS[2] = PID(kpYaw, kiYaw, kdYaw); //yaw PID

    
    timer500Hz.setOverflow(500, HERTZ_FORMAT);
    timer500Hz.attachInterrupt(timer500HzCallback);
    timer500Hz.resume();

    timer2500Hz.setOverflow(2500, HERTZ_FORMAT);
    timer2500Hz.attachInterrupt(timer2500HzCallback);
    timer2500Hz.resume();

    Wire.begin();
    Wire.setClock(400000);

    Serial.begin(115200); //115200 is required for Teapot Demo output
    while (!Serial);
    
}

void main()
{
    Gyro baseGyro(baseInterruptPin, 0x68, baseXGyroOffset, baseYGyroOffset, baseZGyroOffset, baseXAccelOffset, baseYAccelOffset, baseZAccelOffset);

    if (tick500Hz) {
        tick500Hz = false;
        PID_Loop_500Hz();
    }

    if (tick2500Hz) {
        tick2500Hz = false;
        FOC_Control_2500Hz();
    }
}


