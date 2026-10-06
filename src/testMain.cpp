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


//Offsets
int topXGyroOffset = 0;
int topYGyroOffset = 0;
int topZGyroOffset = 0;
int topXAccelOffset = 0;
int topYAccelOffset = 0;
int topZAccelOffset = 0;



volatile bool fifoReady = false;
volatile bool tick500Hz = false;
volatile bool tick2500Hz = false;

void fifoISR() {
  fifoReady = true;
}

void timer500HzCallback() {
    tick500Hz = true;
}

void timer2500HzCallback() {
    tick2500Hz = true;
}

HardwareTimer timer500Hz(TIM2);
HardwareTimer timer2500Hz(TIM3);

FOC_Control_20kHz()
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

PID_Loop()
{
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
            mpu.dmpGetQuaternion(&q, FIFOBuffer);
            mpu.dmpGetGravity(&gravity, &q);
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

    
    // Configure Timer1 for 20 kHz
    // Formula: freq = F_CPU / (prescaler * (1 + OCR1A))
    // For 16 MHz, prescaler=1, target=20000 Hz:
    // OCR1A = 16,000,000 / 20,000 - 1 = 799
    noInterrupts();
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    TCCR1B |= (1 << WGM12);     // CTC mode
    TCCR1B |= (1 << CS10);      // Prescaler = 1
    OCR1A = 799;                // Compare value for 20 kHz
    TIMSK1 |= (1 << OCIE1A);    // Enable compare match interrupt -- Links to ISR(TIMER1_COMPA_vect)
    interrupts();

    Wire.begin();
    Wire.setClock(400000);

    Serial.begin(115200); //115200 is required for Teapot Demo output
    while (!Serial);
    
}

void main()
{
    Gyro topGyro(topInterruptPin, 0x68, topXGyroOffset, topYGyroOffset, topZGyroOffset, topXAccelOffset, topYAccelOffset, topZAccelOffset);
    topGyro();
    if (fifoReady) {
    fifoReady = false;

    uint8_t status = mpu.getIntStatus();

        // optional: check if this was a data-ready event
        if (status & 0x01) {  // DATA_RDY_INT bit in MPU6050
            PID_Loop();
        }
    }

    if (tick2500Hz) {
        FOC_Control_20kHz();
    }
}


