#include "mbed.h"
#include "rtos.h"

//Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12
 
//Incremental encoder input pins
#define CHA   D7
#define CHB   D8  
 
//Motor Drive output pins   //Mask in output byte
#define L1Lpin D4           //0x01
#define L1Hpin D5           //0x02
#define L2Lpin D3           //0x04
#define L2Hpin D6           //0x08
#define L3Lpin D9           //0x10
#define L3Hpin D10          //0x20
 
#define INTERVAL 0.1
//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};
 
//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed
 
//Phase lead to make motor spin
int8_t lead = -2;  //2 for forwards, -2 for backwards
int8_t orState = 0;    //Rotot offset at motor state 0


//Status LED
DigitalOut led1(LED1);
 
//Photointerrupter inputs
DigitalIn I1(I1pin);
DigitalIn I2(I2pin);
DigitalIn I3(I3pin);
 
//Motor Drive outputs
DigitalOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
DigitalOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
DigitalOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

int mode =0;
float R = 0x7f800000, V = 0x7f800000, oldR = 0x7f800000, oldV = 0x7f800000;
    float r, v;
 
//Set a given drive state
void motorOut(int8_t driveState){
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
      
    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    if (driveOut & 0x01) L1L = 1;
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L = 1;
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L = 1;
    if (driveOut & 0x20) L3H = 0;
    }
    
    //Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
}
 
//Basic synchronisation routine    
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(1.0);
    
    //Get the rotor state
    return readRotorState();
}

//////////////////////////////////////////////////////////////////

InterruptIn Int1(I1pin);
InterruptIn Int2(I2pin);
InterruptIn Int3(I3pin);

Ticker SCounter;

int8_t direction = 1;

uint32_t rotationCount = 0;

int8_t currentState = 0;
uint16_t speedCounter = 0;
uint16_t currentSpeed = 0;
uint16_t speedBuf[6] = {0,0,0,0,0,0};

const uint16_t speedCountFreq = 10000;



void updateSCounter(){
    speedCounter++;
}

float speedAVG(uint16_t currentSpeed){
    uint16_t temp = 0;
    for(int i = 1; i < 6; i++){
        speedBuf[i-1] = speedBuf[i];
        temp += speedBuf[i];
    }
    temp += currentSpeed;
    speedBuf[5] = currentSpeed;
    
    return (float)speedCountFreq/(float)temp;
}

void updateMotor(float speed){
    //rotate for R rotations, if speed = 0 then use max speed
        if(speedAVG(currentSpeed) > speed){
            motorOut((currentState-orState+lead-direction+6)%6);
        }
        else{
             motorOut((currentState-orState+lead+6)%6);
        }
}

void setDir(float speed, float sign){
    if(speed >= 10 && sign >= 0){
        lead = 2;
        direction = 1;
    }
    else if(speed >= 0 && sign >= 0){
        lead = 1;
        direction = 1;
    }
    else if(speed >= 10 && sign < 0){
        lead = -2;
        direction = -1;
    }
    else{
        lead = -1;
        direction = -1; 
    }
}

float controller(float speed, float rotations_elapsed,float totalR){
        
        float newspeed = 8*(totalR - rotations_elapsed)/speed + 0.5;
        if(newspeed <= 0.5){
            lead = 0;
            direction = 0;
            return 0;
        }
        else if (newspeed < speed){
            return newspeed;
        }
        else{
            return speed;
        }
    }
       
  void updateState(){  //called at all 6 coil angles
    currentState = readRotorState();
    currentSpeed = speedCounter;
    speedCounter = 0;
    rotationCount++;
    
    if(mode == 1){
        r = ((float)rotationCount/6);
                v=controller(30,r, R);
                if(v != 0){
                    setDir(v, V);
                    updateMotor(v);
                }
    }
    else if(mode == 2){
        r = ((float)rotationCount/6);
                //pc.printf("Free run at %f: currentSpeed = %f current r = %f\n\r",V, speedAVG(currentSpeed), r);
                updateMotor(v);
    }
    else if(mode == 3){
        r = ((float)rotationCount/6);
        //pc.printf("rotations left = %f\n\r", R-r);
        v=controller(abs(V),r, R);
        if(v != 0){
            setDir(v, V);
            updateMotor(v);
        }
    }
    
}  
    
/////////////////////////////////////////////////////////////////

//Main
int main() {
    
    
    //Initialise the serial port
    Serial pc(SERIAL_TX, SERIAL_RX);
    pc.printf("Hello\n\r");
    
    //Run the motor synchronisation
    
    orState = motorHome();
    currentState = orState;
    
    pc.printf("Rotor origin: %x\n\r",orState);
    //orState is subtracted from future rotor state inputs to align rotor and motor states
    
    Int1.rise(&updateState);
    Int2.rise(&updateState);
    Int3.rise(&updateState);
    Int1.fall(&updateState);
    Int2.fall(&updateState);
    Int3.fall(&updateState);
    
    SCounter.attach(&updateSCounter, 1/((float)(speedCountFreq)));
    
    
    
    while (1) {
        R = 0x7f800000; V = 0x7f800000, oldR = 0x7f800000, oldV = 0x7f800000;
        pc.scanf("R%7f", &R);  //read in from serial
        pc.scanf("V%7f", &V);
        pc.printf("%f rotations requested at %f Rps\n\r",R, V);
        

        if(R != oldR && V == oldV){//rotate at max speed

            rotationCount = 0;
            setDir(abs(R), R);
            oldR = R;
            V = 30;
            v = abs(V);
            mode = 1;
            while(!pc.readable());
            mode = 0;
            
        }
        else if(R == oldR && V != oldV){//rotate indefinitely at V rps

            rotationCount = 0;
            oldV = V;
            
            setDir(abs(V), V);
            v=abs(V);
            mode = 2;
            while(!pc.readable());
            mode = 0;
        }
        else if(R != oldR && V != oldV){
            
            rotationCount = 0;
            oldR = R; oldV = V;
            //rotate R rotations at V rps
            setDir(abs(V), V);
            v=abs(V);
            mode = 3;
            while(!pc.readable());
            mode = 0;
        }
        else{
            mode = 0;
            //do nothing and wait for next time
            while(!pc.readable());
        }
    }
}
