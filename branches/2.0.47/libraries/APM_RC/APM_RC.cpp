/*
	APM_RC.cpp - Radio Control Library for ArduPirates Arduino Mega with IPWM
	
	Total rewritten by Syberian
	
	Methods:
		Init() : Initialization of interrupts an Timers
		OutpuCh(ch,pwm) : Output value to servos (range : 900-2100us) ch=0..10
		InputCh(ch) : Read a channel input value.  ch=0..7
		GetState() : Returns the state of the input. 1 => New radio frame to process
		             Automatically resets when we call InputCh to read channels
		
*/

/*
APM motor remap to the MultiWii-style

/*
Another remap to foolish the original board:
AP: 0,1,2,3,6,7 - motors, 4,5 - camstab (who the f*ck has implemented this???)
mw: 0,1,3,4,5,6 - motors



*/


#include "APM_RC.h"

#include <avr/interrupt.h>
#include "WProgram.h"


//######################################################
// ENABLE serial PPM receiver by uncommenting the line below

//#define SERIAL_SUM


//##################################################
// Define one from your Receiver/Serial channels set. This can be used for both the PPM SUM and the normal point-to-point receiver to arduino connections

//#define TX_set1				//Graupner/Spektrum					PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,CAMPITCH,CAMROLL

//#define TX_standard				//standard  PPM layout Robbe/Hitec/Sanwa	ROLL,PITCH,THROTTLE,YAW,MODE,AUX2,CAMPITCH,CAMROLL

//#define TX_standard_mode6				//standard, Mode channel is 6  PPM layout Robbe/Hitec/Sanwa	ROLL,PITCH,THROTTLE,YAW,AUX1,MODE,CAMPITCH,CAMROLL

//#define TX_set2				// some Hitec/Sanwa/others				PITCH,ROLL,THROTTLE,YAW,AUX1,AUX2,CAMPITCH,CAMROLL

#define TX_mwi				// MultiWii layout					ROLL,THROTTLE,PITCH,YAW,AUX1,AUX2,CAMPITCH,CAMROLL

//##################################################


#if (!defined(__AVR_ATmega1280__))&&(!defined(__AVR_ATmega2560__))
# error Please check the Tools/Board menu to ensure you have selected Arduino Mega as your target.
#else

// Variable definition for Input Capture interrupt
volatile unsigned int ICR4_old;
volatile unsigned char PPM_Counter=0;
volatile uint16_t PWM_RAW[8] = {2400,2400,2400,2400,2400,2400,2400,2400};
volatile unsigned char radio_status=0;


// ******************
// rc functions split channels
// ******************
#define MINCHECK 800
#define MAXCHECK 2200

volatile int16_t failsafeCnt = 0;

volatile uint16_t rcPinValue[8] = {900,1500,1500,1500,1500,1500,1500,1500}; // interval [1000;2000]
static int16_t rcData[8] ; // interval [1000;2000]
static int16_t rcCommand[4] ; // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW 
static int16_t rcHysteresis[8] ;
static int16_t rcData4Values[8][4];


// ***PPM SUM SIGNAL***
volatile uint16_t rcValue[8] = {1500,1500,1500,1500,1500,1500,1500,1500}; // interval [1000;2000]

// Configure each rc pin for PCINT
void configureReceiver() {
    for (uint8_t chan = 0; chan < 8; chan++)
      for (uint8_t a = 0; a < 4; a++)
        rcData4Values[chan][a] = 1500; //we initiate the default value of each channel. If there is no RC receiver connected, we will see those values
      // PCINT activated only for specific pin inside [A8-A15]
      DDRK = 0;  // defined PORTK as a digital port ([A8-A15] are consired as digital PINs and not analogical)
      PORTK   = (1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<6) | (1<<7); //enable internal pull ups on the PINs of PORTK
#ifdef SERIAL_SUM
	  PCMSK2=1;
#else
	  PCMSK2 = 255;
#endif
	  PCMSK0 = B00010000; // sonar port B4 - d10 echo
      PCICR   = B101; // PCINT activated only for PORTK dealing with [A8-A15] PINs

	  //Remember the registers not declared here remains zero by default... 
// timer5	  
}

ISR(PCINT2_vect) { //this ISR is common to every receiver channel, it is call everytime a change state occurs on a digital pin [D2-D7]
static  uint8_t mask;
static  uint8_t pin;
static  uint16_t cTime,dTime;
static uint16_t edgeTime[8];
static uint8_t PCintLast;
  cTime = TCNT5;         // from sonar
    pin = PINK;             // PINK indicates the state of each PIN for the arduino port dealing with [A8-A15] digital pins (8 bits variable)
mask = pin ^ PCintLast;   // doing a ^ between the current interruption and the last one indicates wich pin changed
  sei();                    // re enable other interrupts at this point, the rest of this interrupt is not so time critical and can be interrupted safely
  PCintLast = pin;          // we memorize the current state of all PINs [D0-D7]

#ifdef SERIAL_SUM
static uint8_t pps_num=0;
static uint8_t pps_etime=0;

    if (!(pin & 1)) {
      dTime = cTime-pps_etime; 
		if (dTime<4400) {rcPinValue[pps_num] = dTime>>1;
		pps_num++;pps_num&=7;} // upto 8 packets in slot
		else pps_num=0; 
    } else pps_etime = cTime; 

#else // generic split PPM  
  // mask is pins [D0-D7] that have changed // the principle is the same on the MEGA for PORTK and [A8-A15] PINs
  // chan = pin sequence of the port. chan begins at D2 and ends at D7
  if (mask & 1<<0)    
    if (!(pin & 1<<0)) {
      dTime = cTime-edgeTime[0]; if (1600<dTime && dTime<4400) rcPinValue[0] = dTime>>1; 
    } else edgeTime[0] = cTime; 
  if (mask & 1<<1)      
    if (!(pin & 1<<1)) {
      dTime = cTime-edgeTime[1]; if (1600<dTime && dTime<4400) rcPinValue[1] = dTime>>1; 
    } else edgeTime[1] = cTime;
  if (mask & 1<<2) 
    if (!(pin & 1<<2)) {
      dTime = cTime-edgeTime[2]; if (1600<dTime && dTime<4400) rcPinValue[2] = dTime>>1; 
    } else edgeTime[2] = cTime;
  if (mask & 1<<3)
    if (!(pin & 1<<3)) {
      dTime = cTime-edgeTime[3]; if (1600<dTime && dTime<4400) rcPinValue[3] = dTime>>1;
    } else edgeTime[3] = cTime;
  if (mask & 1<<4) 
    if (!(pin & 1<<4)) {
      dTime = cTime-edgeTime[4]; if (1600<dTime && dTime<4400) rcPinValue[4] = dTime>>1;
    } else edgeTime[4] = cTime;
  if (mask & 1<<5)
    if (!(pin & 1<<5)) {
      dTime = cTime-edgeTime[5]; if (1600<dTime && dTime<4400) rcPinValue[5] = dTime>>1;
    } else edgeTime[5] = cTime;
  if (mask & 1<<6)
    if (!(pin & 1<<6)) {
      dTime = cTime-edgeTime[6]; if (1600<dTime && dTime<4400) rcPinValue[6] = dTime>>1;
    } else edgeTime[6] = cTime;
  if (mask & 1<<7)
    if (!(pin & 1<<7)) {
      dTime = cTime-edgeTime[7]; if (1600<dTime && dTime<4400) rcPinValue[7] = dTime>>1;
    } else edgeTime[7] = cTime;
#endif	
}
/* RC standard matrix (we are using analog inputs A8..A15 of MEGA board)
0	Aileron
1	Elevator
2	Throttle
3	Rudder
4	RX CH 5 Gear
5	RX CH 6 Flaps
5.bis	RX CH 6 Flaps 3 Switch
6	RX CH7
7 *)	RX CH8
*/
//MultiWii compatibility layout:
/*
THROTTLEPIN              //PIN 62 =  PIN A8
ROLLPIN                      //PIN 63 =  PIN A9
PITCHPIN                     //PIN 64 =  PIN A10
YAWPIN                       //PIN 65 =  PIN A11
AUX1PIN                      //PIN 66 =  PIN A12
AUX2PIN                      //PIN 67 =  PIN A13
CAM1PIN                      //PIN 68 =  PIN A14
CAM2PIN                      //PIN 69 =  PIN A15
*/
#ifdef TX_set1
static uint8_t pinRcChannel[8] = {1, 3, 2, 0, 4,5,6,7}; //Graupner/Spektrum
#endif
#ifdef TX_standard
static uint8_t pinRcChannel[8] = {0, 1, 2, 3, 4,5,6,7}; //standard  PPM layout Robbe/Hitec/Sanwa
#endif
#ifdef TX_standard_mode6
static uint8_t pinRcChannel[8] = {0, 1, 2, 3, 5,4,6,7}; //standard layout with swapped 5,6 channels (Mode switch on 6 channel)
#endif
#ifdef TX_set2
static uint8_t pinRcChannel[8] = {1, 0, 2, 3, 4,5,6,7}; // some Hitec/Sanwa/others
#endif
#ifdef TX_mwi
static uint8_t pinRcChannel[8] = {1, 2, 0, 3, 4,5,6,7}; // mapped multiwii to APM layout
#endif


uint16_t readRawRC(uint8_t chan) {
  uint16_t data;
  uint8_t oldSREG;
  oldSREG = SREG;
  cli(); // Let's disable interrupts
    data = rcPinValue[pinRcChannel[chan]]; // Let's copy the data Atomically
  SREG = oldSREG;
  sei();// Let's enable the interrupts
  return data; // We return the value correctly copied when the IRQ's where disabled
}
  
//######################### END RC split channels

// Constructors ////////////////////////////////////////////////////////////////
/*
timer usage:
0 8bit
1 general servo
2 8bit
3 servo3,5,2 (OCR ABC)
4 servo6,7,8
5 rc input and sonar

*/
// Constructors ////////////////////////////////////////////////////////////////

APM_RC_Class::APM_RC_Class()
{
}

// Public Methods //////////////////////////////////////////////////////////////

void APM_RC_Class::Init(void)
{
//We are using JUST 1 timer1 for 16 PPM outputs!!! (Syberian)
  // Init PWM Timer 1
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(12,OUTPUT);
//  pinMode(32,OUTPUT);//cam roll
//  pinMode(33,OUTPUT);// cam pitch
  pinMode(44,OUTPUT);//cam roll L5
  pinMode(45,OUTPUT);// cam pitch L4
  
 // OCR1A=254
  //general servo
  TCCR5A =0; //standard mode with overflow at A and OC B and C interrupts
  TCCR5B = (1<<CS11); //Prescaler set to 8, resolution of 0.5us
  TIMSK5=B00000111; // ints: overflow, capture, compareA
  OCR5A=65510; // approx 10m limit, 33ms period
  OCR5B=3000;
  
  
//motors
  OCR1A = 1800; 
  OCR1B = 1800; 
  ICR1 = 40000; //50hz freq
  TCCR1A =((1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)); // A and B used
  TCCR1B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);
//  ICR1 = 40000; //50hz freq...Datasheet says  (system_freq/prescaler)/target frequency. So (16000000hz/8)/50hz=40000,

  //TCCR3A = (1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1); // ctc on icr
//  TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);
  OCR3A = 1800; 
  OCR3B = 1800; 
  OCR3C = 1800; 
  ICR3 = 40000; //50hz freq
  TCCR3A =((1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1));
  TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);

  OCR4A = 1800; 
  OCR4B = 1800; 
  OCR4C = 1800; 
  ICR4 = 40000; //50hz freq
  TCCR4A =((1<<WGM31)|(1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1));
  TCCR4B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);

  
 configureReceiver();
}



uint16_t OCRxx1[8]={1800,1800,1800,1800,1800,1800,1800,1800,};
//int OCRxx2[8]={1800,1800,1800,1800,1800,1800,1800,1800,};
char OCRstate=7;
/*
D	Port PWM
2	e4	0 3B
3	e5	1 3C
4	g5	2
5	e3	3 3A
6	h3	4 4A
7	h4	5 4B
8	h5	6 5C
9	h6	7
//2nd gro6up
22	a0	8
23	a1	9
24	a2	10
25	a3	11
26	a4	12
27	a5	13
28	a6	14
29	a7	15
*/
ISR(TIMER5_COMPB_vect)
{ // set the corresponding pin to 1
	OCRstate++;
	OCRstate&=15;
switch (OCRstate>>1)
	{//case 0:  if(OCRstate&1)PORTC&=(1<<5)^255; else PORTC|=(1<<5);break; //d32, cam roll
	//case 1: if(OCRstate&1)PORTC&=(1<<4)^255; else PORTC|=(1<<4);break;   //d33, cam pitch
	case 0:  if(OCRstate&1)PORTL&=(1<<5)^255; else PORTL|=(1<<5);break; //d44, cam Roll
	case 1: if(OCRstate&1)PORTL&=(1<<4)^255; else PORTL|=(1<<4);break;   //d45, cam Pitch
	//case 2: PORTG|=(1<<5);break;
	//case 3: PORTE|=(1<<3);break;
	//case 4: PORTH|=(1<<3);break;
	//case 5: PORTH|=(1<<4);break;
	//case 6: PORTH|=(1<<5);break;
	//case 7: PORTH|=(1<<6);break;
	}
	if(OCRstate&1)OCR5B+=5000-OCRxx1[OCRstate>>1]; else OCR5B+=OCRxx1[OCRstate>>1];
}

/*
ch			3		4		1		2		7		8		10		11	
motor mapping
=======================================================================
Pin			2		3		5		6		7		8		11		12
=======================================================================
TRI			S		BC		RC		LC		-		-		-		-	
QuadX		LFW		RBW		RFC		LBC		-		-		-		-	
QuadP		FW		BW		RC		LC		-		-		-		-	
HexaP		BLW		FRC		FW		BC		FLC		BRW		-		-	
HexaX		FLW		BRC		RW		LC		FRC		BLW		-		-	
Y6			LDW		BDW		RDW		LUC		RUC		BUC		-		-	
OCTA_X																	
OCTA_P																	
=============

Motors description:
B- back
R- right
L- left
F- front
U- upper
D- lower
W- clockwise rotation
C- counter clockwise rotation (normal propeller)
S- servo (for tri)

Example: FLDW - front-left lower motor with clockwise rotation (Y6 or Y4)

*/



void APM_RC_Class::OutputCh(unsigned char ch, uint16_t pwm)
{
  pwm=constrain(pwm,MIN_PULSEWIDTH,MAX_PULSEWIDTH);
  pwm<<=1;   // pwm*2;
 
 switch(ch)
  {
    case 0:  OCR3A=pwm; break; //5
    case 1:  OCR4A=pwm; break; //6
    case 2:  OCR3B=pwm; break; //2
    case 3:  OCR3C=pwm; break; //3
    case 4:  OCRxx1[1]=pwm;break;//OCRxx1[ch]=pwm;CAM PITCH
    case 5:  OCRxx1[0]=pwm;break;//OCRxx1[ch]=pwm;CAM ROLL
    case 6:  OCR4B=pwm; break; //7
    case 7:  OCR4C=pwm; break; //8

    case 9:  OCR1A=pwm;break;// d11
    case 10: OCR1B=pwm;break;// d12
    case 8:  //2nd group
    case 11:
    case 12:
    case 13:
    case 14:
    case 15: break;//OCRxx2[ch-8]=(tempx*5)/79;break;
  } 
}

uint16_t APM_RC_Class::InputCh(unsigned char ch)
{
  uint16_t result;
  uint16_t result2;
  
  // Because servo pulse variables are 16 bits and the interrupts are running values could be corrupted.
  // We dont want to stop interrupts to read radio channels so we have to do two readings to be sure that the value is correct...
result=readRawRC(ch); 
  
  // Limit values to a valid range
  result = constrain(result,MIN_PULSEWIDTH,MAX_PULSEWIDTH);
  radio_status=1; // Radio channel read
  return(result);
}

unsigned char APM_RC_Class::GetState(void)
{
return(1);// always 1
}

// InstantPWM implementation
// This function forces the PWM output (reset PWM) on Out0 and Out1 (Timer5). For quadcopters use
void APM_RC_Class::Force_Out0_Out1(void)
{
  if (TCNT3>5000)  // We take care that there are not a pulse in the output
    TCNT3=39990;   // This forces the PWM output to reset in 5us (10 counts of 0.5us). The counter resets at 40000
  if (TCNT4>5000)
    TCNT4=39990; 
  if (TCNT1>5000)
    TCNT1=39990; 
}
// This function forces the PWM output (reset PWM) on Out2 and Out3 (Timer1). For quadcopters use
void APM_RC_Class::Force_Out2_Out3(void)
{
 // if (TCNT1>5000)
 //   TCNT1=39990;
}
// This function forces the PWM output (reset PWM) on Out6 and Out7 (Timer3). For quadcopters use
void APM_RC_Class::Force_Out6_Out7(void)
{
//  if (TCNT3>5000)
//    TCNT3=39990;
}

// allow HIL override of RC values
// A value of -1 means no change
// A value of 0 means no override, use the real RC values
bool APM_RC_Class::setHIL(int16_t v[NUM_CHANNELS])
{
/*
	uint8_t sum = 0;
	for (unsigned char i=0; i<NUM_CHANNELS; i++) {
		if (v[i] != -1) {
			_HIL_override[i] = v[i];
		}
		if (_HIL_override[i] != 0) {
			sum++;
		}
	}
	radio_status = 1;
	if (sum == 0) {
		return 0;
	} else {
		return 1;
	}
*/
	radio_status = 1;
	return 1;
}

void APM_RC_Class::clearOverride(void)
{
	for (unsigned char i=0; i<NUM_CHANNELS; i++) {
		_HIL_override[i] = 0;
	}
}


// make one instance for the user to use
APM_RC_Class APM_RC;

#endif // defined(ATMega1280)
