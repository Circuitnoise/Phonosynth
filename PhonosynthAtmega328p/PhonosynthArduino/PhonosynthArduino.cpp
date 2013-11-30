/*  
	Phonosynth v0.91  coded by Jens Rosenfeld
	
	Waveforms Saw & Sin 256 Segmente

	LFO ist eine Saw (fand ich irgendwie spannender)

	Von den 3 Lautstärke Potis ist nur der Hauptpoti rechts relevant

	Oberste Switch Sw3 entscheidet zwischen Dronesynth & Byteplayer
	Switch2(Dronesynth) -> Dronesynth Engine 2
	Switch3(Dronesynth) -> Dronesynth Engine 3

	Taster S1 Filter on/off
	Taster S2 Mute

	Poti Belegung (Gemeinsam)
	Poti 4 LFO Geschwindigkeit(moduliert Frequenzen)
	Poti 5 LowPass Filter cutoff
	Poti 6 LowPass Filter resonance

	Poti Belegung Dronesynth
	Poti 1-3 Pitch Ozillatoren

	Poti Belegung ByteCodePlayer
	Poti 1  Code Select (18 Codes)
	Poti 2  X Value
	Poti 3  Y Value
	
	Changelog:
	V0.91
	*fix* in ByteBeat mode switch 1 and 2 turns led 1/2 on, that interupts the LED display of the bytebeat selection no.
	*fix* in setLEDs-> turning off LED1 does work
	*add* LED status for drone synth

	Fuse Settings:
	f
*/

#include <MozziGuts.h>					// Mozzi lib
#include <Oscil.h>
#include <LowPassFilter.h>				
#include <mozzi_rand.h>					// Mozzi rand lib
#include <tables/saw2048_int8.h>			// table for Oscils to play
#include <tables/sin2048_int8.h>			// table for Oscils to play
#include <tables/sin512_int8.h>			// table for Oscils to play
#include <Sample.h> // Sample template

//Phonosynth Arduino PINS
#define LED1 8		// 
#define LED2 10		//
#define LED3 7		//
#define LED4 3		//
#define VOICE1	6	//
#define VOICE2	9	//
#define VOICE3	3	//
#define SWITCH1	0	//
#define SWITCH2	1	//
#define SWITCH3	2	//
#define PBUTTON1	5	//
#define PBUTTON2	4	//
#define POTI1	14	//
#define POTI2	15	//
#define POTI3	16	//
#define POTI4	17	//
#define POTI5	18	//
#define POTI6	19	//

#define CONTROL_RATE 64 // defines the update rate of the controls

//OSCILLATORS
Oscil<SAW2048_NUM_CELLS, AUDIO_RATE> oSaw1(SAW2048_DATA);
Oscil<SAW2048_NUM_CELLS, AUDIO_RATE> oSaw2(SAW2048_DATA);
Oscil<SAW2048_NUM_CELLS, AUDIO_RATE> oSaw3(SAW2048_DATA);
Oscil<SIN512_NUM_CELLS, AUDIO_RATE> oLFO1(SIN512_DATA);
Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> oSin1(SIN2048_DATA);
Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> oSin2(SIN2048_DATA);
Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> oSin3(SIN2048_DATA);

//Frequency Modulation

int mod_ratio = 3; // harmonics
long fm_intensity=300; // carries control info from updateControl() to updateAudio()

// LowPassFilter
LowPassFilter lpf;

// Engine selector
static char PhonosynthMode=0, FMModus=0, NoiseModus=0, MutePhonosynth=1,LPFByPass=0;

// DigitalReadValues
static char PButton1Value, PButton2Value,SWITCH1Value,SWITCH2Value,SWITCH3Value;

// Previous DigitalReadValues
static char PButton1Value_PREVIOUS = LOW, PButton2Value_PREVIOUS=LOW;

// AnalogReadValues
static int POTI1Value,POTI2Value,POTI3Value,POTI4Value;

static unsigned char POTI5Value,POTI6Value;

//ByteBeat Variables
static char ByteCode, XValue, YValue, BytecodeSel;

static unsigned long long t=0;

// LED Bits
#define LED1Flag 0
#define LED2Flag 1
#define LED3Flag 2
#define LED4Flag 3

/** Set the Phonosynth leds 1-4
	@note LED 4 is not working, because of pwm usage on the same pin
		  usage: setLEDS(0).. setLEDS(15)
	@param input unsigned char interpreted as byte
	*/
void setLEDs(unsigned char LEDFlag)
{
	if ( LEDFlag & (1 << LED1Flag) )
	{
		digitalWrite(LED1, HIGH); // LED1 ein
	} else
	{
		digitalWrite(LED1, LOW); // LED1 aus
	}
	
	if ( LEDFlag & (1 << LED2Flag) )
	{
		digitalWrite(LED2, HIGH); // LED2 ein
	} else
	{
		digitalWrite(LED2, LOW); // LED2 aus
	}

	if ( LEDFlag & (1 << LED3Flag) )
	{
		digitalWrite(LED3, HIGH); // LED3 ein
	} else
	{
		digitalWrite(LED3, LOW);// LED3 aus
	}

	if ( LEDFlag & (1 << LED4Flag) )
	{
		digitalWrite(LED4, HIGH); // LED4 ein
	} else
	{
		digitalWrite(LED4, LOW);  // LED4 aus
	}
	
}

/** generates Bytecode Output
	@param input unsigned char number of the code to play
	*/
char getByteBeat(unsigned char ByteCodeSel) {

switch (ByteCodeSel) {
	
	case 0:
	ByteCode=0;
	setLEDs(0);
	break;
	case 1:
//	ByteCode = ((-t&4095)*(255&t*(t&t>>13))>>XValue)+(127&t*(234&t>>8&t>>3)>>(3&t>>YValue)); // by tejeez
	ByteCode = t&((t<<3)/((t/((YValue/50)+1)+2000)%4000))*((XValue/2)+3); // by DP
	setLEDs(1);
	break;
	case 2:
	ByteCode = t*(t>>XValue&t>>8&123&t>>YValue); // by tejeez
	setLEDs(2);
	break;
	case 3:
	//ByteCode = (t*((t>>9|t>>XValue)&YValue&t>>6));   // by visy
	ByteCode = (((t*15+(XValue*t)) & (t>>(((9-YValue/25))+1)))) | t*XValue | (t/2)*XValue;
	setLEDs(3);
	break;
	case 4:
	//ByteCode = (t*(t>>5|t>>XValue))>>(t>>(YValue));   // by tejeez
	ByteCode=(t*((XValue+2)/2))^(t/((YValue+3)/2)); //by DP
	setLEDs(4);
	break;
	case 5:
	ByteCode = ((((t*(t>>XValue)|(t>>9))&46&t>>8))^((t&t>>(YValue))|(t>>6))); // by xpansive;
	setLEDs(5);
	break;
	case 6:
	ByteCode = ((t&4096)?((t*(t^t%XValue)|(t>>4))>>1):(t>>YValue)|((t&8192)?t<<2:t)); // by skurk (raer's version)
	setLEDs(6);
	break;
	case 7:
	ByteCode =  t*(t>>XValue&t>>8&YValue&t>>3);
	setLEDs(7);
	break;
	case 8:
	ByteCode = (t*5&(t>>XValue))|(t*3&(t*4>>(YValue))); // by miiro
	setLEDs(8);
	break;
	case 9:
	ByteCode = (t*(t>>5|t>>XValue))>>(t>>(YValue));   // by tejeez
	setLEDs(9);
	break;
	case 10:
	ByteCode = (t*5&(t>>XValue))|(t*3&(t*4>>(YValue))); // by miiro
	setLEDs(10);
	break;
	case 11:
	ByteCode = (t|((t>>XValue)|(t>>7)))*t&(t>>11|t>>YValue); // by red
	setLEDs(11);
	break;
	case 12:
	ByteCode = (t>>6|t|t>>(t>>XValue))*10+((t>>11)&YValue);  //by viznut
	setLEDs(12);
	break;
	case 13:
	ByteCode = t*(XValue+20)>>9|t*42>>YValue;  //by Circuitnoise
	setLEDs(13);
	break;
	case 14:
	ByteCode = t*(XValue+20)>>8|t*42>>YValue;  //by Circuitnoise
	setLEDs(14);
	break;
	case 15:
	ByteCode = (((t>>XValue)|(t>>10))-2)%11*t&(YValue);  //by Circuitnoise
	setLEDs(15);
	break;
	case 16:
	ByteCode = (t>>XValue)|(t&(YValue));  //by Circuitnoise
	setLEDs(1);
	break;
	case 17:
	ByteCode = (t>>XValue*2)|(t&(YValue));  //by Circuitnoise
	setLEDs(2);
	break;
	case 18:
	ByteCode = (t>>XValue*2)|((t&(YValue*4))-3);  //by Circuitnoise
	setLEDs(3);
	break;
}

 return ByteCode;	
}
/** manages the control changes of the DroneEngine
	*/
void updateControlDroneEngine(){
	
	if (FMModus==0){
		// standard drone
		oSaw1.setFreq(POTI1Value);
		oSin1.setFreq(POTI2Value);
		oSaw2.setFreq(POTI3Value);
	}
	else{
		// FM Modulation
		// map the knob to carrier frequency
		int carrier_freq = POTI1Value;

		//calculate the modulation frequency to
		int mod_freq = carrier_freq * mod_ratio;
		// set the FM oscillator frequencies to the calculated values
		oSaw1.setFreq(POTI1Value);
		oSaw2.setFreq(mod_freq);
		oSaw3.setFreq(POTI3Value);
		fm_intensity = POTI2Value;
	}
	
	if (NoiseModus==1) {
		oSaw1.setFreq(POTI1Value);
		oSin1.setFreq(POTI2Value);
		oSin2.setFreq(POTI3Value/23);
		oSaw1.setPhase(rand(SAW2048_NUM_CELLS));
		oSin1.setPhase(rand(SIN2048_NUM_CELLS));

	}

	// LFO frequency
	oLFO1.setFreq((float)POTI4Value/49);
}
/** manages the control changes of the ByteBeatEngine
	*/
void updateControlByteBeatEngine(){
	
	BytecodeSel=(POTI1Value/4)/14; // Range 0-18
	XValue=(POTI2Value/4)/10; // Range 0-25
	YValue=(POTI3Value/4)/10; // Range 0-25

}

/** setup of Phonosynth
	First routine after power on!!
	*/
void setup(){

  // set input/output pins
  pinMode(SWITCH1, INPUT_PULLUP);
  pinMode(SWITCH2, INPUT_PULLUP);
  pinMode(SWITCH3, INPUT_PULLUP);
  pinMode(PBUTTON1, INPUT_PULLUP);
  pinMode(PBUTTON2, INPUT_PULLUP);
  pinMode(POTI1, INPUT);
  pinMode(POTI2, INPUT);
  pinMode(POTI3, INPUT);
  pinMode(POTI4, INPUT);
  pinMode(POTI5, INPUT);        	
  pinMode(POTI6, INPUT);        	
  pinMode(LED1, OUTPUT);        	
  pinMode(LED2, OUTPUT);        	
  pinMode(LED3, OUTPUT);
  
  //Setup Waves
  oSaw1.setFreq(440);
  oSaw2.setFreq(440);
  oSaw3.setFreq(440);
  oSin1.setFreq(440);
  oSin2.setFreq(440);
  oSin3.setFreq(440);
  oLFO1.setFreq(5);

  // Set LowPassFilter Resonance
  lpf.setResonance(0);

  // Start the action!
  startMozzi(CONTROL_RATE);

}

/** Mozzi Loop
	*/
void loop(){
  audioHook();
}

/** Mozzi updateControl routine
	*/
void updateControl(){
	
	// read pushbuttons, switches and potentiometer values
	PButton1Value = digitalRead(PBUTTON1); // on/off
	PButton2Value = digitalRead(PBUTTON2); // on/off
	SWITCH1Value = digitalRead(SWITCH1); // on/off
	SWITCH2Value = digitalRead(SWITCH2); // on/off
	SWITCH3Value = digitalRead(SWITCH3); // on/off
	
	POTI1Value = mozziAnalogRead(POTI1); // value is 0-1023
	POTI2Value = mozziAnalogRead(POTI2); // value is 0-1023
	POTI3Value = mozziAnalogRead(POTI3); // value is 0-1023
	POTI4Value = mozziAnalogRead(POTI4); // value is 0-1023
	POTI5Value = map(mozziAnalogRead(POTI5),0,1023,1,255); // value is 0-255
	POTI6Value = map(mozziAnalogRead(POTI6),0,1023,0,255); // value is 0-255
	
	// set LowPassFilter values
	lpf.setCutoffFreq(POTI5Value);
	lpf.setResonance(POTI6Value);

    // Phonosynth Drones or ByteBeatPlayer selection via Switch3(top one)
    if (SWITCH3Value == HIGH) {
		PhonosynthMode=1;
		}
		else{
		PhonosynthMode=0;
		
		//Noise Modus?
		if (SWITCH1Value == HIGH) {
			setLEDs(6); 
			NoiseModus=1;
		}
		else {
			NoiseModus=0;
			// Frequency Modulation Modus?
			if (SWITCH2Value == HIGH) {
				FMModus=1;
				setLEDs(5);
			}else{
				FMModus=0;
				setLEDs(4);
			}
		} // end of Switch1Value

	}  // end of Phonosynth || ByteBeatPlayer
	
	// LowPassFilter bypass button S1
	if(PButton1Value_PREVIOUS==LOW && PButton1Value==HIGH){
		LPFByPass=0;
		delayMicroseconds(400);
	}else if(PButton1Value_PREVIOUS==HIGH && PButton1Value==LOW){
		LPFByPass=1;
		delayMicroseconds(400);
	}
	PButton1Value_PREVIOUS=PButton1Value;
	

    // Mute Button S2
	if(PButton2Value_PREVIOUS==LOW && PButton2Value==HIGH){
		  MutePhonosynth=1;
		  delayMicroseconds(400);
	  }else if(PButton2Value_PREVIOUS==HIGH && PButton2Value==LOW){
		  MutePhonosynth=0;
		  delayMicroseconds(400);
	  }
	PButton2Value_PREVIOUS=PButton2Value;
	  
	// handle now the specific engine controls
	if (PhonosynthMode==0) {
		updateControlDroneEngine();
	} else {
		updateControlByteBeatEngine();
	}
   
}
/** Mozzi updateAudio routine
	*/
int updateAudio(){

	char Voice=0; // Voice
	
	// Dronesynth=0 || ByteCodePlayer
	if (PhonosynthMode == 0) {
		
		if (FMModus==0){
			// standard drone sound
			Voice = ((((oSaw1.next()*oLFO1.next())>>8)+((oSin1.next()*oLFO1.next())>>8)+((oSaw2.next()*oLFO1.next())>>8)))>>1;
			//Voice = ((((oSaw1.next()*oLFO1.next())>>8)+((oSin1.next()*oLFO1.next())>>8)+((oSaw2.next()*oLFO1.next())>>8)))/2;
			}
			
		if (FMModus==1){
			// FM Modulation
			long modulation = fm_intensity * oSaw1.next();
			//Voice= Voice + (oLFO1.next()*(oSaw2.phMod(modulation)+oSaw3.phMod(modulation))>>8); // phMod does the FM
			Voice= ((Voice + (oLFO1.next()*(oSaw2.phMod(modulation)+oSaw3.phMod(modulation))>>8))>>1); // phMod does the FM
		}
		
		if (NoiseModus==1) {
			// more Noise mode
			Voice = (Voice + (((oSaw1.next()+(oSin1.next()))>>1)+((oSin2.next()*oLFO1.next())>>8)))>>2;
			//Voice = Voice + ((oSaw1.next()+(oSin1.next())+((oSin2.next()*oLFO1.next())>>8))>>2);
		}
			
	} else {  // ByteBeat Modus
	 // increase time
	 t++; 
	 
	 // gets ByteBeatSample 
	 Voice=getByteBeat(BytecodeSel);
	}
	
	// LowPassFilter on/off depends on buttons S2 push
	if (LPFByPass == 0) {
		Voice = lpf.next(Voice);
	}
	
	// mute (multiplier 0/1) depends on buttons S1 push
	return (Voice*MutePhonosynth);
}

