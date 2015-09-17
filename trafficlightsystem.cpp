/*
* File TrafficLightMaster.cpp
* Created 2013-10-29
*
* A sketch for the Traffic Light Master/Server System.
*
* @author1: Swoorup Joshi
* @author2: Louis Grynfeltt
* @author3: Semjon Khozjainov
* @version 2.0
*
* Semester 02/2013
* Lecture ICT106
* Project 2
*
*/

// traffic light digital pins positioned at 270 degree
const int PIN_GREEN_LIGHT_270 = 49;
const int PIN_ORANGE_LIGHT_270 = 46;
const int PIN_RED_LIGHT_270 = 47;
const int PIN_FILTERTURN_LIGHT_270 = 48;
const int PIN_PED_LIGHT_270 = 45;
const int PIN_PEDCROSS_ALARM_270 = 51;
const int PIN_BTN_PEDCROSS_270 = 50;

// traffic light digital pins positioned at 0 degree
const int PIN_GREEN_LIGHT_0 = 30;
const int PIN_ORANGE_LIGHT_0 = 28;
const int PIN_RED_LIGHT_0 = 26;
const int PIN_FILTERTURN_LIGHT_0 = 24;
const int PIN_PED_LIGHT_0 = 34;
const int PIN_PEDCROSS_ALARM_0 = 32;
const int PIN_BTN_PEDCROSS_0 = 31;

// traffic light digital pins positioned at 90 degree
const int PIN_GREEN_LIGHT_90 = 5;
const int PIN_ORANGE_LIGHT_90 = 6;
const int PIN_RED_LIGHT_90 = 7;
const int PIN_FILTERTURN_LIGHT_90 = 8;
const int PIN_PED_LIGHT_90 = 2;
const int PIN_PEDCROSS_ALARM_90 = 3;
const int PIN_BTN_PEDCROSS_90 = 4;

// button pins for special purpose
const int PIN_BTN_EMERGENCY = 52;
const int PIN_BTN_ROADWORK = 53;
const int PIN_BTN_SELECTSEQUENCE = 41;

// analog pins for calibration
const int PIN_POT = A2;
const int PIN_THER = A0;
const int PIN_LDR = A1;

// time how much the green signal can stay upto or 1 sequence interval
const unsigned int DEFAULT_GO_LIGHT_TIME = 10000;
// time how much the orange light is led up when transiting the green signal to red
const unsigned int DEFAULT_ORANGE_LIGHT_TIME = 5000;

// ADC value for darkness of LDR below which sequence is slowed
const int LDR_DARK_ADC = 30;

// Kelvin temp after which sequence is slowed
const int THER_CRITICAL_DEG = 300;

const int SERIAL_BAUD = 9600;
const int ALARM_FREQ = 560;

// number of sequences
const int MAX_SEQUENCES = 4;

// main sequences array which controls all the sequence of the system
bool sequence[MAX_SEQUENCES][9] = {
    //each three sets of values represent greensignal state, filter turn state, ped light state
    //from left to right state sets: led states of trafficLight 270, trafficLight 0 and trafficlight 90
    {1,0,0,     1,1,0,      0,0,0}, 
    {1,1,0,     1,0,0,      0,0,0},
    {1,0,0,     1,0,0,      1,1,0},
    {0,0,0,     1,1,0,      0,0,0}
};

// buzzer class to indicate the ped crossing
class BUZZER {
private:
    int pin;
    boolean soundAlarm;
    int freq;
    unsigned long lastMillis;
    
public: 
    BUZZER() {
        soundAlarm = false;
        lastMillis = 0;
    }
    
    void SetFrequency(int f) {
        freq = f;
    }
    
    void SetPin(int n) {
        pin = n;
        pinMode(pin, OUTPUT);
    }
    
    void TurnOffAlarm() {
        soundAlarm = false;
    }
    
    void TriggerAlarm() {
        soundAlarm = true;
    }
    
    void buzz(long frequency, long length) {
        long delayValue = 1000000/frequency/2;

        long numCycles = frequency * length / 1000;
    
        for (long i = 0; i < numCycles; i++) {
            digitalWrite(pin, HIGH);
            delayMicroseconds(delayValue);
            digitalWrite(pin, LOW);
            delayMicroseconds(delayValue);
        }
    }
    
    // UpdateAsync allows the buzzer to buzz every 90 ms asynchronously
    void UpdateAsync() {
        if (soundAlarm) {
            unsigned long currentMillis = millis();
            if (currentMillis - lastMillis >= 90) {
                buzz(freq, 20);
                lastMillis = currentMillis;
            }
        }
    }
};

// Button class object to represent all the button functions
class BUTTON {
private:
    int pin;
    int prevState;
public:
    BUTTON() {
        prevState = 0;
    }
    
    void SetPin (int n) {
        pin = n;
        pinMode(pin, INPUT);
    }
    
    int ReadState() {
        return digitalRead(pin);
    }
    
    bool IsPressed() {
        return ReadState() == HIGH;
    }
    
    // this function makes the button act as a electronic switch
    // used for road work mode and emergency mode
    bool ReadSwitch() {
        if (ReadState() == HIGH && prevState == 0)
            prevState = 1;
        
        if (prevState == 1 && ReadState() == LOW)
            prevState = 2;
        
        if (prevState == 2 && ReadState() == HIGH)
            prevState = 3;
            
        if (prevState == 3 && ReadState() == LOW)
            prevState = 0;
            
        if(prevState == 1 || prevState == 2)
            return true;
        else
            return false;
    }
};

// LED object to represent functions led
class LED
{
private:
    int pin;
    int state;
public:
    LED() {
        state = LOW;
    }
    
    int IsOn() {
        return state == HIGH;
    }
        
    void SetPin (int n) {
        pin = n;
        pinMode(pin, OUTPUT);
    }
        
    void SwitchOn() {
        if (state == HIGH) 
            return;
    
        digitalWrite(pin, HIGH);
        state = HIGH;
    }
        
    void SwitchOff() {
        if (state == LOW)
            return;

        digitalWrite(pin, LOW);
        state = LOW;
    }
};

// base class for analog input devices
class SensorBase 
{
protected:
    int pin;
    int vIn;
    boolean bSmoothing;
    
    // smoothing parameters
    // smoothing gets 10 sets of reading and then averages it out
    int readings[10];
    int total;
    int index;
    boolean bFirstInit;
    
public:
    SensorBase() {
        index = 0;
        total = 0;
        bFirstInit = true;
    }
    
    void SetPin(int n) {
        pin = n;
    }
    
    void ApplySmoothing() {
        bSmoothing = true;
        for (int i = 0; i < 10; i++)
        {
            readings[i] = 0;
        }
    }
    
    void SetVIn(int v) {
        vIn = v;
    }
    
    // reads value between 0 to 1023
    int ReadADC() {
        // on smoothing return the average reading
        if (bSmoothing) {
			// cancel out the top previous reading
            total = total - readings[index];
			// overwrite by the new reading
            readings[index] = analogRead(pin);
			// add the new reading
            total = total + readings[index];
            index = index + 1;
            
            if (index >= 10) {
                index = 0;
                bFirstInit = false;
            }
            
            if (bFirstInit)
                return total / index;
            return total / 10;
        }
        return analogRead(pin);
    }
    
    float ReadVolt() {
        return (((ReadADC() + 1) * 5) / 1024.0);
    }
};

// potentiometer does not need to have extended function, 
// so simply inherit from the Sensorbase class 
class POTENTIOMETER : public SensorBase
{
};

// LDR class for LDR component
class LDR : public SensorBase
{
private:
    int r2;
public:
    void SetFixedResistorR2(int rOhm) {
        r2 = rOhm;
    }
    
    float ReadResistenceR1() {
        float r1 = (r2 * (vIn / ReadVolt())) - (float)r2;
        return r1;
    }
};

// Thermistor class for Thermistor
class THERMISTOR : public SensorBase
{
private:
    int r2;
public:
    void SetFixedResistorR2(int rOhm) {
        r2 = rOhm;
    }
    
    float ReadResistenceR1() {
        float r1 = (r2 * (vIn / ReadVolt())) - (float)r2;
        return r1;
    }
        
    double GetKelvinTempUsingHart() {
        static const double c1 = 1.346239e-03;
        static const double c2 = 2.309426e-04;
        static const double c3 = 9.815900e-08;
        double logRt = log(ReadResistenceR1());
        double t = ( 1.0 / (c1 + c2*logRt + c3*logRt*logRt*logRt ) );
        return t;
        
    }
};

// Counter implementation using millis
class Counter {
private:    
    unsigned long lastMillis;
    unsigned long prevCounter;
    boolean paused;
    
public:
    Counter() {
        Reset();
        paused = false;
    }
    
    void Pause() {
        if (paused)
            return;
        prevCounter = GetCounter();
        paused = true;
    }
    
    void UnPause() {
        if (!paused)
            return;
        Reset(prevCounter);
        paused = false;
    }
    
    bool IsPaused() {
        return paused;
    }
    
    unsigned int GetCounter(){
        if (IsPaused())
            return prevCounter;
        return millis() - lastMillis;
    }
    
    void Reset(long val = 0L) {
        lastMillis = millis() - val;
    }
};

// TrafficLight class contains components in one set of traffic
class TRAFFICLIGHT {
public:
    // the standard trafficlight led components
    LED ledGreen;
    LED ledOrange;
    LED ledRed;
    
    // turn light may indicate either to turn right or left
    LED ledFilterTurn;
    
    // led and buzzer to indicate ped crossing
    LED ledPed;
    BUZZER crossingBuzzer;
    
    // counter used to keep orange led on when transitioning from green to red
    Counter OrangeTransitionCounter;
    int transitionState;
    
    TRAFFICLIGHT() {
        OrangeTransitionCounter.Reset();
        transitionState = 0;
    }
    
    void SetPins(int pinGreenLED, int pinOrangeLED, int pinRedLED, int pinFilterTurnLED, int pinPedLED, int pinBuzzer) {
        ledGreen.SetPin(pinGreenLED);
        ledGreen.SwitchOff();
        
        ledOrange.SetPin(pinOrangeLED);
        ledOrange.SwitchOff();
        
        ledRed.SetPin(pinRedLED);
        ledRed.SwitchOff();
        
        ledFilterTurn.SetPin(pinFilterTurnLED);
        ledFilterTurn.SwitchOff();
        
        ledPed.SetPin(pinPedLED);
        ledPed.SwitchOff();
        
        crossingBuzzer.SetPin(pinBuzzer);
        crossingBuzzer.SetFrequency(BUZZER_FREQ);
        crossingBuzzer.TurnOffAlarm();
    }
    
    // this function lights the green light only
    void GreenLight() {
        ledGreen.SwitchOn();
        ledOrange.SwitchOff();
        ledRed.SwitchOff();
    }
    
    // this function lights the red light only
    void RedLight() {
        ledGreen.SwitchOff();
        ledOrange.SwitchOff();
        ledRed.SwitchOn();
    }
    
    // this function lights the orange light only
    void OrangeLight() {
        ledGreen.SwitchOff();
        ledOrange.SwitchOn();
        ledRed.SwitchOff();
    }
    
    // this function lights emergency light only switching off
    // the turn and the ped lights as well
    void EmergencyLightOnly() {
        RedLight();
        ledPed.SwitchOff();
        ledFilterTurn.SwitchOff();
    }
 
    // this function lights the orange light only switching off
    // the turn and the ped light as well
    void RoadWorkLightOnly(){
        OrangeLight();
        ledPed.SwitchOff();
        ledFilterTurn.SwitchOff();
    }
  
    // this function simulates normal light condition when called at every loop
    // it automatically transitions from green to red signal by using the OrangeTransitionCounter
    void UpdateWithLightTransition(bool goSignal, bool turnLight, bool pedLight, int orangeLightTime) {
    
        // if green or go signal turn off transitioning process 
        // and only light up the green light
        if (goSignal) {
            transitionState = 0;
            GreenLight();
        }
        
        // if red signal and green light was previously on
        // start transitioning to red by firt turning on the 
        // orange light
        else if(!goSignal && ledGreen.IsOn() && transitionState == 0) {
            OrangeLight();
            transitionState = 1;
            OrangeTransitionCounter.Reset();
        } 
        // if red signal and greenlight was not on, directly
        // switch on the red light
        else if(!goSignal && !ledGreen.IsOn() && transitionState == 0)
            RedLight();
        
        // check if we are still transitioning to red light
        // if orangeTransitionCounter has passed orangeLightTime interval 
        // turn on the Red light only
        if (transitionState == 1) {
            if (OrangeTransitionCounter.GetCounter() < orangeLightTime)
                OrangeLight();
            else
                transitionState = 2;
        }
        
        if (transitionState == 2)
            RedLight();
            
        if (turnLight)
            ledFilterTurn.SwitchOn();
        else
            ledFilterTurn.SwitchOff();
        
        // ped lights turn on only when the normal lights are in red signal
        if (pedLight && ledRed.IsOn())
            ledPed.SwitchOn();
        else
            ledPed.SwitchOff();
    }
};

// Trafficlight object at various orientation 0, 90 and 270
TRAFFICLIGHT trafficLight0;
TRAFFICLIGHT trafficLight90;
TRAFFICLIGHT trafficLight270;

THERMISTOR ther;
LDR ldr;

// buttons for special purpose
BUTTON btnEmergency;
BUTTON btnRoadWork;
BUTTON btnSelectSequence;

// buttons for ped crossing
BUTTON btnPedCross0;
BUTTON btnPedCross270;
BUTTON btnPedCross90;

POTENTIOMETER pot;

// main counter keeps track of the sequence
Counter mainCounter;

void setup()
{
    Serial.begin(SERIAL_BAUD);
    
    trafficLight0.SetPins(
        PIN_GREEN_LIGHT_0, 
        PIN_ORANGE_LIGHT_0, 
        PIN_RED_LIGHT_0,
        PIN_FILTERTURN_LIGHT_0,
        PIN_PED_LIGHT_0,
        PIN_PEDCROSS_BUZZER_0
    );
    
    trafficLight90.SetPins(
        PIN_GREEN_LIGHT_90, 
        PIN_ORANGE_LIGHT_90, 
        PIN_RED_LIGHT_90,
        PIN_FILTERTURN_LIGHT_90,
        PIN_PED_LIGHT_90,
        PIN_PEDCROSS_BUZZER_90
    );
    
    trafficLight270.SetPins(
        PIN_GREEN_LIGHT_270, 
        PIN_ORANGE_LIGHT_270, 
        PIN_RED_LIGHT_270,
        PIN_FILTERTURN_LIGHT_270,
        PIN_PED_LIGHT_270,
        PIN_PEDCROSS_BUZZER_270
    );
    
    mainCounter.Reset();
    
    pot.SetPin(PIN_POT);
    pot.ApplySmoothing();
    
    ther.SetPin(PIN_THER);
    ther.SetFixedResistorR2(10000);
    ther.SetVIn(5);
    ther.ApplySmoothing();
    
    ldr.SetPin(PIN_LDR);
    ldr.SetFixedResistorR2(10000);
    ldr.SetVIn(5);
    ldr.ApplySmoothing();
    
    btnEmergency.SetPin(PIN_BTN_EMERGENCY);
    btnRoadWork.SetPin(PIN_BTN_ROADWORK);
    
    btnSelectSequence.SetPin(PIN_BTN_SELECTSEQUENCE);
    btnPedCross0.SetPin(PIN_BTN_PEDCROSS_0);
    btnPedCross270.SetPin(PIN_BTN_PEDCROSS_270);
    btnPedCross90.SetPin(PIN_BTN_PEDCROSS_90);
}

// on emergency mode turn on red light only at all junctions
void OnEmergencyMode() {
    trafficLight0.EmergencyLightOnly();
    trafficLight90.EmergencyLightOnly();
    trafficLight270.EmergencyLightOnly();
}

// on ped crossing mode when time is finished flash ped light at all junction
void FlashForPedCrossFinish() {
    static Counter pedCrossCounter;
    // flash the ped light by checking the counter time
    if(pedCrossCounter.GetCounter() <= 500) {
        trafficLight270.ledPed.SwitchOn();
        trafficLight0.ledPed.SwitchOn();
        trafficLight90.ledPed.SwitchOn();
    }
    else if(pedCrossCounter.GetCounter() <= 1000) {
        trafficLight0.ledPed.SwitchOff();
        trafficLight90.ledPed.SwitchOff();
        trafficLight270.ledPed.SwitchOff();
    }
    else
        pedCrossCounter.Reset();
}

// on road work mode flash orange light only at all junction
void FlashForRoadWork() {
    static Counter roadWorkCounter;
    // flash the orange light by checking the counter time
    if(roadWorkCounter.GetCounter() <= 500) {
        trafficLight0.RoadWorkLightOnly();
        trafficLight90.RoadWorkLightOnly();
        trafficLight270.RoadWorkLightOnly();
    }
    else if(roadWorkCounter.GetCounter() <= 1000) {
        trafficLight0.ledOrange.SwitchOff();
        trafficLight90.ledOrange.SwitchOff();
        trafficLight270.ledOrange.SwitchOff();
    }
    else
        roadWorkCounter.Reset();
}

// Update normal light sequence by checking the main counter
int UpdateNormalSequence(int goLightTime) {     
    int i = 1;
    while (mainCounter.GetCounter() > i * goLightTime)
        i++;
    
    // On Select Sequence Button, cycle through the sequences immediately
    if (btnSelectSequence.IsPressed()) {
        // reset the main counter to start of new sequence
        mainCounter.Reset(i * goLightTime);
        i++;
    }
        
    // if maximum sequences has reached reset the main counter
    if (i == MAX_SEQUENCES + 1) {
        mainCounter.Reset();
        i = 1;
    }
    
    return i;
}

// function to allow for ped crossing on the press of a button
// ped crossing works by pausing the main counter at the end of a sequence 
// and resuming after the ped has crossed.
// returns whether alarm is currently on progression or not
bool AddPedCrossStageOnRequest(int state, int pedCrossTime, int orangeLightTime){
#define NO_PED_CROSS 0
#define PED_CROSS_BTN_PRESSED 1
#define PED_CROSS_IN_PROGRESS 2
    static int alarmState = 0;
    
    // check for button presses at all the junctions
    if(btnPedCross0.IsPressed() || btnPedCross90.IsPressed() || btnPedCross270.IsPressed())
        if (alarmState == NO_PED_CROSS) alarmState = PED_CROSS_BTN_PRESSED;
    
    // update the buzzer asyncronously
    trafficLight0.crossingBuzzer.UpdateAsync();
    trafficLight90.crossingBuzzer.UpdateAsync();
    trafficLight270.crossingBuzzer.UpdateAsync();
    
    static int prevState = 0;
    static Counter pedCrossCounter;
    
    // check if the current sequence has ended
    if (prevState != state && alarmState == PED_CROSS_BTN_PRESSED) {
        // pause the timer
        mainCounter.Pause();
        // reset the crossing counter to 0
        pedCrossCounter.Reset();
        // activate the flag to transition all the green signal to red first
        alarmState = PED_CROSS_IN_PROGRESS; 
        return true;
    }

    if (alarmState == PED_CROSS_IN_PROGRESS) {
        if (pedCrossCounter.GetCounter() <= pedCrossTime) {
                // if there were no green lights from the previous sequence, directly turn on the red light
                if (trafficLight0.ledRed.IsOn() && trafficLight90.ledRed.IsOn() && trafficLight270.ledRed.IsOn())
                {
                    trafficLight270.UpdateWithLightTransition(0, 0, 1, orangeLightTime);
                    trafficLight0.UpdateWithLightTransition(0, 0, 1, orangeLightTime);
                    trafficLight90.UpdateWithLightTransition(0, 0, 1, orangeLightTime);
                    trafficLight0.crossingBuzzer.TriggerAlarm();
                    trafficLight90.crossingBuzzer.TriggerAlarm();
                    trafficLight270.crossingBuzzer.TriggerAlarm();
                }
                // if there were green lights on the previous sequence, transition to red first
                else {
                    trafficLight270.UpdateWithLightTransition(0, 0, 0, orangeLightTime);
                    trafficLight0.UpdateWithLightTransition(0, 0, 0, orangeLightTime);
                    trafficLight90.UpdateWithLightTransition(0, 0, 0, orangeLightTime);
                    trafficLight0.crossingBuzzer.TurnOffAlarm();
                    trafficLight90.crossingBuzzer.TurnOffAlarm();
                    trafficLight270.crossingBuzzer.TurnOffAlarm();
                }
                return true;
        } 
        // if ped cross time has finish blink the ped light as in australian ped crossing
        else if (pedCrossCounter.GetCounter() <= pedCrossTime + orangeLightTime) {
            FlashForPedCrossFinish();
            trafficLight0.crossingBuzzer.TurnOffAlarm();
            trafficLight90.crossingBuzzer.TurnOffAlarm();
            trafficLight270.crossingBuzzer.TurnOffAlarm();
            return true;
        } 
        else {
            alarmState = NO_PED_CROSS;
        }
    }
    prevState = state;
    return false;
}

void loop() {
    delay(50);
    
    boolean hotDay = ther.GetKelvinTempUsingHart() >= THER_CRITICAL_DEG;
    boolean nightMode = ldr.ReadADC() <= LDR_DARK_ADC;
    
    static int goLightTimeCalibed = DEFAULT_GO_LIGHT_TIME;
    static int orangeLightTimeCalibed = DEFAULT_ORANGE_LIGHT_TIME;
    
    // if both emergency button and road work are pressed activate the calibration
    if (btnEmergency.IsPressed() && btnRoadWork.IsPressed()) {
        goLightTimeCalibed = map(pot.ReadADC(), 0, 1023, DEFAULT_GO_LIGHT_TIME/2, DEFAULT_GO_LIGHT_TIME * 2);
        orangeLightTimeCalibed = map(pot.ReadADC(), 0, 1023, DEFAULT_ORANGE_LIGHT_TIME/2, DEFAULT_ORANGE_LIGHT_TIME * 2);
    } 
    else {
        boolean emergencyMode = btnEmergency.ReadSwitch();
        boolean roadWorkMode = btnRoadWork.ReadSwitch();
        
        if (emergencyMode) {
            OnEmergencyMode();
        }
        
        if (roadWorkMode) {
            FlashForRoadWork();
        }
        
        // pause the main counter when one of the modes are activated
        if(roadWorkMode || emergencyMode) {
            mainCounter.Pause();
            return;
        }
    }
    
    int goLightTime = goLightTimeCalibed;
    int orangeLightTime = orangeLightTimeCalibed;
    double multiplier =  1.0;
    
    if (nightMode) { 
        multiplier += 0.4;
    }
    
    if(hotDay) {
        multiplier += 0.3;
    }
    
    // extended the go or green signal time and the orange light time
    int goLightTime = goLightTimeCalibed * multiplier;
    int orangeLightTime = orangeLightTimeCalibed * multiplier;
    
    // get the current state of the sequence
    int state = UpdateNormalSequence(goLightTime);
    
    // print out the state of the system for debugging
    Serial.print("Temperature: ");
    Serial.print(ther.GetKelvinTempUsingHart());
    Serial.print(" LDR: ");
    Serial.print(ldr.ReadADC());
    Serial.print(" PotADC: ");
    Serial.print(pot.ReadADC());
    Serial.print(" Cross: ");
    Serial.print(btnPedCross0.IsPressed() || btnPedCross90.IsPressed() || btnPedCross270.IsPressed());
        
    Serial.print(" main Counter: ");
    Serial.print(mainCounter.GetCounter());
    Serial.print(" goTime: ");
    Serial.print(goLightTime);
    Serial.print(" Orange: ");
    Serial.print(orangeLightTime);
    Serial.print(" state: ");   
    Serial.println(state);
        
    // if alarm is in progress, exit from normal sequence
    if (AddPedCrossStageOnRequest(state, goLightTime, orangeLightTime))
        return;
    
    // unpause the main counter if it was paused
    mainCounter.UnPause();
    
    // update the leds on the traffic light depending on the values in the sequence array
    trafficLight270.UpdateWithLightTransition(sequence[state - 1][0], sequence[state - 1][1], sequence[state - 1][2], orangeLightTime);
    trafficLight0.UpdateWithLightTransition(sequence[state - 1][3], sequence[state - 1][4], sequence[state - 1][5], orangeLightTime);
    trafficLight90.UpdateWithLightTransition(sequence[state - 1][6], sequence[state - 1][7], sequence[state - 1][8], orangeLightTime);
}
