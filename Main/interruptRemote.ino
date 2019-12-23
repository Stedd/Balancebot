//Variables
const byte  NO_CHANNELS       = 2;
const byte  CHANNEL_PINS[]    = {12, 14};
const int   CH_MIN            = 980;
const int   CH_MAX            = 1997;
const int   INIT_VALUE        = CH_MAX/2;
float       remoteRaw[NO_CHANNELS];
float       remoteCMD[NO_CHANNELS];

volatile unsigned int interruptTime_ch1, interruptTimeLast_ch1;
volatile unsigned int interruptTime_ch2, interruptTimeLast_ch2;
volatile unsigned int pwm_time_ch1, pwm_time_ch2;


  void ch1_interrupt() {
    interruptTime_ch1 = micros();
    if (interruptTime_ch1 >interruptTimeLast_ch1 && (interruptTime_ch1 - interruptTimeLast_ch1)< 2100){
        pwm_time_ch1 = interruptTime_ch1 - interruptTimeLast_ch1;
    }

    interruptTimeLast_ch1 = interruptTime_ch1;
  }

  void ch2_interrupt() {
    interruptTime_ch2 = micros();
    if (interruptTime_ch2 > interruptTimeLast_ch2 && (interruptTime_ch2 - interruptTimeLast_ch2)< 2100 ){
        pwm_time_ch2 = interruptTime_ch2 - interruptTimeLast_ch2;
    }
    interruptTimeLast_ch2 = interruptTime_ch2;
  }


void initRemote(){

  //Ch1
  pinMode(CHANNEL_PINS[0], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CHANNEL_PINS[0]), ch1_interrupt, CHANGE);

  //Ch2
  pinMode(CHANNEL_PINS[1], INPUT);
  attachInterrupt(digitalPinToInterrupt(CHANNEL_PINS[1]), ch2_interrupt, CHANGE);

}


void readRemote(){

  remoteCMD[0] = floatMap(pwm_time_ch1, 992.0, 2007.0, -2.5, 2.5); //turn rate
  remoteCMD[1] = floatMap(pwm_time_ch2, 982.0, 1997.0, -0.25, 0.25); //speed

  // Remote control
  // Serial.print("ch1:");
  // Serial.print(pwm_time_ch1);
  // Serial.print(" ");
  // Serial.print("ch2:");
  // Serial.print(pwm_time_ch2);
  // Serial.print(" ");
  // Serial.print("ch1_mapped:");
  // Serial.print(remoteCMD[0]);
  // Serial.print(" ");
  // Serial.print("ch2_mapped:");
  // Serial.println(remoteCMD[1]);

}


float floatMap(int in, float inMin, float inMax, float outMin, float outMax){
  return (in - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
