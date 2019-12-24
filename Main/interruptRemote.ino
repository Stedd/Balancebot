//Variables
const     byte          NO_CHANNELS       = 2;
const     byte          CHANNEL_PINS[]    = {12, 14};
volatile  unsigned int  interruptTime_ch1, interruptTimeLast_ch1;
volatile  unsigned int  interruptTime_ch2, interruptTimeLast_ch2;
volatile  unsigned int  pwm_time_ch1, pwm_time_ch2;


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
