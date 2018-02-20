// From https://www.rcgroups.com/forums/showthread.php?2037080-DIY-Arduino-joystick-to-PPM

#define NB_WAY 8 // number of ways
#define LOW_LENGTH 400 // How long last (in µs) a low between 2 pulses
#define MIN_PPM_PULSE 600 // minimum pulse length in µs
#define PPM_PULSE_LENGTH 1100 // how much more µs will last the max pulse length
#define MID_PPM_PULSE 1150 // Mid point for the channel
#define PACKET_LENGTH 16000 // How long (µs) last a full trame

// trame length is fixed ! Every trame will make PACKET_LENGTH µs !
// MUST NO BE MORE THAN 32ms !!! (timer's prescaler constraint)

#define PPM_OUTPUT 8 // OUTPUT PIN

#define INVERT_X_AXIS_1 false
#define INVERT_Y_AXIS_1 false
#define INVERT_X_AXIS_2 true
#define INVERT_Y_AXIS_2 true

int way_value[NB_WAY];
int way_pin[NB_WAY];
//int way_min[NB_WAY];
//int way_max[NB_WAY];

int i = 0;
int p = 0; // temp var for duty cycle calculation
int last_i_timer = 0; // last way's value sent through PPM signal
unsigned long int trame_elapsed_time = 0;
bool output_state = LOW;

// Settings for joystick input from chair
// First joystick
const int yAxis1Pin = A0;
const int xAxis1Pin = A1;
const int vRef1Pin  = A2;
int vRef1Value = 0;

// Second joystick
const int yAxis2Pin = A4;
const int xAxis2Pin = A3;
const int vRef2Pin  = A5;
int vRef2Value = 0;

void setup() {
  // ppm output :
  pinMode(PPM_OUTPUT, OUTPUT);
  digitalWrite(PPM_OUTPUT, output_state);
  
  // inits arrays
  for(i=0;i<NB_WAY;i++)
  {
    way_pin[i] = 14 + i;
    pinMode(way_pin[i], INPUT);
    way_value[i] = analogRead(way_pin[i]);
    //way_min[i] = way_value[i];
    //way_max[i] = way_value[i];
  }

  // init timer
  cli();          // deactivate interruptions
  TCCR1A = 0x00;  // set timer1 registers to 0
  TCCR1B = 0x00;     
  TIMSK1 = 0x00;
    
  OCR1A = 65535;// set to the max
    // CTC mode:
  TCCR1B |= (1 << WGM12);
    // prescaler to 8, that allow (@16mhz) 32.8ms trame
  TCCR1B |= (0 << CS10);
  TCCR1B |= (1 << CS11);
  TCCR1B |= (0 << CS12);
    // timer activation
  TIMSK1 |= (1 << OCIE1A);
  sei();
}

ISR(TIMER1_COMPA_vect)
{
  TIMSK1 &= (0 << OCIE1A);
  if(output_state)
  { // END OF A HIGH, we have to wait LOW_LENGTH ms before next pulse 
    output_state = LOW;
    digitalWrite(PPM_OUTPUT, output_state);
    OCR1A = 2 * LOW_LENGTH; // set when next timer interruption will occur
    TIMSK1 |= (1 << OCIE1A);  // restart timer
    trame_elapsed_time += LOW_LENGTH;
  }
  else
  { // END of a LOW_LENGTH, new pulse !
    output_state = HIGH;
    digitalWrite(PPM_OUTPUT, output_state);
    if(last_i_timer >= NB_WAY) // last way, so wait until next packet
    {
      OCR1A = (2 * PACKET_LENGTH) - (trame_elapsed_time * 2);// set when next timer interruption will occur
      TIMSK1 |= (1 << OCIE1A); // restart timer
      last_i_timer = 0;
      trame_elapsed_time = 0;
    }
    else
    {
      OCR1A = 2 * way_value[last_i_timer];// set when next timer interruption will occur
      TIMSK1 |= (1 << OCIE1A); // restart timer
      last_i_timer ++;
      trame_elapsed_time += way_value[NB_WAY];
      trame_elapsed_time = 0;
    }
  }  
}


void loop() {
  /* Process joystick 1 */
  vRef1Value = analogRead(vRef1Pin);

  int yAxis1Delta = 0;
  int reading = analogRead(yAxis1Pin);
  if(INVERT_Y_AXIS_1 == true)
  {
    yAxis1Delta = 4.5 * (reading - vRef1Value);
  } else {
    yAxis1Delta = 4.5 * (vRef1Value - reading);
  }
  int yAxis1Value = MID_PPM_PULSE + yAxis1Delta;

  int xAxis1Delta = 0;
  reading = analogRead(xAxis1Pin);
  if(INVERT_X_AXIS_1 == true)
  {
    xAxis1Delta = 4.5 * (reading - vRef1Value);
  } else {
    xAxis1Delta = 4.5 * (vRef1Value - reading);
  }
  int xAxis1Value = MID_PPM_PULSE + xAxis1Delta;

  /* Process joystick 2 */
  vRef2Value = analogRead(vRef2Pin);

  int yAxis2Delta = 0;
  reading = analogRead(yAxis2Pin);
  if(INVERT_Y_AXIS_2 == true)
  {
    yAxis2Delta = 4.5 * (reading - vRef2Value);
  } else {
    yAxis2Delta = 4.5 * (vRef2Value - reading);
  }
  int yAxis2Value = MID_PPM_PULSE + yAxis2Delta;

  int xAxis2Delta = 0;
  reading = analogRead(xAxis2Pin);
  if(INVERT_X_AXIS_2 == true)
  {
    xAxis2Delta = 4.5 * (reading - vRef2Value);
  } else {
    xAxis2Delta = 4.5 * (vRef2Value - reading);
  }
  int xAxis2Value = MID_PPM_PULSE + xAxis2Delta;


  /* Set channel values */
  //way_value[0] = MIN_PPM_PULSE + analogRead(A0);
  way_value[0] = xAxis1Value;
  way_value[1] = xAxis2Value;
  way_value[2] = yAxis1Value;
  way_value[3] = yAxis2Value;
  way_value[4] = MID_PPM_PULSE;
  way_value[5] = MID_PPM_PULSE;
  way_value[6] = MID_PPM_PULSE;
  way_value[7] = MID_PPM_PULSE;

  /*
  for(i=0;i<NB_WAY;i++)
  {
     // Read current value of way i :
     p = analogRead(way_pin[i]);
     
     // auto calibration...
     if(p > way_max[i]) way_max[i] = p;
     if(p < way_min[i]) way_min[i] = p;
     
     // Arduino map function sucks
      way_value[i] = MIN_PPM_PULSE + PPM_PULSE_LENGTH * (float)((float)(p - way_min[i]) / (float)(way_max[i] - way_min[i]));
  } */
}
