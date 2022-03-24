// SMPTE Timecode Reader
// Uses INT0 (Pin 2)
/*
  PAL = 25FPS = 40ms per frame
  40ms / 80bits = 500us per bit
  250us per half bit
  125us per quarter bit
  0 bit max = 625us
  1 bit max = 375us (also 0 bit min)
  1 bit min = 125us

  NTSC = 30FPS (29.97) = 33.333ms per frame
  33.3ms / 80bits = 417us per bit
  208us per half bit
  104us per quarter bit
  0 bit max = 521us
  1 bit max = 312us (also 0 bit min)
  1 bit min = 104us


  Modified by sendust   2022/2/10
  do not put serial.println in loop routine !!!
    - Only led feedback is available (It's timing critical mission)
    - 100us is too short for serial communication

  2022/2/14 4 bit decimal to binary (A1 ~ A4 : data, A5 : clock)
            - Need two arduino (one for pc interface, one for pulse width detection)
            - connect A1 ~ A5 between two arduino
  2022/2/15 add drop frame, color frame flag (show on LCD)
  2022/2/16 New led feedback class
  2022/2/18 Improve heartbeat class, Add A5 pulse LOW state delay
  2022/3/25 Reorder userbit data
  
*/

//Sample using LiquidCrystal library
#include <LiquidCrystal.h>


// select the pins used on the LCD panel
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);  // LiquidCrystal(rs, enable, d4, d5, d6, d7)



// #define PAL         // Comment out or delete this line for NTSC signal timings

#ifdef PAL
  #define uMax0 625 // Any time greater than this is to big
  #define uMax1 375 // Midpoint timing between a 1 and 0 bit length
  #define uMin1 125 // Any time less than this is to short
#else
  #define uMax0 521 // Any time greater than this is to big
  #define uMax1 312 // Midpoint timing between a 1 and 0 bit length
  #define uMin1 104 // Any time less than this is to short
#endif
  

const word sync = 0xBFFC; // Sync word to expect when running tape forward

enum flagBits {
  tcValid,        // TC copied to xtc is valid (Used by main loop to determing if a timecode has arrived)
  tcFrameError,   // ISR edge out of timing bounds (only gets reset after next valid TC read)
  tcOverrun,      // TC was not read from xtc by main loop before next value was ready (so main loop can tell if timecodes have been lost)
  tcForceUpdate,  // Valid TC will always be copied to buffer even if last value not read (if set by main code then TC will always be copied)
  tcHalfOne       // ISR is reading a 1 bit so ignore next edge (Internal to ISR)
};


class led_fback       // class for LED feedback, debugging purpose
{
  public:
    led_fback(uint8_t pin_no)
    {
      pin_control = pin_no;
      pin_state = 0;
      pinMode(pin_control, OUTPUT);
      digitalWrite(pin_control, pin_state);
    }
  
    void toggle()
    {
      pin_state = !pin_state;
      digitalWrite(pin_control, pin_state);
    }

    void set(bool state)
    {
      pin_state = state;
      digitalWrite(pin_control, pin_state);
    }
  

  private:
    uint8_t pin_control;
    bool pin_state;
  
};


class cHeartBeat
{
  private:
  
  unsigned long m_count;
  int m_pinLED;
  int m_pin_state, m_pin_state_old;
  unsigned long m_mask_result, m_mask_result_old;
  const unsigned long m_mask = B01111110;
  
  public:
  cHeartBeat(const int pinLED)            // define heartbeat LED pin
  {
    pinMode(pinLED, OUTPUT);
    digitalWrite(pinLED, LOW);
    m_pinLED = pinLED;
    m_count = 0;
    m_pin_state = 1;
    m_pin_state_old = !m_pin_state;
    m_mask_result_old = 0;
  }

  void run()
  {
    // Serial.println(m_mask);
    m_count++;
    m_mask_result = m_count & (m_mask << 11);
    m_pin_state = m_mask_result && 1;
    if (m_pin_state != m_pin_state_old)
    {
      digitalWrite(m_pinLED, m_pin_state);
      m_pin_state_old = m_pin_state;
    }
  }

 
};

struct FLAGS
{
  uint8_t drop_frame, color_frame;
  char drop[2], color[2];
};



volatile uint8_t tc[10] = {0};                            // ISR Buffer to store incoming bits
volatile uint8_t xtc[8] = {0};                            // Buffer to store valid TC data - sync bytes
volatile uint8_t tcFlags = 0;                             // Various flags used by ISR and main code
volatile uint32_t uSeconds;                               // ISR store of last edge change time

char timeCode[13];                                        // For example code another buffer to write decoded timecode
char userBits[12];                                        // For example code another buffer to write decoded user bits
char tc_send[9];                                          // For 4 bit decimal communication (HHMMSSff + FINISH CODE)

bool DCBconversion[][4] =  {{0,0,0,0},     // Number 0
                            {0,0,0,1},
                            {0,0,1,0},
                            {0,0,1,1},
                            {0,1,0,0},
                            {0,1,0,1},
                            {0,1,1,0},
                            {0,1,1,1},
                            {1,0,0,0},
                            {1,0,0,1},   // Number 9
                            {1,0,1,0}};  // Number 10 (Finish signal)


led_fback debug_ledA(12);             // LED for debugging
led_fback debug_ledB(11);
cHeartBeat Heartbeat(13);                   // start heartbeat with LED pin number


FLAGS signal_flag;

void setup(){
  lcd.begin(16,2);

  Serial.begin(115200);
  pinMode(2,INPUT_PULLUP);  // Setup interrupt pin
  pinMode(10,OUTPUT);
  digitalWrite(10, HIGH);   // LCD Backlight on

  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);

  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  digitalWrite(A4, LOW);
  digitalWrite(A5, LOW);
  
  Serial.println(uMax0);
  Serial.println(uMax1);
  Serial.println(uMin1);
  Serial.println(F("Waiting For TC"));

  tc_send[8] = 10;                                 // send Finish code
  
  lcd.setCursor(0,0);
  lcd.print("TC Waiting SIG");

  lcd.setCursor(0,1);
  lcd.print("UB sendust LTC");
  Serial.flush();

  signal_flag.drop[0] = ' ';
  signal_flag.drop[1] = 'd';
  signal_flag.color[0] = ' ';
  signal_flag.color[1] = 'c';

  debug_ledB.set(1);        // Turn off LED
  attachInterrupt(digitalPinToInterrupt(2), int0ISR, CHANGE);

}


void loop(){
  while(!bitRead(tcFlags, tcValid)){ 
      Heartbeat.run();
      };                // Wait for valid timecode

  debug_ledA.toggle();
  
  timeCode[0] = (xtc[0] & 0x03) + '0';                    // 10's of hours
  timeCode[1] = (xtc[1] & 0x0F) + '0';                    // hours
  timeCode[2] = ':';                
  timeCode[3] = (xtc[2] & 0x07) + '0';                    // 10's of minutes
  timeCode[4] = (xtc[3] & 0x0F) + '0';                    // minutes
  timeCode[5] =  ':';               
  timeCode[6] = (xtc[4] & 0x07) + '0';                    // 10's of seconds
  timeCode[7] = (xtc[5] & 0x0F) + '0';                    // seconds
  timeCode[8] =  '.';               
  timeCode[9] = (xtc[6] & 0x03) + '0';                    // 10's of frames
  timeCode[10] = (xtc[7] & 0x0F) + '0';                   // frames


  signal_flag.drop_frame = (xtc[6] & 0x04) >> 2;          // Drop frame flag.
  signal_flag.color_frame =(xtc[6] & 0x08) >> 3;          // Color frame flag.

  timeCode[11] = signal_flag.color[signal_flag.color_frame];
  timeCode[12] = signal_flag.drop[signal_flag.drop_frame];
  
  userBits[0] = ((xtc[0] & 0xF0) >> 4) + '0';             // user bits 1 
  userBits[1] = ((xtc[1] & 0xF0) >> 4) + '0';             // user bits 2  
  userBits[2] = '-';            
  userBits[3] = ((xtc[2] & 0xF0) >> 4) + '0';             // user bits 3
  userBits[4] = ((xtc[3] & 0xF0) >> 4) + '0';             // user bits 4
  userBits[5] = '-';            
  userBits[6] = ((xtc[4] & 0xF0) >> 4) + '0';             // user bits 5
  userBits[7] = ((xtc[5] & 0xF0) >> 4) + '0';             // user bits 6
  userBits[8] = '-';            
  userBits[9] = ((xtc[6] & 0xF0) >> 4) + '0';             // user bits 7
  userBits[10] = ((xtc[7] & 0xF0) >> 4) + '0';            // user bits 8
  
 
  tc_send[0] = (xtc[0] & 0x03);                    // 10's of hours
  tc_send[1] = (xtc[1] & 0x0F);                    // hours
  tc_send[2] = (xtc[2] & 0x07);                    // 10's of minutes
  tc_send[3] = (xtc[3] & 0x0F);                    // minutes
  tc_send[4] = (xtc[4] & 0x07);                    // 10's of seconds
  tc_send[5] = (xtc[5] & 0x0F);                    // seconds
  tc_send[6] = (xtc[6] & 0x03);                    // 10's of frames
  tc_send[7] = (xtc[7] & 0x0F);                    // frames



  for (int i=0; i < 9; i++)       // prepare 4 bit Decimal code
  {
    digitalWrite(A1, DCBconversion[tc_send[i]][0]);
    digitalWrite(A2, DCBconversion[tc_send[i]][1]);
    digitalWrite(A3, DCBconversion[tc_send[i]][2]);
    digitalWrite(A4, DCBconversion[tc_send[i]][3]);
    digitalWrite(A5, HIGH);        // Raise clock high (client begin to read)
    delayMicroseconds(50);          // maintain clock high some period
    digitalWrite(A5, LOW);         // reset clock
    delayMicroseconds(50);          // maintain clock high some period
  }

  lcd.setCursor(3,0);
  lcd.print(timeCode);                                    // Print time code

  lcd.setCursor(3,1);                                     // Print Userbit
  lcd.print(userBits);

  bitClear(tcFlags, tcValid);                             // Finished with TC so signal to ISR it can overwrite it with next TC
}



void int0ISR(){
  uint32_t edgeTimeDiff = micros() - uSeconds;            // Get time difference between this and last edge
  uSeconds = micros();                                    // Store time of this edge

  if ((edgeTimeDiff < uMin1) or (edgeTimeDiff > uMax0)) { // Drop out now if edge time not withing bounds
    bitSet(tcFlags, tcFrameError);
    return;
  }
  
  if (edgeTimeDiff > uMax1)                               // A zero bit arrived
  {

    if (bitRead(tcFlags, tcHalfOne) == 1){                // But we are expecting a 1 edge
      bitClear(tcFlags, tcHalfOne);
      clearBuffer(tc, sizeof(tc));
      return;
    }
    // 0 bit
    shiftRight(tc, sizeof(tc));                           // Rotate buffer right
//    Shift replaces top bit with zero so nothing else to do
    bitClear(tc[0], 7);                                   // Reset the 1 bit in the buffer
  }
  else                                                    // Not zero so must be a 1 bit
  { // 1 bit
    if (bitRead(tcFlags, tcHalfOne) == 0){                // First edge of a 1 bit
      bitSet(tcFlags, tcHalfOne);                         // Flag we have the first half
      return;
    }
    // Second edge of a 1 bit
    bitClear(tcFlags, tcHalfOne);                         // Clear half 1 flag
    shiftRight(tc, sizeof(tc));                           // Rotate buffer right
    bitSet(tc[0], 7);                                     // Set the 1 bit in the buffer

  }
  
  // Congratulations, we have managed to read a valid 0 or 1 bit into buffer
  if (word(tc[0], tc[1]) == sync){                        // Last 2 bytes read = sync?
    debug_ledB.toggle();                                  // Feedback LED for every sync
    bitClear(tcFlags, tcFrameError);                      // Clear framing error
    bitClear(tcFlags, tcOverrun);                         // Clear overrun error
    if (bitRead(tcFlags, tcForceUpdate) == 1){
      bitClear(tcFlags, tcValid);                         // Signal last TC read
    }
    if (bitRead(tcFlags, tcValid) == 1){                  // Last TC not read
      bitSet(tcFlags, tcOverrun);                         // Flag overrun error
      return;                                             // Do nothing else
    }


    for (uint8_t x = 0; x < sizeof(xtc); x++){            // Copy buffer without sync word
      xtc[x] = tc[x + 2];
    }
    bitSet(tcFlags, tcValid);                             // Signal valid TC
  }
  return;
}


void clearBuffer(uint8_t theArray[], uint8_t theArraySize){
  for (uint8_t x = 0; x < theArraySize - 1; x++){
    theArray[x] = 0;
  }
}

void shiftRight(uint8_t theArray[], uint8_t theArraySize){
  uint8_t x;
  for (x = theArraySize; x > 0; x--){
    uint8_t xBit = bitRead(theArray[x - 1], 0);
    theArray[x] = theArray[x] >> 1;
    theArray[x] = theArray[x] | (xBit << 7);
  }
  theArray[x] = theArray[x] >> 1;
}



// void dec2bin64(uint64_t myNum, uint8_t numberOfBits) {
  // if (numberOfBits <= 64){
    // for (uint8_t i = 0; i < numberOfBits; i++) {
      // if (bitRead(myNum,numberOfBits - i - 1)) { 
        // Serial.print("1");
      // }
      // else {
        // Serial.print("0");
      // }
    // }
  // }
// }

// void dec2bin8(uint8_t myNum, uint8_t numberOfBits) {
  // if (numberOfBits <= 8){
    // for (uint8_t i = 0; i < numberOfBits; i++) {
      // if (bitRead(myNum,numberOfBits - i - 1)) { 
        // Serial.print("1");
      // }
      // else {
        // Serial.print("0");
      // }
    // }
  // }
// }

// void tcShiftLeft(){
// uint8_t x;
// for (x = 0; x < (sizeof(tc) - 1); x++){
// tc[x] = tc[x] << 1;
// uint8_t xBit = bitRead(tc[x + 1], 7);
// tc[x] = tc[x] | xBit;
// }
// tc[x] = tc[x] << 1;
// }

// void tcShiftRight(){
// uint8_t x;
// for (x = sizeof(tc); x > 0; x--){
// uint8_t xBit = bitRead(tc[x - 1], 0);
// tc[ x] = tc[x] >> 1;
// tc[x] = tc[x] | (xBit << 7);
// }
// tc[x] = tc[x] >> 1;
// }



/*



class led_feedback        // Depricated. do not use !!!!!   class for LED feedback, debugging purpose
{
    public:
    
    led_feedback(uint8_t *p)    // accept led control pin set (array)
    {
        ary_size = sizeof(p);
        hilo_value = (uint8_t*) malloc(ary_size);
        hilo_pin = (uint8_t*) malloc(ary_size);

        for(int i; i < ary_size; i++)
        {
            hilo_value[i] = 0;
            hilo_pin[i] = p[i];
            pinMode(hilo_pin[i], OUTPUT);    // Setup LED control pin as output
            digitalWrite(hilo_pin[i], hilo_value[i]);    // Initialize with 0
        }
    }
    
    void toggle(uint8_t i)      // optional improvement : check if (i < ary_size) 
    {
        hilo_value[i] = !hilo_value[i];
        digitalWrite(hilo_pin[i], hilo_value[i]);
    }
    
    void on(uint8_t i)
    {
        hilo_value[i] = 1;
        digitalWrite(hilo_pin[i], hilo_value[i]);
    }
    
    void off(uint8_t i)
    {
        hilo_value[i] = 0;
        digitalWrite(hilo_pin[i], hilo_value[i]);
    }
    
    private:

    uint8_t *hilo_pin, *hilo_value, ary_size;

};

*/
