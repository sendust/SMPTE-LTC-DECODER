// SMPTE timecode reader interface by sendust
/*  Use with LTC pulse dectector code (using ISR)
 *  2022/2/15 - First release
 *  2022/2/16 - Introduce heartbeat class
 *  2022/2/17 - LED show ; disabled for performance issue.
 *  2022/2/18 - Enable LED show (improve performance, less use of digital write)
 * 
 * 
 * 
 * 
 */

#define BUFFER_MAX 20

#include <SoftwareSerial.h>
SoftwareSerial softserial(10, 9); // RX, TX



struct SER
{
  bool valid_data, clock_current, clock_old;
  uint8_t buffer[BUFFER_MAX], index, data;
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
    m_mask_result = m_count & (m_mask << 10);
    m_pin_state = m_mask_result && 1;
    if (m_pin_state != m_pin_state_old)
    {
      digitalWrite(m_pinLED, m_pin_state);
      m_pin_state_old = m_pin_state;
    }
  }

 
};

class cLEDSHOW
{
  private:
    unsigned long m_count;
    int m_pinLED1, m_pinLED2, m_pinLED3, m_pinLED4;
    int  m_mask, m_mask_old;
    const uint8_t  m_sequential[8][4] = {{0, 1, 1, 1},
                                         {1, 0, 1, 1},
                                         {1, 1, 0, 1},
                                         {1, 1, 1, 0},
                                         {0, 1, 1, 1},
                                         {1, 0, 1, 1},
                                         {1, 1, 0, 1},
                                         {1, 1, 1, 0}
                                         };

  public:
    cLEDSHOW(int pinLED1, int pinLED2, int pinLED3, int pinLED4)     // define heartbeat LED pin
    {
      pinMode(pinLED1, OUTPUT);
      pinMode(pinLED2, OUTPUT);
      pinMode(pinLED3, OUTPUT);
      pinMode(pinLED4, OUTPUT);
      digitalWrite(pinLED1, HIGH);
      digitalWrite(pinLED2, HIGH);
      digitalWrite(pinLED3, HIGH);  
      digitalWrite(pinLED4, HIGH);               
      m_pinLED1 = pinLED1;
      m_pinLED2 = pinLED2;
      m_pinLED3 = pinLED3;
      m_pinLED4 = pinLED4;      
      m_count = 0;
      m_mask = 0;
      m_mask_old = 1;
    }

  run()
  {
    m_count++;
    m_mask = (m_count >> 13) & B00000111;
    if (m_mask == m_mask_old)
    {
      return;
    }
    digitalWrite(m_pinLED1, m_sequential[m_mask][0]);
    digitalWrite(m_pinLED2, m_sequential[m_mask][1]);
    digitalWrite(m_pinLED3, m_sequential[m_mask][2]);
    digitalWrite(m_pinLED4, m_sequential[m_mask][3]);
    m_mask_old = m_mask;
  }
  
};





SER serial_data;


char tc[12];                          // array for send serial data

//cHeartBeat Heartbeat(2);             // start heartbeat with LED pin number
led_fback debug_ledA(6);             // LED for debugging, Finish code arrived
led_fback debug_ledB(2);             //                    Invalid data length
led_fback debug_ledC(3);             //                    Buffer overflow detected
led_fback debug_ledD(4);


cLEDSHOW led_parade(2, 3, 4, 5);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  softserial.begin(115200);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);

  serial_data.index = 0;
  serial_data.valid_data = 0;

  debug_ledB.set(1);
  debug_ledC.set(1);
  debug_ledD.set(1);

  tc[2] = ':';          // Move from loop to here.  improve performance
  tc[5] = ':';
  tc[8] = '.'; 
  tc[11] = '\r';        // Assign serial print finish code
  
  Serial.println(" -- LTC Reader started  -- ");
}

void loop() {
  // put your main code here, to run repeatedly:
  // Heartbeat.run();
  led_parade.run();         // Disabled for performance issue  2022/2/17
  serial_data.clock_current = digitalRead(A5);            // Probe clock
  
  if ((serial_data.clock_current == HIGH) & (serial_data.clock_old == LOW))   // clock detected !!
  {

    serial_data.data = (digitalRead(A1) << 3) | (digitalRead(A2) << 2) | (digitalRead(A3)) << 1  | digitalRead(A4);   // convert 4bit binary to decimal
    serial_data.buffer[serial_data.index] =  serial_data.data;
    serial_data.index += 1;         // Fill up buffer & increase buffer index
    if (serial_data.data == 10)     // Finish signal arrived
    {
      debug_ledA.toggle();
      tc[0] = serial_data.buffer[0] + '0';
      tc[1] = serial_data.buffer[1] + '0';

      tc[3] = serial_data.buffer[2] + '0';
      tc[4] = serial_data.buffer[3] + '0';

      tc[6] = serial_data.buffer[4] + '0';
      tc[7] = serial_data.buffer[5] + '0';

      tc[9] = serial_data.buffer[6] + '0';
      tc[10] = serial_data.buffer[7] + '0';
      
      softserial.print(tc);
      Serial.print(serial_data.index);
      Serial.print(" ");
      Serial.print(tc);

      if (serial_data.index != 9)   // Finish mark character should arrive at 8th position
      {
        debug_ledB.toggle();        // Invalid length arrived
        Serial.println("Invalid data length");
      }
      serial_data.index = 0;
    }

    if (serial_data.index >= BUFFER_MAX - 1)
    {
      serial_data.index = 0;
      Serial.println("Buffer overflow");
      debug_ledC.toggle();
    }
  }
  
  serial_data.clock_old = serial_data.clock_current;
}
