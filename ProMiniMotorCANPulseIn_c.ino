/*PWM lesen zu CAN umsetzer
  Liest PWM Werte (Lüfteransteuerung) und gibt diese auf CAN Bus aus.

   Changelog:
   2018-06-18: Version _a
  Basis ist MotorPoti ProMini-Modul
   SCK:13,
   MISO:12,
   MOSI:11,
   CS:10,
   INT:9

  2018-06-19:
  Pulse nicht mit PulseIn() erfassen (blockiert alles, bzw. funktioniert nicht)
  Zeit messen und Pin mittels digital read pollen.
  Option: Interrupts und nur Pin2 und 3 (INT 0 und 1) nutzen
*/

//#include <Wire.h>
#include <mcp_can.h>
#include <SPI.h>

// pins external hardware

/*****TIMER FLAGS*****/
unsigned long prevMillis = 0;
unsigned long prevMillisLed1 = 0;
int BlinkTypeLed1 = 0; //0=Off;
int Led1State = LOW;
unsigned long prevMillisLedLinks = 0;
int BlinkTypeLedLinks = 0; //0=Off;
int LedLinksState = LOW;
unsigned long prevMillisRegler = 0;
const long cycleSchnellBlinken = 50;  //langsames Blinken = Empfang von Sollwerten
const long cycleLangsamBlinken = 1000;    //schnelles Blinken = Verstellen = e>0;
const long cycleRegler = 10;    //10ms Regler
const long cycleCan = 20;   //Tx cycle
unsigned long prevMillisCan = 0;
const unsigned int CanRxTimeout_ms = 10000; //nach 10s gehen die Rx Werte auf SNA
unsigned long int lastCANMessage_ms = 0;

/*****CAN MCP2515 Values*****/
#define CAN0_INT 9   // Setting pin 9 for /INT input                           
MCP_CAN CAN0(10);     // Set CS to pin 10
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string
unsigned char rxSelFrame[8];
long unsigned int selId = 345;
long unsigned int selSendIf = 0x18FF8000; //Bsp.
long unsigned int RotaryFrameId = 0x18FEF400; //Zurücksetzen der Drehknopfspeicherwerte
long unsigned int LedFrameId = 0x18FEF401; //LED links und rechts
long unsigned int MotorpotiFrameId = 0x18FEF402; //Motorpoti-Soll-Position links und rechts
long unsigned int PulseInFrameId = 0x18FEF403; //Sollwerte vom Rechner (Vector)
long unsigned int PulseInFrameID_Tx = 0x18FEF397; //Istwerte zum Rechner (Vector)

byte data[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};

// Interrupt.0 befindet sich beim digitalen Pin 2 (fest)
// Interrupt.1 befindet sich beim digitalen Pin 3 (fest)

/*  Wir erstellen die Variable mit dem Schlüsselwort volatile,
  um sicherzustellen, dass die Variable vom
  normalen Programmfluss und vom Interrupt verfügbar ist.
*/
volatile long unsigned int Pulsdauer2 = 0;
volatile long unsigned int Pulsdauer3 = 0;

volatile long unsigned int Periodendauer2 = 0;
volatile long unsigned int Periodendauer3 = 0;

float Tastverhaeltnis2[] = {0, 0, 0, 0};
float Tastverhaeltnis3[] = {0, 0, 0, 0};

volatile long unsigned int PulsStart2 = 0;
volatile long unsigned int PulsStart3 = 0;

unsigned int FO1_out_CAN = 255; //SNA
unsigned int FO2_out_CAN = 255; //SNA

const int LED1_pin = 6;

void setup() {
  // initialize serial:
  Serial.begin(9600);

  //Pins
  //Puls(PWM)-Mmessung 1
  pinMode(2, INPUT);
  pinMode(2, INPUT_PULLUP);
  //Puls(PWM)-Messung 2
  pinMode(3, INPUT);
  pinMode(3, INPUT_PULLUP);
  //Digitaler Ausgang (über Optokoppler, dominant HIGH, 24V, ohne Vorwiderstand!!! --> muss in CCU sein)
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  //LEDs
  pinMode(LED1_pin, OUTPUT);

  /*****CAN MCP2515*****/
  //Initialize MCP2515 running at 8MHz with a baudrate of 500kb/s and the masks and filters disabled.
  pinMode(CAN0_INT, INPUT);  // Configuring pin for /INT input
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515...");
  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted

  attachInterrupt(digitalPinToInterrupt(2), Zeitmessen2Change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), Zeitmessen3Change, CHANGE);

}

void Zeitmessen2Change() {
  if (digitalRead(2) == LOW) {
    //durch Optokoppler invertiertes verhältnis
    Periodendauer2 = micros() - PulsStart2;
    PulsStart2 = micros();
    //Serial.println("Rise");
  } else {
    Pulsdauer2 = micros() - PulsStart2;
    //Serial.println("Fall");
  }
}

void Zeitmessen3Change() {
  if (digitalRead(3) == LOW) {
    //invertiert da Optokoppler
    Periodendauer3 = micros() - PulsStart3;
    PulsStart3 = micros();
    //Serial.println("Rise");
  } else {
    Pulsdauer3 = micros() - PulsStart3;
    //Serial.println("Fall");
  }
}


void loop() {

  /*****CAN: mögliche Frames Empfangen, INT ist LOW wenn ein neuer Frame vorliegt*****/
  if (!digitalRead(CAN0_INT))                        // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
    //ausgewählten Frame übernehmen
    for (unsigned int i = 0; i < 8; i++) {
      rxSelFrame[i] = rxBuf[i];
    }

    //Prüfen ob PulseIn Frame
    if ((rxId & 0x1FFFFFFF) == PulseInFrameId) {
      Serial.println("PulseIn Frame empfagen: ");

      lastCANMessage_ms = millis(); //Timeout watchdog
      BlinkTypeLed1 = 1;

      FO1_out_CAN = rxBuf[0];
      FO2_out_CAN = rxBuf[4];

    } //end PulseIn Frame


    /*    //Debug-test
        if ((rxId & 0x80000000) == 0x80000000)    // Determine if ID is standard (11 bits) or extended (29 bits)
          sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
        else
          sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);
        Serial.print(msgString);
        Serial.println();
    */
  }

  /*****TASKS*****/
  unsigned long currentMillis = millis();

  //CAN Timeouts
  if (currentMillis - lastCANMessage_ms >= CanRxTimeout_ms) {
    //lastCANMessage_ms = currentMillis;
    FO1_out_CAN = 255;
    FO2_out_CAN = 255;

    BlinkTypeLed1 = 2;

    Serial.println("Kein CAN Frame empfangen -> Timeout -> SNA");
  }

   //Blink Tasks LED 1
    if (BlinkTypeLed1 == 0) {
      Led1State = LOW;
    }
    else if (BlinkTypeLed1 == 1) {
      //langsames Blinken
      if (currentMillis - prevMillisLed1 >= cycleLangsamBlinken) {
        //save the last time the Blink was computed
        prevMillisLed1 = currentMillis;
        // if the LED is off turn it on and vice-versa:
        if (Led1State == LOW) {
          Led1State = HIGH;
        } else {
          Led1State = LOW;
        }
      }
    }
    else if (BlinkTypeLed1 == 2) {
      //schnelles Blinken
      if (currentMillis - prevMillisLed1 >= cycleSchnellBlinken) {
        //save the last time the Blink was computed
        prevMillisLed1 = currentMillis;
        // if the LED is off turn it on and vice-versa:
        if (Led1State == LOW) {
          Led1State = HIGH;
        } else {
          Led1State = LOW;
        }
      }
    }
    digitalWrite(LED1_pin, Led1State);
    //delay(5);
    
/*
    //Blink Tasks LED Links
    if (BlinkTypeLedLinks == 0) {
      LedLinksState = LOW;
    }
    else if (BlinkTypeLedLinks == 1) {
      //langsames Blinken
      if (currentMillis - prevMillisLedLinks >= cycleLangsamBlinken) {
        //save the last time the Blink was computed
        prevMillisLedLinks = currentMillis;
        // if the LED is off turn it on and vice-versa:
        if (LedLinksState == LOW) {
          LedLinksState = HIGH;
        } else {
          LedLinksState = LOW;
        }
      }
    }
    else if (BlinkTypeLedLinks == 2) {
      //schnelles Blinken
      if (currentMillis - prevMillisLedLinks >= cycleSchnellBlinken) {
        //save the last time the Blink was computed
        prevMillisLedLinks = currentMillis;
        // if the LED is off turn it on and vice-versa:
        if (LedLinksState == LOW) {
          LedLinksState = HIGH;
        } else {
          LedLinksState = LOW;
        }
      }
    }
    digitalWrite(ledLinks, LedLinksState);
    //delay(5);
  */



  //100ms Task
  if (currentMillis - prevMillisCan >= cycleCan) { //cycle100ms
    // save the last time you run this Task
    prevMillisCan = currentMillis;

    for (int i = 0; i < 3; i++) {
      Tastverhaeltnis2[i + 1] = Tastverhaeltnis2[i];
      Tastverhaeltnis3[i + 1] = Tastverhaeltnis3[i];
    }
    Tastverhaeltnis2[0] = 100 * (float)Pulsdauer2 / (float)Periodendauer2;
    Tastverhaeltnis3[0] = 100 * (float)Pulsdauer3 / (float)Periodendauer3;

    Serial.print("THigh(2): ");
    Serial.print(Pulsdauer2);
    Serial.print("µs, T(2): ");
    Serial.print(Periodendauer2);
    Serial.print("µs, THigh/T(2): ");
    Serial.print(Tastverhaeltnis2[0]);
    Serial.print("%");

    Serial.print(",  THigh2(3): ");
    Serial.print(Pulsdauer3);
    Serial.print("µs, T(3): ");
    Serial.print(Periodendauer3);
    Serial.print("µs, THigh/T(3): ");
    Serial.print(Tastverhaeltnis3[0]);
    Serial.println("%");

    float Tastverhaeltnis2_avg = 0;
    float Tastverhaeltnis3_avg = 0;
    for (int i = 0; i < 4; i++) {
      Tastverhaeltnis2_avg += Tastverhaeltnis2[i];
      Tastverhaeltnis3_avg += Tastverhaeltnis3[i];
    }
    Tastverhaeltnis2_avg = Tastverhaeltnis2_avg / 4;
    Tastverhaeltnis3_avg = Tastverhaeltnis3_avg / 4;

    //FO1 Ausgang schreiben
    if (FO1_out_CAN > 1) {
      //Bereich ERR/SNA
      //Dann wird der FO Ausgang automatisch geschrieben bei PWM an Pin2 > 10% DutyCycle
      //Nach dem Start wird FO_out immer auf 255 gesetzt solange kein CAN Frame empfangen wird.
      Serial.print("FO1_out automatisch setzen (DutxCylePin3): ");
      Serial.print(Tastverhaeltnis2_avg);
      Serial.print(", ");
      if (Tastverhaeltnis2_avg > 10) {
        digitalWrite(4, HIGH);
        Serial.println("HIGH");
      } else {
        digitalWrite(4, LOW);
        Serial.println("LOW");
      }
    } else {
      //Vorgabe des Werte via CAN Frame Byte 0 Bit 1
      Serial.print("FO1_out manuell setzen: ");
      Serial.println(FO1_out_CAN);
      digitalWrite(4, FO1_out_CAN);
    }


    //FO2 Ausgang schreiben
    if (FO2_out_CAN > 1) {
      //Bereich ERR/SNA
      //Dann wird der FO Ausgang automatisch geschrieben bei PWM an Pin2 > 10% DutyCycle
      //Nach dem Start wird FO_out immer auf 255 gesetzt solange kein CAN Frame empfangen wird.
      Serial.print("FO2_out automatisch setzen (DutxCylePin3): ");
      Serial.print(Tastverhaeltnis3_avg);
      Serial.print(", ");
      if (Tastverhaeltnis3_avg > 10) {
        digitalWrite(5, HIGH);
        Serial.println("HIGH");
      } else {
        digitalWrite(5, LOW);
        Serial.println("LOW");
      }
    } else {
      //Vorgabe des Werte via CAN Frame Byte 0 Bit 1
      Serial.print("FO2_out manuell setzen: ");
      Serial.println(FO2_out_CAN);
      digitalWrite(5, FO2_out_CAN);
    }


    //PulseIn CAN versenden
    //CAN Frame senden
    byte sndStat = 0;
    // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
    data[0] = ((Pulsdauer2 >> 0) & 0xFF);
    data[1] = ((Pulsdauer2 >> 8) & 0xFF);
    data[2] = (((int)Tastverhaeltnis2_avg * 2 >> 0) & 0xFF);
    data[3] = ((Periodendauer2 / 500 >> 0) & 0xFF);

    data[4] = ((Pulsdauer3 >> 0) & 0xFF);
    data[5] = ((Pulsdauer3 >> 8) & 0xFF);
    data[6] = (((int)Tastverhaeltnis3_avg * 2 >> 0) & 0xFF);
    data[7] = ((Periodendauer3 / 500 >> 0) & 0xFF);

    sndStat = CAN0.sendMsgBuf(PulseInFrameID_Tx, 1, 8, data);  //18FEF398 = MotorpotiFrame
    if (sndStat == CAN_OK) {
      Serial.println("Message Sent Successfully!");
    } else {
      Serial.println("Error Sending Message...");
    }
  } //end CAN cycle (100ms)


} //end loop()


