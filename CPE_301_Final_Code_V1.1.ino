#include <LiquidCrystal.h>
#include "DHT.h"
#include <Stepper.h>
#include <Wire.h>
#include "RTClib.h"

// clock variables
RTC_DS1307 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// module Global variables
float Temp_Threshold = 78;
float Water_Threshold = 500;

// Global stepper varialbs
const int stepsPerRevolution = 2048;  // number of steps for a full revolution
const int rolePerMinute = 10;  // rotations per minute

// LCD
LiquidCrystal lcd(2, 3, 4, 5, 6, 7);  // set the LCD pin to 2, 3, 4, 5, 6, and 7

// DHT
#define DHTPIN 12 // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11 // DHT 11 type
DHT dht(DHTPIN, DHTTYPE); // Setting up the dht sensor module type and pin position

// Fan pins
#define ENABLE A2
#define DIRA A3
#define DIRB A4

//ADC SETUP
#define RDA 0x80
#define TBE 0x20  
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

volatile unsigned char *AD_MUX = (unsigned char*) 0x7C;
volatile unsigned char *ADCSR_A = (unsigned char*) 0x7A;
volatile unsigned char *ADCSR_B = (unsigned char *) 0x7b;
volatile unsigned int *ADC_DATA = (unsigned int*) 0x78;

volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;

long value = 0UL;
long maxtime = 160000000UL;

// Timers
volatile unsigned char* myTCCR1A = (unsigned char*) 0x80;
volatile unsigned char* myTCCR1B = (unsigned char*) 0x81;
volatile unsigned int* myTCNT1 = (unsigned int*) 0x84;
volatile unsigned char* myTIFR1 = (unsigned char*) 0x36;
int ticks = 62500;  // Equals to 1 second for timer delay function

// B register
volatile unsigned char* port_b = (unsigned char*) 0x25; // Setting the port_b (data register) to address 0x25 (sets bit as high or low, outputs data)
volatile unsigned char* ddr_b = (unsigned char*) 0x24;  // Setting the ddr_b (Data Direction Register) to address 0x24 (sets it as input or output)
volatile unsigned char* pin_b = (unsigned char*) 0x23;  // Setting pin_b (Input Pin Address) to 0x23 (Reading a value from a pin)

// K register
volatile unsigned char* pin_k = (unsigned char*) 0x106;
volatile unsigned char* ddr_k = (unsigned char*) 0x107;
volatile unsigned char* port_k = (unsigned char*) 0x108;


void adc_init();
unsigned int adc_read(unsigned char adc_channel);
unsigned int adc_read(unsigned char adc_channel);
void adc_init();
void MyDelay(unsigned int ticks);
void U0putchar(int U0pdata);
void U0init(unsigned long U0baud);
int Water_Sensor();
void TimerDelay (unsigned int ticks); 
void clock_module(); 
void Fan_Motor();
void Vent_control();
void LCD_error();
void LCD_data(float h, float f);
double DHT_sensor();
void ERROR_STATE();
void RUNNING_STATE();
void IDLE_STATE();
void DISABLED_STATE();

int toggle = 0;

void setup()
{
  //-----------------------RTC CODE-------------------------//
  U0init(57600);

  #ifndef ESP8266
  while (!Serial); // wait for serial port to connect. Needed for native USB
  #endif

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC"); //Error Message (Needs removal)
    while (1) delay(10);
  }

  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running, let's set the time!"); //Error Message (Needs removal)
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
//---------------------------------------------------------//
   
  U0init(9600);
  // Stepper motor
  myStepper.setSpeed(rolePerMinute);  // Initiating stepper motor

  // LCD
  lcd.begin(16,2);  // Initialize the lcd row and column

  // DHT
  dht.begin();  // Set up the dht sensor

  // Setting up the timer
  *myTIFR1 = B00000000;
  *myTCNT1 = B00000000;
  *myTCCR1B = B00000000;
  *myTCCR1A = B00000000;

// ADC
  U0init(9600); // setup the UART
  adc_init(); // setup the ADC

    //pin 13 controls the NPN transistor that turns the fan on and off
   
    //pin 53 is Led Green
    //pin 52 is Led Yellow
    //pin 51 is Led Red
    //pin 50 is Led Blue
  *ddr_b = B10001111;

  // button input
  *ddr_k = B01111111;
}

void loop() 
{
  if (*pin_k == B00000001)  //on/off == off
   {
     DISABLED_STATE();
   }
  if (*pin_k == B00000000 && (Water_Sensor() > Water_Threshold) && (DHT_sensor() < Temp_Threshold)) //on/off == on && water level > threshold && temp < threshold
    {
      IDLE_STATE();
    }

  if (*pin_k == B00000000 && (Water_Sensor() > Water_Threshold) && (DHT_sensor() > Temp_Threshold)) //on/off == on && water level > threshold && temp > threshold || reset == on
    {
      RUNNING_STATE();
    }

  if (*pin_k == B00000000 && (Water_Sensor() < Water_Threshold)) //on/off == on && water level < threshold 
    {
      ERROR_STATE();
    }
}
//-----------------------------------------STATES-----------------------------------------------//
void DISABLED_STATE()
{
  if(toggle != 1){
    toggle = 1;
    lcd.clear();
    *port_b &= B01111111;
    *port_b |= B00000010;
    *port_b &= B11111110;
    *port_b &= B11111011;
    *port_b &= B11110111;
    //*port_b |= B10000000; //pin 13 fan motor on
  }
  Serial.println("DISABLED State"); //Message (Needs removal)
  clock_module();
  Serial.println(); // Probably for a new line

  *port_b |= B00000010; // Led Yellow = on

  lcd.setCursor(0,0);
  lcd.print("System Off");
}

void IDLE_STATE() 
{
    if(toggle != 2){
    toggle = 2;
    lcd.clear();
    *port_b &= B01111111;
    *port_b |= B00000001; 
    *port_b &= B11111101;
    *port_b &= B11111011;
    *port_b &= B11110111;
    //*port_b |= B10000000; //pin 13 fan motor on
  }
  // monitor for state change and log it using clock module
  Serial.println("IDLE State"); //Message (Needs removal)
  clock_module();
  Serial.println();
  
    DHT_sensor(); // read from DHT sensor and display to lcd
    Vent_control();
}

void RUNNING_STATE() 
{
    if(toggle != 3){
    toggle = 3;
    lcd.clear();
    //*port_b &= B01111111; //pin 13 fan motor off
    *port_b |= B00001000; 
    *port_b &= B11111101;
    *port_b &= B11111011;
    *port_b &= B11111110;
    *port_b |= B10000000;
  }
  DHT_sensor();
  Vent_control();
  // monitor state change and log it using clock module
  Serial.println("RUNNING State"); //Message (Needs removal)
  clock_module(); 
  Serial.println();

}

void ERROR_STATE() 
{
    if(toggle != 4){
    toggle = 4;
    lcd.clear();
    *port_b &= B01111111;
    *port_b |= B00000100;
    *port_b &= B11110111; 
    *port_b &= B11111101;
    *port_b &= B11111110;
    //*port_b |= B10000000; //pin 13 fan motor on
  }
  Vent_control();
  Serial.println("ERROR State"); //Message (Needs removal)
  clock_module();
  Serial.println();

  LCD_error(); 
}

//----------------------------------------------------------------------------------------------//


// Sensor setup
double DHT_sensor() 
{
  float h = dht.readHumidity(); // Read humidity
  float f = dht.readTemperature(true);  // Read temperature as Fahrenheit (isFahrenheit = true)

  if (isnan(h) || isnan(f)) // Check if any reads failed and exit early (to try again).
  {
    Serial.println(F("Failed to read from DHT sensor!")); //Error Message (Needs removal)
    return;
  }
  LCD_data(h, f); // Display temperature and humidity to lcd data

  return f;
}

void LCD_data(float h, float f) 
{
  lcd.setCursor (0,0);
  lcd.print ("Humidity: ");
  lcd.print (h);
  lcd.print ("%");

  lcd.setCursor (0,1);
  lcd.print ("Temp: ");
  lcd.print (f);
  lcd.print (" F");
}

void LCD_error() 
{
  lcd.setCursor (0,0);
  lcd.print("     ERROR");

  lcd.setCursor(0,1);
  lcd.print ("   LOW WATER");
}


void Vent_control() 
{
  int steps = stepsPerRevolution / 360;
  //Serial.println ("waiting for direction");

    while(*pin_k == B00000010) //Analog pin 9
    {
      // step one revolution  in one direction:
      myStepper.step(steps);
      Serial.println ("left"); //Message (Needs removal)
    }

    while(*pin_k == B00000100) //Analog pin 10
    {
      // step one revolution in the other direction:
      myStepper.step(-steps);
      Serial.println ("right"); //Message (Needs removal)
    }
}


void clock_module() 
{
    DateTime now = rtc.now();

    Serial.print(now.year(), DEC); //Messages (Needs removal)
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" (");
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(") ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();
}

void TimerDelay (unsigned int ticks) 
{
  *myTCCR1B &= 0x00;
  *myTCCR1A &= 0x00;
  *myTCNT1 = (unsigned int) (65535 - ticks);
  *myTCCR1B |= 0b00000100;
  while ((*myTIFR1 &0x01)==0);
  *myTCCR1B &= 0x00;
  *myTIFR1 |= 0x01;
}

int Water_Sensor() 
{
  adc_init();
  int water = adc_read(0x00);
  return water;
}

void adc_init() 
{
  *AD_MUX |= 0b11000000;
  *ADCSR_A |= 0b10100000;
  *ADCSR_B |= 0b01000000;
  *ADC_DATA |= 0x00;
}

unsigned int adc_read(unsigned char adc_channel) 
{
  *ADCSR_B |= adc_channel;
  *ADCSR_A |= 0x40;
  while(!(*ADCSR_A & 0x40));
  return *ADC_DATA;
}

void U0init(int U0baud) //DELAY FUNCTION
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}
