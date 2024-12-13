#include <LiquidCrystal.h>
#include <Stepper.h>
#include <Wire.h>
#include <DHT.h>
#include <RTClib.h>

// Modifyable Global variables
float Temp_Limit = 78;
float Water_Limit = 500;

// Clock variables
RTC_DS1307 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// Stepper variables
const int stepsPerRevolution = 2048; 
const int rolePerMinute = 10; 
Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11); 

// LCD setup
LiquidCrystal lcd(2, 3, 4, 5, 6, 7); 

// DHT setup
#define DHTPIN 12 
#define DHTTYPE DHT11 
DHT dht(DHTPIN, DHTTYPE); 

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

// Timers
volatile unsigned char* myTCCR1A = (unsigned char*) 0x80;
volatile unsigned char* myTCCR1B = (unsigned char*) 0x81;
volatile unsigned int* myTCNT1 = (unsigned int*) 0x84;
volatile unsigned char* myTIFR1 = (unsigned char*) 0x36;
int ticks = 62500;  // Equals to 1 second 

// B register
volatile unsigned char* port_b = (unsigned char*) 0x25;
volatile unsigned char* ddr_b = (unsigned char*) 0x24;
volatile unsigned char* pin_b = (unsigned char*) 0x23;

// K register
volatile unsigned char* pin_k = (unsigned char*) 0x106;
volatile unsigned char* ddr_k = (unsigned char*) 0x107;
volatile unsigned char* port_k = (unsigned char*) 0x108;

// All functions
void adc_init();
unsigned int adc_read(unsigned char adc_channel);
unsigned int adc_read(unsigned char adc_channel);
void adc_init();
//void U0putchar(int U0pdata); //Never used but would be used if needed to print to the terminal
void U0init(unsigned long U0baud);
int Water_Sensor(); //Grabs data from the water sensor
void MyDelay(unsigned int ticks); //Delay function
void clock_module(); //Grabs the date/time from the rtc
void Vent_control(); //Controls the stepper motor
void LCD_error(); //LCD print for the error message
void LCD_data(float h, float f); //LCD print for the data during running and idle state
double DHT_sensor(); //Grabs data from the dht for temperature and humidity

//State funtions
void ERROR_STATE();
void RUNNING_STATE();
void IDLE_STATE();
void DISABLED_STATE();

//To read witch state system is in
volatile int toggle = 0;

void setup() {
  
  //RTC code
  U0init(57600);

  //Checks to make sure rtc time is running, and to set the time
  if (!rtc.isrunning()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); 
  }

  U0init.begin(9600);

  //Stepper motor setup
  myStepper.setSpeed(rolePerMinute);

  //LCD setup
  lcd.begin(16,2);  // Initialize the lcd row and column

  //DHT setup
  dht.begin();  // Set up the dht sensor

  //Setting up the timer
  *myTIFR1 = 0x00;
  *myTCNT1 = 0x00;
  *myTCCR1B = 0x00;
  *myTCCR1A = 0x00;

// ADC
  U0init(9600); // setup the UART
  adc_init(); // setup the ADC

  *ddr_b = B10001111;

  // button input
  *ddr_k = 0x00;
  *port_k = 0xFF;

  //Inturrupt setup
  PCICR |= (1 << PCIE1); 
  PCMSK1 |= (1 << PCINT8);
}

void loop() {
  if (toggle == 1) {
    DISABLED_STATE();
  }

  if (*pin_k == 0x00 && (Water_Sensor() > Water_Limit) && (DHT_sensor() < Temp_Limit)){
    IDLE_STATE();
  }

  if (*pin_k == 0x00 && (Water_Sensor() > Water_Limit) && (DHT_sensor() > Temp_Limit)){
    RUNNING_STATE();
  }

  if (*pin_k == 0x00 && (Water_Sensor() < Water_Limit)) {
    ERROR_STATE();
  }
}

//State functions

void DISABLED_STATE(){
  bool firstTime = true;
  while (*pin_k == 0x00){
    
    if(firstTime){
      toggle = 1;
      lcd.clear();
      *port_b &= 0x7F;
      *port_b |= 0x02;
      *port_b &= 0x72;
      firstTime = false;
    }
    
    clock_module();
    lcd.setCursor(0,0);
    lcd.print("System Off"); //Specific LCD print for disabled state
  }
}

void IDLE_STATE() {
  
  if(toggle != 2){
    toggle = 2;
    lcd.clear();
    *port_b &= 0x7F;
    *port_b |= 0x01;
    *port_b &= 0xF1;
  }
  
  clock_module();
  DHT_sensor();
  Vent_control();
}

void RUNNING_STATE() {
  
  if(toggle != 3){
    toggle = 3;
    lcd.clear();
    *port_b |= 0x08; 
    *port_b &= 0xF8;
    *port_b |= 0x80;
  }

  clock_module();
  DHT_sensor();
  Vent_control();
}

void ERROR_STATE() {
  
  if(toggle != 4){
    toggle = 4;
    lcd.clear();
    *port_b &= 0x7F;
    *port_b |= 0x04;
    *port_b &= 0xF4;
  }
  
  clock_module();
  LCD_error(); 
}

//Other functions

ISR(PCINT1_vect) {
  //Checks previous and current states
  static unsigned char lastPinState = 0xFF;
  unsigned char currentPinState = *pin_k;

  //Acts as the check for the rising edge and falling edge
  if ((lastPinState == 0x01) && (currentPinState == 0x00)) {
    toggle = 1;
  }
    if ((lastPinState == 0x00) && (currentPinState == 0x01)) {
    toggle = 0; 
  }

  // Update the last state
  lastPinState = currentPinState; 
}

double DHT_sensor() {
  //Gets the temperature and humidity
  float h = dht.readHumidity();
  float f = dht.readTemperature(true);

  // Display temperature and humidity to lcd data
  LCD_data(h, f); 

  return f;
}

void LCD_data(float h, float f) {
  //Displays the given information on the LCD display
  lcd.setCursor (0,0);
  lcd.print ("Humidity: ");
  lcd.print (h);
  lcd.print ("%");

  lcd.setCursor (0,1);
  lcd.print ("Temp: ");
  lcd.print (f);
  lcd.print (" F");
}

void LCD_error() {
  //Displays the error message on the LCD
  lcd.setCursor (0,0);
  lcd.print("     ERROR");

  lcd.setCursor(0,1);
  lcd.print ("   LOW WATER");
}


void Vent_control() {
  //Calculates the steps
  int steps = stepsPerRevolution / 360;
  
  //Turns stepper motor left
  while(*pin_k == 0x02){
    myStepper.step(steps);
  }
  //Turns stepper motor right
  while(*pin_k == 0x04){
    myStepper.step(-steps);
  }
}


void clock_module() {
  //Grabs the date and time from the rtc
  DateTime now = rtc.now();
}

void MyDelay (unsigned int ticks) {
  *myTCCR1B &= 0x00;
  *myTCCR1A &= 0x00;
  *myTCNT1 = (unsigned int) (65535 - ticks);
  *myTCCR1B |= 0x04;

  while ((*myTIFR1 &0x01)==0) {
    *myTCCR1B &= 0x00;
    *myTIFR1 |= 0x01;
  }
}

int Water_Sensor() {
  //Reads data from water sensor
  adc_init();
  int water = adc_read(0x00);
  return water;
}

void adc_init() {
  *AD_MUX |= 0xC0;
  *ADCSR_A |= 0xA0;
  *ADCSR_B |= 0x40;
  *ADC_DATA |= 0x00;
}

unsigned int adc_read(unsigned char adc_channel) {
  *ADCSR_B |= adc_channel;
  *ADCSR_A |= 0x40;
  while(!(*ADCSR_A & 0x40));
  return *ADC_DATA;
}

void U0init(int U0baud){
  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / 16 / U0baud - 1);
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0  = tbaud;
}
