/* Campervan console project
 *  Github: https://github.com/manromao/Campervan-console
 *  Includes:
 *  LCD display
 *  Humidity and temperature sensor (DHT)
 *  Accelerometer
 *  Barometer
 *  by Manromao
 */

//Dependencies
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <DHT.h>
#include <EEPROM.h>
#include <Adafruit_BMP280.h>

//Constants
#define DHTPIN 8              // input pin for the DHT sensor
#define IMU_ADDRESS 0x68      // Address of IMU scanner
#define BUTTON_PIN 2          //Pushbutton pin
#define LED_PIN  13           //LED pin
#define DHTTYPE DHT11         // type of DHT sensor
#define FREQUENCY_DATA 1000   //Frequency to update the data read.
#define FREQUENCY_BUTTON 100  //Frequency to read the button push
#define SCREEN_DEFAULT 0      //Default initial screen. Will be +1 on first execution.
#define MAX_SCREENS 3         //number of screens on the display. One per sensor
#define BUTTON_TIME 3000      //Time to trigger the calibration screen
#define CAL_LOOPS 10          //Loops for the calibration


//Global variables for sensor data
int16_t AcX,AcY,AcZ,Tmp_acc,GyX,GyY,GyZ;
double pitch,roll,pitch_offset,roll_offset =0.00d ;
float hum_dht,t_dht;

//Global variables for screen management
int current_screen = SCREEN_DEFAULT;

//Variables for button
unsigned long dataPrevMillis = 0;
unsigned long keyPrevMillis = 0;
byte longKeyPressCountMax = 10;    
byte longKeyPressCount = 0;
byte prevKeyState = HIGH;        

//Sensor objects. If issues with address, use I2C scanner
Adafruit_BMP280 bmp; // I2C Interface
LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7,3,POSITIVE);
DHT dht(DHTPIN, DHTTYPE);

void setup()
{
  Serial.begin(9600);
  Serial.println("Serial Initialized");
  
  //Connections for IMU
  Serial.print("Initializing IMU...");
  Wire.begin();
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.println("Initialized!");
  
  //Initialize LCD
  Serial.print("Initializing LCD...");
  lcd.begin(16,2); // LCD 2x16 initialization
  lcd.setCursor(0,0); 
  lcd.print("Hello!");
  lcd.setCursor(0,1); 
  lcd.print("JelouVan!");
  Serial.println("Initialized!");

  // initialize the pushbutton pin as an input:
  pinMode(BUTTON_PIN, INPUT);

  Serial.print("Initializing DHT...");
  dht.begin();
  Serial.println("Initialized!");

  //Initiliaze BMP
  Serial.print("Initializing BMP...");
  bmp.begin();
    /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  Serial.println("Initialized!");


  //Read calibrated values... if empty they will be initialized to 0
  Serial.print("Read from static memory...");
  readMsg();
  Serial.print(pitch_offset);Serial.print(" ");Serial.print(roll_offset);
  Serial.println("Initialized!");

  Serial.println("End of setup");
  
  delay(3000);
}

void loop()
{  
  // read the state of the pushbutton value:
   if (millis() - keyPrevMillis >= FREQUENCY_BUTTON) {
        keyPrevMillis = millis();
        
        byte currKeyState = digitalRead(BUTTON_PIN);
        
        if ((prevKeyState == HIGH) && (currKeyState == LOW)) {
            keyRelease();
        }
        else if ((prevKeyState == LOW) && (currKeyState == HIGH)) {
            keyPress();
        }
        else if (currKeyState == HIGH) {
            longKeyPressCount++;
        }
        
        prevKeyState = currKeyState;
    }
    
    if (millis() - dataPrevMillis >= FREQUENCY_DATA) {
      dataPrevMillis = millis();
      screen_update();
      }    
}

void screen_update(){
      Serial.println("Clearing LCD");
      lcd.clear();
      switch (current_screen) {
        case 1:
          screen1();
          break;
        case 2:
          screen2();
          break;
        case 3:
          screen3();
          break;       
        default:
          screen1();
          break;
      }
  
}
//Level screen
void screen1(){
  Serial.println("Screen1");
  readAcc();

  Serial.print(pitch-pitch_offset);
  Serial.print(" ");
  Serial.print(roll-roll_offset);
  Serial.print(" ");
  //Serial.print(t_acc);
  Serial.print(" ");

  int print_pitch = round5(pitch - pitch_offset);
  int print_roll = round5(roll - roll_offset);

 //Printing pitch
  lcd.setCursor(1,0);   
  lcd.print("+");
  lcd.setCursor(1,1);
  lcd.print("-");

  if (print_pitch >=0){
    lcd.setCursor(2,0);
  }
  else{
    lcd.setCursor(1,1);
  }
  lcd.print(print_pitch);
  lcd.print("*");

   //Printing roll
  lcd.setCursor(11,0);   
  lcd.print("+");
  lcd.setCursor(10,0);
  lcd.print("-");

  if (print_roll >=0){
    lcd.setCursor(12,0);
  }
  else{
    lcd.setCursor(6,0);
  }
  lcd.print(print_roll);
  lcd.print("*");

}
//DHT screen
void screen2(){
  Serial.println("Screen2");
  readDHT();
  
  Serial.print(hum_dht);
  Serial.print(" ");
  Serial.println(t_dht);

  lcd.setCursor(0,0); 
  lcd.print("T: ");
  lcd.print(round(t_dht));
  lcd.print(" *C");

  lcd.setCursor(0,1); 
  lcd.print("H: ");
  lcd.print(round(hum_dht));
  lcd.print(" %");
}
//altimeter screen
void screen3(){
  double P = bmp.readPressure()/100;
  double Alt = bmp.readAltitude(1019.66);
  
  Serial.println("Screen3");
  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(P = bmp.readPressure()/100); //displaying the Pressure in hPa, you can change the unit
  Serial.println(" hPa");

  Serial.print(F("Approx altitude = "));
  Serial.print(Alt); //The "1019.66" is the pressure(hPa) at sea level in day in your region
  Serial.println(" m");                    //If you don't know it, modify it until you get your current altitude

  lcd.setCursor(0,0); 
  lcd.print("P:");
  lcd.print(round(P));
  lcd.print(" Pa");

  lcd.setCursor(0,1); 
  lcd.print("Alt:");
  lcd.print(round(Alt));
  lcd.print(" m");
}


void readDHT(){
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  hum_dht = dht.readHumidity();
  // Read temperature as Celsius (the default)
  t_dht = dht.readTemperature();

  // Check if any reads failed and exit early (to try again).
  if (isnan(hum_dht) || isnan(t_dht)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

}

void readAcc()
{  
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU_ADDRESS,14,true);

  int AcXoff,AcYoff,AcZoff,GyXoff,GyYoff,GyZoff;
  int temp,toff;
  
  //Acceleration data correction
  /*
  AcXoff = -950;
  AcYoff = -300;
  AcZoff = 0;
 */


  AcXoff = 0;
  AcYoff = 0;
  AcZoff = 0;

  //Temperature correction
  toff = -1600;

  //Gyro correction
  GyXoff = 480;
  GyYoff = 170;
  GyZoff = 210;

  //read accel data
  AcX=(Wire.read()<<8|Wire.read()) + AcXoff;
  AcY=(Wire.read()<<8|Wire.read()) + AcYoff;
  AcZ=(Wire.read()<<8|Wire.read()) + AcYoff;

  //read temperature data
  temp=(Wire.read()<<8|Wire.read()) + toff;
  //t_acc = temp/340 + 36.53;


  //read gyro data
  GyX=(Wire.read()<<8|Wire.read()) + GyXoff;
  GyY=(Wire.read()<<8|Wire.read()) + GyYoff;
  GyZ=(Wire.read()<<8|Wire.read()) + GyZoff;

  //get pitch/roll
  getAngle(AcX,AcY,AcZ);

  //send the data out the serial port
  //Serial.print("Angle: ");
  //Serial.print("Pitch = "); Serial.print(pitch);
  //Serial.print(" | Roll = "); Serial.println(roll);

  //Serial.print("Temp: ");
  //Serial.print("Temp(F) = "); Serial.print(tf);
  //Serial.print(" | Temp(C) = "); Serial.println(tx);

  //Serial.print("Accelerometer: ");
  //Serial.print("X = "); Serial.print(AcX);
  //Serial.print(" | Y = "); Serial.print(AcY);
  //Serial.print(" | Z = "); Serial.println(AcZ);

  //Serial.print("Gyroscope: ");
  //Serial.print("X = "); Serial.print(GyX);
  //Serial.print(" | Y = "); Serial.print(GyY);
  //Serial.print(" | Z = "); Serial.println(GyZ);
  //Serial.println(" ");
  
}

//convert the accel data to pitch/roll
void getAngle(int Vx,int Vy,int Vz) {
  double x = Vx;
  double y = Vy;
  double z = Vz;
  
  pitch = atan(x/sqrt((y*y) + (z*z)));
  roll = atan(y/sqrt((x*x) + (z*z)));
  //convert radians into degrees
  pitch = pitch * (180.0/3.14);
  roll = roll * (180.0/3.14) ;
}

int round5(int n) {
  return (n/5 + (n%5>2)) * 5;
}

//Update screen on short key press
void shortKeyPress() {
      current_screen +=1;
      Serial.println("Button ON");
      Serial.print("Switching to screen: ");
      Serial.println(current_screen);
      
      if (current_screen > MAX_SCREENS){
        current_screen= SCREEN_DEFAULT;
      } 
      screen_update();
}


// called when button is kept pressed for more than 2 seconds. For calibrating ACc
void longKeyPress() {
    Serial.println("Calibrating...");
    lcd.clear();
    lcd.setCursor(0,0); 
    lcd.print("Calibrating...");
    
    int i = 0;
    double p,r = 0;    
    while (i<CAL_LOOPS){
      readAcc();
      p = p + pitch/CAL_LOOPS;
      r = r + roll/CAL_LOOPS;
      delay(200);
      i++;
    }    
    pitch_offset=p;
    roll_offset=r;

    Serial.println(pitch_offset);
    Serial.println(roll_offset);

    //Write to static memory
    if (isnan(pitch_offset)){pitch_offset =0.00d;}
    if (isnan(roll_offset)){roll_offset =0.00d;}
    
    writeMsg(pitch_offset, roll_offset);

    lcd.setCursor(0,0); 
    lcd.print("Done!");
}

// called when key goes from not pressed to pressed
void keyPress() {
    Serial.println("key press");
    longKeyPressCount = 0;
}

// called when key goes from pressed to not pressed
void keyRelease() {
    Serial.println("key release");
    if (longKeyPressCount >= longKeyPressCountMax) {
        longKeyPress();
    }
    else {
        shortKeyPress();
    }
}

void erase(void)
{
  for (int i = 0 ; i < EEPROM.length() ; i++)
  EEPROM.write(i, 0);
}

void writeMsg(double data1, double data2)
{
  erase();
  EEPROM.put(0,data1);
  EEPROM.put(sizeof(data1)+1,data2);

}

void readMsg()
{
  EEPROM.get(0,pitch_offset);
  if (isnan(pitch_offset)){pitch_offset=0;}

  EEPROM.get(sizeof(pitch_offset)+1,roll_offset);
  if (isnan(roll_offset)){roll_offset=0;}
}
