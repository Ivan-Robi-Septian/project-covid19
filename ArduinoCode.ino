#include <SoftwareSerial.h>
#include <PZEM004Tv30.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <Adafruit_MLX90614.h>
#include "DHT.h"
#define output_pin 11
#define output_pinn 12
#define DHTPIN 2
#define DHTTYPE DHT22

//Sensor to Nextion
float temperature = 0.0f;
float humidity = 0.0f;
float perb = 0.0f;
float ug = 0.0f;
float tegangan = 0.0f;
float arus = 0.0f;
float daya = 0.0f;
float cosp = 0.0f;
int timeValue = 0;
int timen = 0;
int timem = 0;
int timeb = 0;
unsigned long mil = 0;
bool mode_ = 0;
bool modes = 0;

float PPM = 0.0f;
const int buzzer = 9;

//Variable for ozone Sensor MQ-131
int sensor_AOUT = A3; //connection from AOUT on sensor to A1 on Arduino
int sensor_DOUT = 3; //connection from DOUT on the sensor to pin 2 on Arduino
int rawvalue_AOUT; //variable for AOUT value
int rawvalue_DOUT; //variable for DOUT value
float value_AOUT; //variable for AOUT value
float PPM1 = 0.0;
//ADC min value
float x1 = 0;
//PPM min value
float y1 = 1.5;
//ADC max value
float x2 = 750;
//ADC max value
float y2 = 0;
float m;
float c;

// For millis
const unsigned long evenInterval = 1000;
unsigned long previousTime = 0;
const unsigned long evenInterval1 = 1000;
unsigned long previousTime1 = 0;
const unsigned long evenInterval2 = 1000;
unsigned long previousTime2 = 0;
const unsigned long evenInterval3 = 1000;
unsigned long previousTime3 = 0;
const unsigned long evenInterval4 = 1000;
unsigned long previousTime4 = 0;
const unsigned long evenInterval5 = 1000;
unsigned long previousTime5 = 0;
const unsigned long evenInterval6 = 1000;
unsigned long previousTime6 = 0;
const unsigned long evenInterval7 = 1000;
unsigned long previousTime7 = 0;
const unsigned long evenInterval8 = 1000;
unsigned long previousTime8 = 0;


Adafruit_MLX90614 mlx = Adafruit_MLX90614();
DHT dht(DHTPIN, DHTTYPE);
File myFile;
int pinCS = 53;
String dia, dib, dic, did, die, dif, dig;

//PZEM004Tv30 pzem(10,11);   // Software Serial pin 10 (RX) & 11 (TX) for arduino uno
PZEM004Tv30 pzem(&Serial3);  

void setup() {
  pinMode(output_pin, OUTPUT); 
  pinMode(output_pinn, OUTPUT); 
  pinMode(buzzer, OUTPUT);
  pinMode(sensor_DOUT, INPUT); 

  // Set up 
  Serial.begin(9600);
  dht.begin();
  mlx.begin();
  if (SD.begin())
  {
    Serial.println("SD card is ready to use.");
  } else
  {
    Serial.println("SD card initialization failed");
    return;
  }

}

void loop() {
  myFile = SD.open("testg.txt", FILE_WRITE);

  // Control and monitor from Nextion
  if (Serial.available()) {

    String indata = Serial.readStringUntil('#');
    if (indata.indexOf("on") > -1) {
      digitalWrite(output_pin, HIGH);
      Serial.print("b0.bc0=63504");
      Serial.write(0xFF);
      Serial.write(0xFF);
      Serial.write(0xFF);
      Serial.print("DEVICE IS READY");

      Serial.print("b1.bc0=0");
      Serial.write(0xFF);
      Serial.write(0xFF);
      Serial.write(0xFF);
      mode_ = 0; //manual Mode
    }
    else if (indata.indexOf("off") > -1) {
      digitalWrite(output_pin, LOW);
      Serial.print("b0.bc0=0");
      Serial.write(0xFF);
      Serial.write(0xFF);
      Serial.write(0xFF);

      Serial.print("b1.bc0=63504");
      Serial.write(0xFF);
      Serial.write(0xFF);
      Serial.write(0xFF);
      mode_ = 0; //manual Mode
      modes = 0;

    }
    else if (indata.indexOf("start") > -1) {
      byte position_ = indata.indexOf("start");
      timeValue = indata.substring(0, position_).toInt();
      mode_ = 1; //auto Mode
    }
    else if (indata.indexOf("auto") > -1) {
      digitalWrite(output_pin, HIGH);
      Serial.print("b4.bc0=63504");
      Serial.write(0xFF);
      Serial.write(0xFF);
      Serial.write(0xFF);
      timen = 600;
      timem = timen - timen * 0.3;
      timeb = 3;
      modes = 1;//manual Auto

    }
  }

    //Using millis not delay
  if (millis() - mil > 1000) {
    //1sec timer
    mil = millis();

    if (timeValue > 0 && mode_ == 1) {
      timeValue--;
      digitalWrite(output_pin, HIGH);
    }
    else  if (mode_ == 1) {
      digitalWrite(output_pin, LOW);
    }

    if (timem > 0 && modes == 1 && PPM >= 1.00) {
      timem--;
      if (timem > 0 && modes == 1 && PPM >= 1.30) {
        digitalWrite(output_pin, LOW);
      }
      else if (timem > 0 && modes == 1 && PPM <= 1.15) {
        digitalWrite(output_pin, HIGH);
      }
    } else if (timem == 0 && modes == 1) {
      digitalWrite(output_pin, LOW);
    }
    if (timeb > 0 && modes == 1 && PPM <= 0.10 && timem == 0) {
      timeb--;
      tone(buzzer, 1000);
    } else if (timeb == 0 && modes == 1) {
      noTone(buzzer);
      modes = 0;
      timeb = 3;
    }
  }//end of millis
  PPM =  perb;


  //Humudity and Temperature Sensor DHT22

  unsigned long currentTime = millis();
  if (currentTime - previousTime >= evenInterval) {
    humidity = -23.0838 + 0.9892 * dht.readHumidity();
    //  Check the result on the serial monitor
    //  Serial.print("Humidity");
    //  Serial.print("     ");
    //  Serial.print(dht.readHumidity());
    //  Serial.println("Temperature");
    //  Serial.print("        ");
    //  Serial.print(dht.readTemperature());

    String command = "humidity.txt=\"" + String(humidity, 1) + "\"";
    Serial.print (command);
    Serial.write (0xff);
    Serial.write (0xff);
    Serial.write (0xff);
 
    temperature = 1.3028041 + 0.9505023 * dht.readTemperature();
    String command2 = "temperature.txt=\"" + String(temperature, 1) + "\"";
    Serial.print (command2);
    Serial.write (0xff);
    Serial.write (0xff);
    Serial.write (0xff);
    // Ozone sensor MQ-131
    {
      rawvalue_AOUT = analogRead(sensor_AOUT); //download value from AOUT
      rawvalue_DOUT = digitalRead(sensor_DOUT); //download value with DOUT

      if (rawvalue_AOUT > 700) {
        PPM1 = 0.01;
      }

      else {
        m = (y2 - y1) / (x2 - x1);
        c = y1 - m * x1;
        value_AOUT = (m * rawvalue_AOUT) + c;
        float sum1 = 0;
        for (int i = 0; i < 50; i++) {
          sum1 = sum1 + value_AOUT;
        }
        PPM1 = sum1 / 50;
      }
      //  Check the result on the serial monitor
      //  Serial.print("PPM value =");
      //  Serial.print(rawvalue_AOUT);
      //  Serial.print("     ");
      //  Serial.print("adc value");
      //  Serial.println(PPM);
      //  Serial.print("digital value");
      //  Serial.println(rawvalue_DOUT);

      perb = PPM1;

      String command3 = "perb.txt=\"" + String(perb, 1) + "\"";
      Serial.print (command3);
      Serial.write (0xff);
      Serial.write (0xff);
      Serial.write (0xff);
    }

    //Current and voltage sensorPZEM 004T
    float voltage = pzem.voltage();
    if (voltage != NAN) {
      //  Check the result on the serial monitor    
      //        Serial.print("Tegangan: "); Serial.print(voltage); Serial.println("V");
    } else {
      //  Check the result on the serial monitor    
      //        Serial.println("Error reading voltage");
    }

    tegangan = -12.88 + 1.069 * voltage;
    String command4 = "tegangan.txt=\"" + String(tegangan, 1) + "\"";
    Serial.print (command4);
    Serial.write (0xff);
    Serial.write (0xff);
    Serial.write (0xff);
    float current = pzem.current();
    if (current != NAN) {
      //  Check the result on the serial monitor    
      //        Serial.print("Arus: "); Serial.print(current); Serial.println("A");
    } else {
      //  Check the result on the serial monitor    
      //        Serial.println("Error reading current");
    }

    arus = -0.096 + 0.4255 * current;
    String command5 = "current.txt=\"" + String(arus, 1) + "\"";
    Serial.print (command5);
    Serial.write (0xff);
    Serial.write (0xff);
    Serial.write (0xff);
    float power = pzem.power();
    if (power != NAN) {
      //  Check the result on the serial monitor    
      //        Serial.print("Daya: "); Serial.print(power); Serial.println("W");
    } else {
      //  Check the result on the serial monitor
      //        Serial.println("Error reading power");
    }

    float pf = pzem.pf();
    if (current != NAN) {
      //  Check the result on the serial monitor
      // Serial.print("PF: "); Serial.println(pf);
    } else {
      //  Check the result on the serial monitor
      //Serial.println("Error reading power factor");
    }

    cosp = pf;

    daya = tegangan * arus * cosp;
    String command6 = "power.txt=\"" + String(daya, 1) + "\"";
    Serial.print (command6);
    Serial.write (0xff);
    Serial.write (0xff);
    Serial.write (0xff);

    //Infrared sensor MLX
    {
      //  Check the result on the serial monitor    
      //Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempC());
      //Serial.print("*C\tObject = "); Serial.print(mlx.readObjectTempC()); Serial.println("*C");
      //Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempF());
      //Serial.print("*F\tObject = "); Serial.print(mlx.readObjectTempF()); Serial.println("*F");
      //Serial.println();}
      ug = mlx.readObjectTempC();
      String command7 = "ug.txt=\"" + String(ug, 1) + "\"";
      Serial.print (command7);
      Serial.write (0xff);
      Serial.write (0xff);
      Serial.write (0xff);

      // make the graph
    }
    int adcwave;
    if (rawvalue_AOUT < 750) {
      adcwave = rawvalue_AOUT;
    } else if (rawvalue_AOUT >= 750) {
      adcwave = 750;
    }

    int Value = map((750 - adcwave), 0, 1000, 0, 255); //Read the value
    String Tosend = "add "; //send the string "add "
    Tosend += 17; //send the id of the block 
    Tosend += ",";
    Tosend += 0; //Channel of the id
    Tosend += ",";
    Tosend += Value;//Send the value 
    Serial.print(Tosend);
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.write(0xff);
    int Value1 = map(mlx.readObjectTempC(), 0, 275, 0, 255); //Read the value 
    String Tosend1 = "add "; //send the string "add "
    Tosend1 += 18; //send the id of the block
    Tosend1 += ",";
    Tosend1 += 0; //Channel of the id
    Tosend1 += ",";
    Tosend1 += Value1;//Send the value 
    Serial.print(Tosend1);
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.write(0xff);

    int Value2 = map(dht.readTemperature(), 0, 275, 0, 255); //Read the value 
    String Tosend2 = "add ";//send the string "add "
    Tosend2 += 18; //send the id of the block
    Tosend2 += ",";
    Tosend2 += 1; //Channel of the id
    Tosend2 += ",";
    Tosend2 += Value2; //Send the value 
    Serial.print(Tosend2);
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.write(0xff);
    // Log to SD CARD
    if (myFile) {
      dia = String(humidity);
      dib = String(temperature);
      dic = String(perb);
      did = String(tegangan);
      die = String(arus);
      dif = String(daya);
      dig = String(ug);
      myFile.print(dia);
      myFile.print(",");
      myFile.print(dib);
      myFile.print(",");
      myFile.print(dic);
      myFile.print(",");
      myFile.print(did);
      myFile.print(",");
      myFile.print(die);
      myFile.print(",");
      myFile.print(dif);
      myFile.print(",");
      myFile.print(dig);
      myFile.print(",");
    }
    else {
      Serial.println("error opening file.txt");
    }
    myFile.close();
    previousTime = currentTime;
  }
}
