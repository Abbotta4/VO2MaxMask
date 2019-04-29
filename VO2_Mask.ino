#include <Wire.h>
#include <MAX30105.h>
#include <heartRate.h>

// Heart rate variables
MAX30105 particleSensor;

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

// Anemometer variables
const float zeroWindAdjustment =  0; // negative numbers yield smaller wind speeds and vice versa.

int TMP_Therm_ADunits;  //temp termistor value from wind sensor
float RV_Wind_ADunits;    //RV output from wind sensor 
float RV_Wind_Volts;
unsigned long lastMillis;
int TempCtimes100;
float zeroWind_ADunits;
float zeroWind_volts;
float WindSpeed_MPH;

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");

  // Initialize HR sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power.\n");
    Serial.flush();
    while (1);
  }
  Serial.println("Attach heart rate sensor.");

  particleSensor.setup(); // Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); // Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); // Turn off Green LED
}

void loop() {
  if (millis() - lastMillis > 200) { // read every 200 ms - printing slows this down further
    
    TMP_Therm_ADunits = analogRead(A1);
    RV_Wind_ADunits = analogRead(A2);
    RV_Wind_Volts = (RV_Wind_ADunits *  0.0048828125);

    zeroWind_ADunits = -0.0006*((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits) + 1.0727 * (float)TMP_Therm_ADunits + 47.172;  //  13.0C  553  482.39

    zeroWind_volts = (zeroWind_ADunits * 0.0048828125) - zeroWindAdjustment;  
    
    WindSpeed_MPH =  pow(((RV_Wind_Volts - zeroWind_volts) /.2300) , 2.7265);

    Serial.print("   WindSpeed MPH ");
    Serial.println((float)WindSpeed_MPH);
    lastMillis = millis();    
  }

  long irValue = particleSensor.getIR(); // Read photodiode info

  if (checkForBeat(irValue) == true) {
    // sensed a beat
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);               // wait for a tenth of a second
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);
  Serial.print(", WindSpeed MPH=");
  Serial.println((float)WindSpeed_MPH);

  if (irValue < 50000)
    Serial.println(" No finger?");
}
