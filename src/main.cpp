// include the library code:
#include <Adafruit_CircuitPlayground.h>
#include "arduinoFFT.h"
#include "Wire.h"
#include "Adafruit_LiquidCrystal.h"

const uint16_t samples = 128; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = samples / 3.0;
unsigned int sampling_period_us;
unsigned long microseconds;

uint32_t OFFTIME_COLOR = 0x7DDA58; // green
uint32_t DYSKINESIA_COLOR = 0xD20103; // red
uint32_t NONE_COLOR = 0x060270; // blue

float X, Y, Z;

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);

// initialize the library with the numbers of the interface pins
Adafruit_LiquidCrystal lcd(2, 3, 12, 6, 9, 10);

void setAllPixels(uint32_t color);
void sendToLCD(String condition);

void setup() {
  Serial.begin(115200);          // faster printout
  CircuitPlayground.begin();
  lcd.begin(16, 2);

  // while(!Serial);
  Serial.println("Serial Ready");

  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  microseconds = micros();
}

bool active = false;
bool lastButton = false;


void loop() {
  /* SAMPLING
    determine the frequency of the movement from the accelerometer X Y Z values
    and store them in the input vector vReal[]
  */
  bool currentButton = CircuitPlayground.leftButton();

  if (currentButton && !lastButton) {
    active = !active;
    CircuitPlayground.clearPixels();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Press to start");
  }

  Serial.print("System is now ");
  Serial.println(active ? "ACTIVE" : "INACTIVE");

  if (!active) {
    lastButton = currentButton;
    return;
  }


  for (uint16_t i=0; i<samples; i++){

    X = CircuitPlayground.motionX();
    Y = CircuitPlayground.motionY();
    Z = CircuitPlayground.motionZ();
    
    vReal[i] = sqrt(X*X + Y*Y + Z*Z); // calculate the magnitude of the vector
    vImag[i] = 0.0; // set the imaginary part to 0

    while (micros() - microseconds < sampling_period_us); // wait until next sample
    microseconds += sampling_period_us;
  }

  /* FFT
    compute the FFT of the input vector vReal[] and store the results in the output vector vReal[]
  */

  FFT.dcRemoval();	/* Remove DC component (gravity) */
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);	/* Weigh data */
  FFT.compute(FFTDirection::Forward);	/* Compute FFT */
  FFT.complexToMagnitude(); /* Compute magnitudes */
  
  for (uint16_t i=0; i < (samples>>1); i++){
    double freq = (i * 1.0 * samplingFrequency) / samples; // calculate the frequency of each bin
    Serial.print("Hz: ");
    Serial.println(freq);

    Serial.print(">Data:");
    Serial.println(vReal[i], 4); //Print out the magnitude of each bin
  }

  // print out the highest magnitude of the major peak with the corresponding frequency
  Serial.print("MajorPeak (Magnitude): ");
  double frequency, magnitude; 
  FFT.majorPeakParabola(&frequency, &magnitude);
  Serial.print(frequency, 4);
  Serial.print("Hz ");
  Serial.print(magnitude, 4);
  Serial.println(" ");

  /*
    Determine the threshold magnitude for the frequencies that we want to detect 
  */

  CircuitPlayground.clearPixels();

  if ((magnitude > 10 && magnitude < 100) && (frequency >= 3.0 && frequency < 5.0)){
    Serial.println("Detected: Offtime");
    sendToLCD("Offtime");
    setAllPixels(OFFTIME_COLOR); // green
  }
  else if ((magnitude >= 100 && magnitude < 200) && (frequency >= 5.0 && frequency <= 7.0)) {
    Serial.println("Detected: Dyskinesia");
    sendToLCD("Dyskinesia");
    setAllPixels(DYSKINESIA_COLOR); // red
  }
  else {
    Serial.println("Detected: None");
    sendToLCD("None");
    setAllPixels(NONE_COLOR); // blue
  }

}

void setAllPixels(uint32_t color){
  for (int i = 0; i < 10; i++){
    CircuitPlayground.setPixelColor(i, color);
  }
}

void sendToLCD(String condition){
  lcd.clear(); 
  lcd.setCursor(0, 0);
  lcd.print("Detected:");
  lcd.setCursor(0, 1);
  lcd.print(condition);  
}



