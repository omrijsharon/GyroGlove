#include <EEPROM.h>


// Bend Sensor
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
#define BEND_Vin 16
int bendPin = A0;
float raw_throttle;
int throttle;
float throttle_avg = 0.0;
float throttle_var = 0.0;
int avg_size = 16; // averaging over 16 samples measured in 10bit gives 14bit accuracy when converting to joystick axis
float throttle_array[16]; // size of float array must be equal to avg_size
int idx;
int min_throttle;
int max_throttle;
int counter = 0;
int max_count = 500;
unsigned long StartTime;
unsigned long CurrentTime;
unsigned long ElapsedTime;

bool writeEEPROM = false;

int state = 0;

void setup() {
    EEPROM.get(12, min_throttle);
    EEPROM.get(14, max_throttle);
    pinMode(BEND_Vin, OUTPUT);
    digitalWrite(BEND_Vin, HIGH);
    Serial.begin(115200);
    delay(5000);
    for (int i=0; i<avg_size; i++){
        raw_throttle = analogRead(bendPin);
        throttle_avg += raw_throttle/avg_size;
        throttle_array[i] = raw_throttle;
    }
    for (int i=0; i<avg_size; i++){
        throttle_array[i] = raw_throttle;
        throttle_var += (sq(throttle_array[i] - throttle_avg)) / avg_size;
    }
    Serial.print("fisrt measurement: "); Serial.print(throttle_avg); Serial.print(" : "); Serial.println(throttle_var);
}

void loop() {
    // Calibrate min throttle
    Serial.print("Open your hand as wide as possible for throttle calibration:");
    delay(5000);

    StartTime = millis();
    ElapsedTime = 0;
    counter = 0;

    while (ElapsedTime<=5000 && counter < max_count) {
        
      for (int i=0; i<avg_size; i++){
          raw_throttle = analogRead(bendPin);
          throttle_avg = (throttle_avg * (avg_size-1) + raw_throttle) / avg_size;
          throttle_var = (throttle_var * (avg_size-1) + sq(raw_throttle - throttle_avg)) / avg_size;
      }
      CurrentTime = millis();
      ElapsedTime = CurrentTime - StartTime;
      // Serial.print(ElapsedTime); Serial.print(" : "); Serial.print(throttle_avg); Serial.print(" : "); Serial.print(sqrt(throttle_var)); Serial.print(": ");  Serial.println(counter);
      for (int j=0; j<max_count; j++){
        if ((j % 10)==0){
          if (j < counter) {
            Serial.print("!");
          }
          else {
            Serial.print("i");
          }
        }
      }
      if (sqrt(throttle_var) < 30.0) {
        counter ++ ;
      }
      else {
        counter = 0;
      }
      Serial.println("");

    }
    min_throttle = 16384 * throttle_avg/1024;
    Serial.print("minimum throttle set to "); Serial.println(min_throttle);

    // Calibrate max throttle
    Serial.print("Open your hand as wide as possible for throttle calibration:");
    delay(5000);

    StartTime = millis();
    ElapsedTime = 0;
    counter = 0;
    while (ElapsedTime<=5000 && counter < max_count) {
        
      for (int i=0; i<avg_size; i++){
          raw_throttle = analogRead(bendPin);
          throttle_avg = (throttle_avg * (avg_size-1) + raw_throttle) / avg_size;
          throttle_var = (throttle_var * (avg_size-1) + sq(raw_throttle - throttle_avg)) / avg_size;
      }
      CurrentTime = millis();
      ElapsedTime = CurrentTime - StartTime;
      // Serial.print(ElapsedTime); Serial.print(" : "); Serial.print(throttle_avg); Serial.print(" : "); Serial.print(sqrt(throttle_var)); Serial.print(": ");  Serial.println(counter);
      for (int j=0; j<max_count; j++){
        if ((j % 10)==0){
          if (j < counter) {
            Serial.print("!");
          }
          else {
            Serial.print("i");
          }
        }
      }
      if (sqrt(throttle_var) < 30.0) {
        counter ++ ;
      }
      else {
        counter = 0;
      }
      Serial.println("");

    }
    max_throttle = 16384 * throttle_avg/1024;
    Serial.print("maximum throttle set to "); Serial.println(max_throttle);
    if (writeEEPROM == false) {
      EEPROM.put(12, min_throttle);
      EEPROM.put(14, max_throttle);
      EEPROM.put(16, avg_size);
      writeEEPROM = true;
      Serial.println("throttle calbration parameters are stored in EEPROM.");
      Serial.println(" ");
      Serial.println(" ");
      delay(5000);

    }      

    while (true) {
      for (int i=0; i<avg_size; i++){
          raw_throttle = analogRead(bendPin);
          throttle_avg = (throttle_avg * (avg_size-1) + raw_throttle) / avg_size;
      }
      throttle = map(constrain(16 * throttle_avg, min_throttle, max_throttle), min_throttle, max_throttle, 0, 16384);
      Serial.print("throttle: "); Serial.print(throttle_avg*16); Serial.println(",");
    }
}

