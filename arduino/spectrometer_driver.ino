/*
* Author: GroupGets, with modifications by Nathaniel Hanson
* Date: 07/12/2022
* Purpose: Spectrometer Driver for C12880MA Mini-Spectrometer from Hamamatsu
*/

/*
 * Macro Definitions
 */
 
#define SPEC_TRG         A0
#define SPEC_ST          A1
#define SPEC_CLK         A2
#define SPEC_VIDEO       A3
#define WHITE_LED        A4
#define LASER_404        A5

#define SPEC_CHANNELS    288 // New Spec Channel
uint16_t data[SPEC_CHANNELS];
int delayTime = 1;

void setup(){

  //Set desired pins to OUTPUT
  pinMode(SPEC_CLK, OUTPUT);
  pinMode(SPEC_ST, OUTPUT);
  pinMode(LASER_404, OUTPUT);
  pinMode(WHITE_LED, OUTPUT);

  digitalWrite(SPEC_CLK, HIGH); // Set SPEC_CLK High
  digitalWrite(SPEC_ST, LOW); // Set SPEC_ST Low
  Serial.begin(115200); // Baud Rate set to 115200
}

/*
 * This functions reads spectrometer data from SPEC_VIDEO
 * Look at the Timing Chart in the Datasheet for more info
 */
void readSpectrometer(){
  // Start clock cycle and set start pulse to signal start
  digitalWrite(SPEC_CLK, LOW);
  delayMicroseconds(delayTime);
  digitalWrite(SPEC_CLK, HIGH);
  delayMicroseconds(delayTime);
  digitalWrite(SPEC_CLK, LOW);
  digitalWrite(SPEC_ST, HIGH);
  delayMicroseconds(delayTime);

  //Sample for a period of time
  for(int i = 0; i < 15; i++){

      digitalWrite(SPEC_CLK, HIGH);
      delayMicroseconds(delayTime);
      digitalWrite(SPEC_CLK, LOW);
      delayMicroseconds(delayTime); 
 
  }

  //Set SPEC_ST to low
  digitalWrite(SPEC_ST, LOW);

  //Sample for a period of time
  for(int i = 0; i < 85; i++){

      digitalWrite(SPEC_CLK, HIGH);
      delayMicroseconds(delayTime);
      digitalWrite(SPEC_CLK, LOW);
      delayMicroseconds(delayTime); 
      
  }

  //One more clock pulse before the actual read
  digitalWrite(SPEC_CLK, HIGH);
  delayMicroseconds(delayTime);
  digitalWrite(SPEC_CLK, LOW);
  delayMicroseconds(delayTime);

  //Read from SPEC_VIDEO
  for(int i = 0; i < SPEC_CHANNELS; i++){

      data[i] = analogRead(SPEC_VIDEO);
      
      digitalWrite(SPEC_CLK, HIGH);
      delayMicroseconds(delayTime);
      digitalWrite(SPEC_CLK, LOW);
      delayMicroseconds(delayTime);
        
  }

  //Set SPEC_ST to high
  digitalWrite(SPEC_ST, HIGH);

  //Sample for a small amount of time
  for(int i = 0; i < 7; i++){
    
      digitalWrite(SPEC_CLK, HIGH);
      delayMicroseconds(delayTime);
      digitalWrite(SPEC_CLK, LOW);
      delayMicroseconds(delayTime);
    
  }

  digitalWrite(SPEC_CLK, HIGH);
  delayMicroseconds(delayTime);
  
}

/*
 * The function below prints out data to the terminal or 
 * processing plot
 */
void printData(){
  
  for (int i = 0; i < SPEC_CHANNELS; i++){
    
    Serial.print(data[i]);
    Serial.print(',');
    
  }
  
  Serial.print("\n");
}

void readSerial() {
  String str = Serial.readString();
  // Check string is non-empty
  if (str.length() < 3){
    return;
  }
  // Set light value
  if (str.indexOf("light_power") > -1) {
    // Set light equal to the power value
    int offset = 12; // Number of characters until useful data begins
    String command = str.substring(offset);
    command.trim();
    if (command.equals("ON")) {
      digitalWrite(WHITE_LED, HIGH); 
    } else if (command.equals("OFF")) {
      digitalWrite(WHITE_LED, LOW); 
    }
    return;
  }
  // Set integration time
  if (str.indexOf("integration_time") > -1) {
    // Extract the integration time to use
    // Set light equal to the power value
    int offset = 16; // Number of characters until useful data begins
    String command = str.substring(offset);
    command.trim();
    // Convert string to integer value
    int newTime = command.toInt();
    delayTime = newTime;
  }
}

void loop(){
   
  readSpectrometer();
  printData();
  delay(10);
  readSerial();
   
}