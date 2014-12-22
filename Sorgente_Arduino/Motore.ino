#define PinAttivo 9
int accessByte;




void setup(){
  Serial.begin(9600);
  pinMode(PinAttivo, OUTPUT);
  analogReference(INTERNAL);
  Serial.println(dRes);
  
}

void loop() {
  // see if there's incoming serial data:
  while(Serial.available()) {
    // read the oldest byte in the serial buffer:
    accessByte = Serial.read();
    // if it's a capital H (ASCII 72), turn on the LED:
    if (accessByte == 'M') {
        analogWrite(PinAttivo, 200);
        // Serial.println("Partito!");
        delay(random(5, 15));
        analogWrite(PinAttivo, 0);
    }
    if (accessByte == 'F') {
      int sensorValue = analogRead(A0);
      // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
      float voltage = sensorValue * (1.1 / 1023.0);
      // print out the value you read:
      //Serial.print("tensione : ");
      Serial.println(voltage,4);
    }
  }
}

  
 
