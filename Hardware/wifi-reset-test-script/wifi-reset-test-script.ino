

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Ready");
  
}

void loop() {
  while(!Serial.available()){
    
  }

  while(Serial.available()){
    digitalWrite(BUILTIN_LED, LOW);
    delay(200);
    digitalWrite(BUILTIN_LED, HIGH);
    delay(200);
    String incoming_data =Serial.readStringUntil('\n');
    delay(10);
    Serial.println(incoming_data);
  }
  
}
