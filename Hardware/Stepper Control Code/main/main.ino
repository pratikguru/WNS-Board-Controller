int direction = 8;
int pulse = 9;

int encoder0PinA = 10;
int encoder0PinB = 11;
int encoder0Pos = 0;
int encoder0PinALast = LOW;
int sol_a = 6;
int sol_b = 5;

int n = LOW;

void setup() {
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, INPUT);
  pinMode(11, INPUT);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  Serial.begin(115200);
}
int pulseStart = 0;
int timeDuration   = 5000;
int pulseDirection = 0;
int32_t pulseCount = 0;
void loop() {


  String incoming = "";
  //Opening with 1.
  //Closing with 0.

  //21 -> pulse
  //25 -> direction

  while (Serial.available()) {
    String incoming_packet = Serial.readStringUntil('\n');
    

    if (incoming_packet[0] == 's') {
      pulseStart = 1;
      Serial.print("Init Complete | ");
      Serial.print("Direction: " + String(pulseDirection ? "Open | " : "close | "));
      Serial.print(" Time : " + String(timeDuration));
    }
    else if (incoming_packet[0] == 'g') {
      
      Serial.print("Direction: " + String(pulseDirection ? "Open | " : "close | "));
      Serial.print(" Time : " + String(timeDuration));
    }
    else if (incoming_packet[0] == 'x') {
      pulseStart = 0;
      Serial.println("De-init Complete");
    }
    else if (incoming_packet[0] == 'd') {
      Serial.println("Direction Flip: " + String(pulseDirection) + " to " + String(!pulseDirection));
      pulseDirection = !pulseDirection;
    }
    else if(incoming_packet[0] == 'o'){
      Serial.println("Solenoid OPEN");
      digitalWrite(sol_a, LOW);
      digitalWrite(sol_b, HIGH);
    }
    else if(incoming_packet[0] == 'c') {
      Serial.println("Solenoid CLOSE");
      digitalWrite(sol_a, HIGH);
      digitalWrite(sol_b, LOW);
    }
    else {
      if (1) {
        int directionControl = incoming_packet[0];
        String parsedTime = "";
        for (int i = 1; i < incoming_packet.length(); i++) {
          parsedTime += incoming_packet[i];
        }

        Serial.println("Direction: " + String((int)(directionControl - '0')));
        Serial.println("Time: " + String(parsedTime.toInt()));

        if (parsedTime.toInt() < 100) {
          Serial.println("Speed to HIGH");
        }
        else {
          timeDuration = parsedTime.toInt();
          pulseDirection = (int)directionControl - '0';
        }
      }
      else {
        Serial.println("Servo Motor not initialized");
      }
    }
  }


  //13456 - driver-count;
  //14917 - driver-count
  //280593 pulses for opening. 
  //27000 pulses for opening. 

  
  while (!Serial.available()) {
    if (pulseStart) {
      n = digitalRead(encoder0PinA);
      if ((encoder0PinALast == LOW) && (n == HIGH)) {
        if (digitalRead(encoder0PinB) == LOW) {
          encoder0Pos--;
        } else {
          encoder0Pos++;
        }
        pulseCount += 1;
        
      }
      encoder0PinALast = n;
      digitalWrite(pulse, LOW);
      delayMicroseconds( timeDuration );
      digitalWrite(pulse, HIGH);
      digitalWrite(direction, pulseDirection);
    }
    else {

    }

  }
}
