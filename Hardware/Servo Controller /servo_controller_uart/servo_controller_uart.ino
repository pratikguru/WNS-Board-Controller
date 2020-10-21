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
int pulseCountAbsolute = 0;
int mode = 0;

void loop() {
  String incoming = "";
 
  // Opening with 1.
  // Closing with 0.
  // 15 rotations needed for opening/closing - first test.
  // 2,50,000 pulses for one rotation. 
  // 100uS pulse. Rising edge.
 

  while (Serial.available()) {
    String incoming_packet = Serial.readStringUntil('\n');
    if (incoming_packet[0] == 's') {
      pulseStart = 1;
      Serial.print("Init Complete | ");
      Serial.print("Direction: " + String(pulseDirection ? "Open | " : "close | "));
      Serial.println(" Time : " + String(timeDuration));
    }
    else if (incoming_packet[0] == 'g') {
      Serial.print("Direction: " + String(pulseDirection ? "Open | " : "close | "));
      Serial.println(" Time : " + String(timeDuration) + " | Mode: " + String(mode ? "Auto | " : "Manual | "));
    }
    else if (incoming_packet[0] == 'x') {
      pulseStart = 0;
      Serial.println("De-init Complete");
      Serial.print("Pulse Count: ");
      Serial.println(pulseCountAbsolute);
    }
    else if (incoming_packet[0] == 'd') {
      Serial.println("Direction Flip: " + String(pulseDirection) + " to " + String(!pulseDirection));
      pulseDirection = !pulseDirection;
    }
    else if (incoming_packet[0] == 'o') {
      Serial.println("Solenoid OPEN");
      digitalWrite(sol_a, LOW);
      digitalWrite(sol_b, HIGH);
    }
    else if (incoming_packet[0] == 'c') {
      Serial.println("Solenoid CLOSE");
      digitalWrite(sol_a, HIGH);
      digitalWrite(sol_b, LOW);
    }

    else if (incoming_packet[0] == 'a') {
      Serial.println("Auto Mode");
      mode = 1;
    }

    else if (incoming_packet[0] == 'm') {
      Serial.println("Manula Mode");
      mode = 0;
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
          timeDuration   = parsedTime.toInt();
          pulseDirection = (int)directionControl - '0';
        }
      }
      else {
        Serial.println("Servo Motor not initialized");
      }
    }
  }


  

  while (!Serial.available()) {
    if (pulseStart) {

      if (!mode) {
        pulseCountAbsolute++;
        digitalWrite(pulse, LOW);
        delayMicroseconds( timeDuration );
        digitalWrite(pulse, HIGH);
        digitalWrite(direction, pulseDirection);
      }
      else {
        Serial.println("Starting auto");
        //7 * 35000
        //250000 pulses for one rotation.
        for (int j = 0; j < 15; j++) {
          for (int32_t i = 0; i < 250000; i++) {
            digitalWrite(pulse, LOW);
            delayMicroseconds( timeDuration );
            digitalWrite(pulse, HIGH);
            digitalWrite(direction, pulseDirection);
          }
          Serial.println("Rotation Complete: " + String(j+1) + " | Direction: " + String(pulseDirection));
        }
        Serial.println("Ending auto");
        pulseStart = 0;
      }
    }
    else {
    }
  }
}
