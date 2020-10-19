

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>

ESP8266WebServer server(80);
IPAddress local_IP(10, 0, 0, 1);
IPAddress gateway(10, 0, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(8, 8, 8, 8);
#define POST_URL "http://wnsserver.wmswns.com/WNSRestServices/rest/insertFiledData1/masterRasp2/"

const char* deviceName = "raghavendraEncrypt.ooo";
// REPLACE WITH YOUR NETWORK CREDENTIALS
const char* ssid = "ENCRYPTION_SSID";
const char* password = "gurudatt123456";

const char* PARAM_INPUT_1 = "input1";
const char* PARAM_INPUT_2 = "input2";

bool CLIENT_MODE = false;

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
  <head>
    <title>ESP Input Form</title>
    <meta name="viewport" content="width=device-width, initial-scale=1" />
  </head>
  <body
    style="
      width: 100%;
      height: 100vh;
      display: flex;
      flex-direction: column;
      justify-content: center;
      align-items: center;
      background-color: white;
    "
  >
    <div
      style="
        background: transparent;
        padding: 50px;
        width: auto;
        height: auto;
        border-radius: 10px;
        box-shadow: 6px 6px 14px 0 rgba(0, 0, 0, 0.2),
          -8px -8px 18px 0 rgba(255, 255, 255, 0.55);
        display: flex;
        justify-content: center;
        align-content: center;
        flex-direction: column;
      "
    >
      <form
        action="/get"
        style="
          display: flex;
          height: auto;
          width: auto;
          flex-direction: column;
          justify-content: center;
          align-items: center;
        "
      >
        <input
          type="text"
          name="input1"
          placeholder="ssid"
          required
          style="
            padding: 10px;
            border-radius: 5px;
            width: 100%;

            border: 1px solid rgba(0, 0, 0, 0.2);
          "
        />

        <br />

        <input
          type="password"
          name="input2"
          placeholder="password"
          required
          style="
            padding: 10px;
            border-radius: 5px;
            margin-top: 10px;
            width: 100%;
            border: 1px solid rgba(0, 0, 0, 0.2);
          "
        />

        <br />

        <input
          type="submit"
          value="Submit"
          style="
            height: 50px;
            width: 100%;
            padding: 10px;
            margin-top: 10px;
            border-radius: 10px;
            border: none;
          "
        />
      </form>
    </div>
  </body>
</html>
)rawliteral";



int connectWifi(const char* ssid, const char* password) {
  WiFi.disconnect();
  WiFi.begin(ssid, password);

  int counter = 0;
  while(WiFi.status() != WL_CONNECTED) {
    delay(10);
    Serial.print(".");
    if (counter > 500){
      
      return 0;
    }
    else {
      counter++;
    }
    
  }  
//  Serial.println();
//  Serial.println("Connected to gateway");
//  Serial.print("IP : ");
//  Serial.println(WiFi.localIP());
  return 1;
}

void setWiFi(const char* Name)
{
  
  if (! WiFi.softAPConfig(local_IP, gateway, subnet)) {
    Serial.println("Allocation Error");
  }
  while (!WiFi.softAP(Name)) {
    Serial.print(".");
    delay(200);
  }
  
}


void blink(){
  digitalWrite(BUILTIN_LED, LOW);
  delay(300);
  digitalWrite(BUILTIN_LED, HIGH);
  delay(300);
}

void handleNewConnectionCredentials(){
  if(server.hasArg("input1") && server.hasArg("input2")) {
    
    
    if(connectWifi(server.arg("input1").c_str(), server.arg("input2").c_str())) { 
      server.send(200, "text/html", "connection OK!... <br><a href=\"/\">Return to Home Page</a>");
      CLIENT_MODE = true;
    }
    else { 
      server.send(200, "text/html", "connecting... <br><a href=\"/\">ERROR!!!!!! RETRY....Return to Home Page</a>");
      CLIENT_MODE = false;
    }

    for(int i = 0; i < 5; i++) {
      blink();
      delay(500);
    }    
  }
}

void handleRoot() {
  server.send(200, "text/html", index_html);
}


void disconnectWiFi(){
  Serial.println("Disconnecting Wifi");
  WiFi.disconnect();
  if(WiFi.status() != WL_CONNECTED){
    Serial.println("Disconnection success");
    CLIENT_MODE = false;
    server.send(200);
  }
}




void setup() {
  Serial.begin(9600);
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, HIGH);
  if(WiFi.reconnect()){
    CLIENT_MODE = true;
    
  }
  else{
    CLIENT_MODE=false;
    
  }
  setWiFi("ESP AP");
  server.on("/", HTTP_GET, handleRoot);
  server.on("/get", HTTP_GET, handleNewConnectionCredentials);
  server.on("/disconnect", HTTP_GET, disconnectWiFi);
  server.begin();
}

void loop() {
  server.handleClient();

  if(CLIENT_MODE){
    
    while(!Serial.available()){  
      digitalWrite(BUILTIN_LED, HIGH);
    }

    while(Serial.available()){
      digitalWrite(BUILTIN_LED, LOW);
      delay(200);
      digitalWrite(BUILTIN_LED, HIGH);
      delay(200);
      String incoming_data = Serial.readStringUntil('\n');
      Serial.println(incoming_data);
      String packet = "{\"hello\":\"world\"}";
      delay(10);  
      if(CLIENT_MODE){
        if(WiFi.status() == WL_CONNECTED) {
          HTTPClient httpClient; 
          httpClient.begin(POST_URL);
          httpClient.addHeader("Content-Type", "application/json");
          httpClient.addHeader("Content-Length", (String)(packet).length());
          int response = httpClient.POST(packet);
          Serial.print("Response: " );
          Serial.println(response);
        }
        else {
          Serial.println("Connection Error");
        }
      }
      else {
        Serial.println("Not operating in client mode");
      }
    }
  }
}
