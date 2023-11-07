#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

ESP8266WebServer httpd(80);
bool MijoteuseOn = true;
float min2Min = 0.0;
float max2Min = 0.0;
float min5Min = 0.0;
float max5Min = 0.0;
int tempsTemp = 0;

float getCurrentTemp() {
  // Read temperature from the sensor and return the value
  int valeur = analogRead(A0);
  
  //Formule pour transformer en degré -> 10000 pour resistance 10k, 3977 pour beta
  float voltage = valeur * 5.0 / 1023.0;
  float resistance = 10000.0 * voltage / (5.0 - voltage);
  float tempKelvin = 1.0 / (1.0 / 298.15 + log(resistance / 10000.0) / 3977); 

  float tempCelcius = tempKelvin - 273.15;

  return tempCelcius;
}

float getIntensité() {
  
}

float* get2Min() {
  float arr[2];
  arr[0] = min2Min;
  arr[1] = max2Min;
  return arr;
}

float* get5Min() {
  float arr[2];
  arr[0] = min5Min;
  arr[1] = max5Min;
  return arr;
}

void handleRacine(){
  String response = "<html><body>C'EST TROP COOL</body></html>";
  httpd.send(200, "text/html", response);
}

void handleMijoteuse() {
  if (httpd.hasArg("action"))
  {
    String action = httpd.arg("action");
    if(action == "on")
      MijoteuseOn = true;
    else if (action == "off")
      MijoteuseOn = false;
  }
  
  String reponse = "<html><body>";
    reponse += "<a href=\"/mijoteuse?action=on\">ON</a>";
    reponse += "<a href=\"/mijoteuse?action=off\">OFF</a>";
    reponse += "<div style=\"width: 10px; height: 10px; background-color: ";
    reponse += (MijoteuseOn ? "green" : "red");
    reponse += "\"></div>";
    reponse += "</body></html>";

    httpd.send(200, "text/html", reponse.c_str()); 
}

void setup() {
  Serial.begin(115200);

  Serial.println("Creation de l'AP...");
  WiFi.softAP("WifiTropCool", "Qwerty123!");
  Serial.println(WiFi.softAPIP());

  LittleFS.begin();

  httpd.on("/", handleRacine);
  httpd.on("/mijoteuse", handleMijoteuse);

  httpd.begin();
}

void loop() {
  httpd.handleClient();
}
