#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <PID_v1.h>

double Setpoint, Input, Output;

double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;

PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

ESP8266WebServer httpd(80);
bool MijoteuseOn = true;
float min2Min = 0.0;
float max2Min = 0.0;
float min5Min = 0.0;
float max5Min = 0.0;
int tempsTemp = 0;

double getCurrentTemp() {
  // Read temperature from the sensor and return the value
  int valeur = analogRead(A0);
  
  //Formule pour transformer en degré -> 10000 pour resistance 10k, 3977 pour beta
  double voltage = valeur * 5.0 / 1023.0;
  double resistance = 10000.0 * voltage / (5.0 - voltage);
  double tempKelvin = 1.0 / (1.0 / 298.15 + log(resistance / 10000.0) / 3977); 

  double tempCelcius = tempKelvin - 273.15; 

  return tempCelcius;
}

String getIntensite() {
  return String(0.0);
}

String getTemps() {
  int hr = tempsTemp / 3600;
  int mins = (tempsTemp - hr * 3600) / 60;
  int sec = tempsTemp - hr * 3600 - mins * 60; 
  sec %= 60;
  mins %= 60;
  hr %= 24;

  String hrMinSec = (String(hr) + ":" + String(mins) + ":" + String(sec));
  return hrMinSec;
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
    reponse += "<p>Temperature courante: " + String(getCurrentTemp(), 2) + "</p>";
    reponse += "<p>Temperature 2min: " + String(min2Min, 2) + " " + String(max2Min) + "</p>";
    reponse += "<p>Temperature 5min: " + String(min5Min, 2) + " " + String(max5Min, 2) + "</p>";
    reponse += "<p>Temps de temperature: " + getTemps() + "</p>";
    reponse += "<p>Intensite de chauffage: " + getIntensite() + "</p>";
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

  Input = analogRead(A0);
  Setpoint =  43.00;
  myPID.SetMode(AUTOMATIC);




  Serial.println("Creation de l'AP...");
  WiFi.softAP("WifiTropCool", "Qwerty123!");
  Serial.println(WiFi.softAPIP());

  httpd.on("/", handleRacine);
  httpd.on("/mijoteuse", handleMijoteuse);

  httpd.begin();

  


}

void loop() {


if(MijoteuseOn){
  Input = analogRead(A0);

  double gap = abs(Setpoint - Input);

  if(gap < 10)
    myPID.SetTunings(consKp, consKi, consKd);
  else
    myPID.SetTunings(aggKp, aggKi, aggKd);

  myPID.Compute();
  analogWrite(3,Output);
}


  httpd.handleClient();
}
