#include <Arduino.h>
#include <map>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <PID_v1.h>

double Setpoint, Input, Output;

double aggKp=37, aggKi=0.5, aggKd=2;
double consKp=37, consKi=0.5, consKd=2;

PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

ESP8266WebServer httpd(80);
bool MijoteuseOn = false;
int tempsInital = 0;
unsigned int compteur = 0;

long tempsDebut = 0.0;
long tempsCourant = 0.0;
long tempsPasse = 0.0;

long elapsedTime = 0;
long startTime = 0;

long tempsChauffage;
long tempsMaxChauffage; 
long tempsDebutChauffage;


std::map<int, double> arrTemp5min;
std::map<int, double> arrTemp2min;


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

float min2Min = getCurrentTemp();
float max2Min = getCurrentTemp();
float min5Min = getCurrentTemp();
float max5Min = getCurrentTemp();

void setMinMax() {
  max2Min = arrTemp2min.begin()->second;
  min2Min = arrTemp2min.begin()->second;
  max5Min = arrTemp5min.begin()->second;
  min5Min = arrTemp5min.begin()->second;

  for (const auto& pair : arrTemp2min) {
    // pour 2min
    if (pair.second > max2Min)
      max2Min = pair.second;
    if (pair.second < min2Min)
      min2Min = pair.second;
  }

  for (const auto& pair : arrTemp5min) {
    // pour 5min
    if (pair.second > max5Min)
      max5Min = pair.second;
    if (pair.second < min5Min)
      min5Min = pair.second;
  }
}

void setTemperature(double Temperature){
  tempsCourant = millis()/1000;
  tempsPasse = tempsCourant - tempsDebut;

  // Ajoute nouvelle temperature
  arrTemp5min[millis()/1000] = Temperature;
  arrTemp2min[millis()/1000] = Temperature;
  Serial.println("valeur ajouter a l'array : " + String(Temperature, 2));
  // Si plus que 2min, on le retire du map
  if (arrTemp2min.begin()->first < millis()/1000 - 120)
    arrTemp2min.erase(arrTemp2min.begin());
  // Si plus que 5min, on le retire du map
  if (arrTemp5min.begin()->first < millis()/1000 - 300)
    arrTemp5min.erase(arrTemp5min.begin());

  // set les valeurs min max si c'est la première température
  if (arrTemp2min.size() == 1) {
    min2Min = Temperature;
    max2Min = Temperature;
  }
  if (arrTemp5min.size() == 1){
    min5Min = Temperature;
    max5Min = Temperature;
  }

  setMinMax();
  tempsDebut = tempsCourant;
}


String getIntensite() {
  return String((Output * 100) / 255);
}

String getTemps() {
  String hrMinSec = "La machine a yogourt n'est pas entre 41 et 45 celcius";
  if(tempsInital != 0) {
    int tempsTemp = (millis() - tempsInital) / 1000;
    int hr = tempsTemp / 3600;
    int mins = (tempsTemp - hr * 3600) / 60;
    int sec = tempsTemp - hr * 3600 - mins * 60; 
    sec %= 60;
    mins %= 60;
    hr %= 24;
  
    hrMinSec = (String(hr) + ":" + String(mins) + ":" + String(sec));
  }
  
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
    reponse += "<p>Temperature 2min: " + String(min2Min, 2) + " - " + String(max2Min) + "</p>";
    reponse += "<p>Temperature 5min: " + String(min5Min, 2) + " - " + String(max5Min, 2) + "</p>";
    reponse += "<p>Temps de temperature: " + getTemps() + "</p>";
    reponse += "<p>Intensite de chauffage: " + getIntensite() + "%</p>";
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
  pinMode(D1, OUTPUT);

  Serial.println("Creation de l'AP...");
  WiFi.softAP("WifiTropCool", "Qwerty123!");
  Serial.println(WiFi.softAPIP());

  httpd.on("/", handleRacine);
  httpd.on("/mijoteuse", handleMijoteuse);

  httpd.begin();

}

void loop() {
  long currentTime = millis();
  elapsedTime = currentTime - startTime;
  double temp = 0;
  if (elapsedTime >= 100)
  {
    Serial.println("chauffing");
    temp = getCurrentTemp();
    Input = temp;
    setTemperature(temp);
    Serial.println("température min (2minutes) : " + String(min2Min, 2));
    Serial.println("température max (2minutes) : " + String(max2Min, 2));
    Serial.println("température min (5minutes) : " + String(min5Min, 2));
    Serial.println("température max (5minutes) : " + String(max5Min, 2));

  if(temp < 41 || temp > 45){
    tempsInital = 0;
    Serial.println("temps initiale: " + String(tempsInital));
    Serial.println("temperature: " + String(temp));
  }
  else if(tempsInital == 0)
    tempsInital = millis();
  }

  if(temp <= 43) {
    double gap = abs(Setpoint - Input);
   if(gap < 10)
      myPID.SetTunings(consKp, consKi, consKd);
    else
      myPID.SetTunings(aggKp, aggKi, aggKd);
   myPID.Compute();
  }
  if(elapsedTime >= 100)
    startTime = currentTime;

  unsigned long tempsActuel = millis();
  int pourcentageChauffage = (Output * 100) / 255;

  if (tempsActuel - tempsDebutChauffage < pourcentageChauffage && MijoteuseOn) {
    digitalWrite(D1, HIGH);
  } else {
    digitalWrite(D1, LOW);

    if (tempsActuel - tempsDebutChauffage >= 100) {
      tempsDebutChauffage = millis();
    }
  }
  httpd.handleClient();
}
