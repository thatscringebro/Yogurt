#include <Arduino.h>
#include <map>
#include <ESP8266WiFi.h>
#include <PID_v1.h>
#include <PubSubClient.h>

double Setpoint, Input, Output;

double aggKp = 37, aggKi = 0.5, aggKd = 2;
double consKp = 37, consKi = 0.5, consKd = 2;

PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

const char* mqttServer = "192.168.1.252";
const int mqttPort = 1883;
const char* mqttUser = "pi";
const char* mqttPassword = "pi";

WiFiClient espClient;
PubSubClient client(espClient);

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

void callback(char* topic, byte* payload, unsigned int length) {
  // Fonction appelée lorsqu'un message est reçu sur un topic auquel nous sommes abonnés

  // Convertir le payload en une chaîne de caractères
  payload[length] = '\0'; // Assurez-vous de terminer la chaîne
  String message = String((char*)payload);

  // Vérifier le topic sur lequel le message a été reçu
  if (String(topic) == "yogourt/control") {
    // Gérer le message reçu sur le topic "yogourt/control"
    if (message == "on") {
      // Recommencer à chauffer
      MijoteuseOn = true;
    } else if (message == "off") {
      // Arrêter de chauffer
      MijoteuseOn = false;
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqttUser, mqttPassword)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("yogourt/control");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  Input = analogRead(A0);
  Setpoint = 43.00;
  myPID.SetMode(AUTOMATIC);

  Serial.println("Connecting to WiFi...");
  WiFi.begin("DEPTI_2.4", "2021depTI");

  // Attendre la connexion au réseau
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");

  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long currentTime = millis();
  elapsedTime = currentTime - startTime;
  double temp = getCurrentTemp();
  Input = temp;
  setTemperature(temp);

  if (temp <= 43) {
    double gap = abs(Setpoint - Input);
    if (gap < 10)
      myPID.SetTunings(consKp, consKi, consKd);
    else
      myPID.SetTunings(aggKp, aggKi, aggKd);
    myPID.Compute();
  }

  if (elapsedTime >= 100){
    startTime = currentTime;
    client.publish("yogourt/temp", String(temp).c_str());
  }

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
}