#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

ESP8266WebServer httpd(80);

void HandleRacine(){
  String response = "<html><body>C'EST TROP COOL</body></html>";
  httpd.send(200, "text/html", response);
}

bool MijoteuseOn = true;
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

  httpd.on("/", HandleRacine);
  httpd.on("/mijoteuse", handleMijoteuse);

  httpd.begin();
}

void loop() {
  httpd.handleClient();
}
