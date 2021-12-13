#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiClient.h>

IPAddress local_IP(192, 168, 0, 250);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);

const char* ssid = "Tinkerers' Lab";
const char* password = "tinker@tl";

double OL=0;
double OR=0;
double OF=0;
int S=0;

Servo myservo;

int l1=14;
int l2=12;
int r1=13;
int r2=15;
int f1=4;
int f2=5;
int servo=2;
int pos = 0;

ESP8266WebServer server(80);

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void control(){
  String postBody = server.arg("plain");
  Serial.println(postBody);
  OL = getValue(data,',',0).toDouble();
  OR = getValue(data,',',1).toDouble();
  OF = getValue(data,',',2).toDouble();
  S = getValue(data,',',3).toInt();
  Serial.println(OL);
  Serial.println(OR);
  Serial.println(OF);
  Serial.println(S);
  if(S==0){
    if(OL>0){
      analogWrite(l1,OL);
      analogWrite(l2,0);
    }
    else{
      analogWrite(l1,0);
      analogWrite(l2,-1*OL);
    }
    if(OR>0){
      analogWrite(r1,OR);
      analogWrite(r2,0);
    }
    else{
      analogWrite(r1,0);
      analogWrite(r2,-1*OR);
    }
    if(OF>0){
      analogWrite(r1,OF);
      analogWrite(r2,0);
    }
    else{
      analogWrite(r1,0);
      analogWrite(r2,-1*OF);
    }
  }
  else{
    myservo.write(75);
    delay(1000);
    myservo.write(0);
    delay(1000);
  }
  server.send(200, F("text/html"), F("Done..."));
}

void setup() {
  Serial.begin(115200);

  
  pinMode(l1, OUTPUT);
  pinMode(l2, OUTPUT);
  pinMode(r1, OUTPUT);
  pinMode(r2, OUTPUT);
  pinMode(f1, OUTPUT);
  pinMode(f2, OUTPUT);
  myservo.attach(servo);
  myservo.write(0);
  
  analogWrite(l1, 0);
  analogWrite(l2, 0);
  analogWrite(r1, 0);
  analogWrite(r2, 0);
  analogWrite(f1, 0);
  analogWrite(f2, 0);

  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure");
  }
  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/check", HTTP_GET, [](){ server.send(200, F("text/html"), F("Hii..."));});
  server.on("/", HTTP_POST, control);
  server.begin();
}

void loop() {
  server.handleClient();
}
