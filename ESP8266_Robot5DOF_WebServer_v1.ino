/*
 * Description: Robot 5 DOF. Controlled by a PCA 9685 and with a microcontroller ESP 8266. WebServer in ESP8266 to control the robot through Internet
 * Author: Pierce001
 * Date: August 2018
 * 
 * 
 */

#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <String.h>
#include "sha256.h"
#include "Base64.h"
#include <math.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//Web server parameters
MDNSResponder mdns;
ESP8266WebServer server(80);

//WiFi parameters
const char* ssid = "homenet"; // your connection name
const char* password = "00araq01"; // your connection password
 WiFiClientSecure client;

//5DOF servo positions in degrees
#define HAND_OPEN 100
#define HAND_CLOSE 170
#define HAND_SEMI_OPEN 120
#define HAND_STEP_ANGLE 10

#define WRIST_RIGHT 0  //Right
#define WRIST_LEFT 180
#define WRIST_CENTER 95
#define WRIST_H_STEP_ANGLE 10

#define WRIST_HIGH 20  //Hihgest position
#define WRIST_MID_HIGH 40
#define WRIST_MID 90
#define WRIST_MID_LOW 130 
#define WRIST_LOW 160
#define WRIST_ANGLE_INC 10
#define WRIST_V_STEP_ANGLE 10

#define ELBOW_HIGH 35 // Highest position
#define ELBOW_MID_HIGH 70
#define ELBOW_MID 100
#define ELBOW_LOW_MID 130
#define ELBOW_LOW 140  //Lowest position
#define ELBOW_ANGLE_INC 5
#define ELBOW_STEP_ANGLE 10

#define BASE_LEFT 180
#define BASE_RIGHT 0
#define BASE_CENTER 90
#define BASE_STEP_ANGLE 10 

#define SERVO_MAX 180
#define SERVO_MIN 0
#define SERVO_INIT 90

#define SERVO_DELAY 500 //servo delay between movements

//Other parameters
#define ANGLE_INCREMENT 20

//PCA 9685 parameters
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver();
#define PWM_FREQ 50  // PWM FREQUENCY  50Hz or  T=20ms
#define POS_0 85    // Pulse width for 0 degrees
#define POS_180 455   // Pulse width for 180 degrees
#define SERVO_MIN_ANGLE 0  //DEGREES
#define SERVO_MAX_ANGLE 180 //DEGREES

#define SERVO_BASE_ID 8
#define SERVO_ELBOW_ID 9
#define SERVO_WRIST_VERT_ID 10
#define SERVO_WRIST_HOR_ID 11
#define SERVO_HAND_ID 12

//SERVO TECHNICAL PARAMETERS - FOR DS3218MG AND DS3325MG
#define SPEED_ROT_1DEG 3 //In miliseconds / degree of rotation

// Thingworx settings  
char* TW_APP_KEY = "5aa92502-64bc-4848-8cbb-cae45b4d8da2";    
char* TW_HOST = "217.165.209.8";
const int TW_PORT = 1010;          
char* TW_THING_NAME = "MyRobot5DOF";
char* TW_SERVICE_NAME = "setRobotParamenters";

 //Parameters to keep the status of the servos
 int hand_stat, elbow_stat, wrist_vert_stat, wrist_hor_stat, base_stat;

//START: Functions related to Robot's movements
void setDelayAfterMovement (int rotation){
  delay (SPEED_ROT_1DEG*rotation);
}

//Base Movements
void TurnLeft (void){ moveServo (SERVO_BASE_ID, BASE_LEFT); base_stat = BASE_LEFT; }
void TurnRight (void){ moveServo (SERVO_BASE_ID, BASE_RIGHT); base_stat = BASE_RIGHT; }
void TurnCenter (void){moveServo (SERVO_BASE_ID, BASE_CENTER); base_stat = BASE_CENTER; }
void MoveBaseLeft (int angle){if (angle+base_stat <= BASE_LEFT){base_stat+=angle; moveServo (SERVO_BASE_ID, base_stat);}}
void MoveBaseRight (int angle){if (base_stat-angle >= BASE_RIGHT){base_stat-=angle; moveServo (SERVO_BASE_ID, base_stat);}}

//Elbow Movements
void RaiseElbow (void){ moveServo (SERVO_ELBOW_ID, ELBOW_HIGH); elbow_stat = ELBOW_HIGH;}
void LowElbow (void){ moveServo (SERVO_ELBOW_ID, ELBOW_LOW); elbow_stat = ELBOW_LOW;}
void RaiseHalfElbow (void){ moveServo (SERVO_ELBOW_ID, ELBOW_MID); elbow_stat = ELBOW_MID;}
void MoveElbowUp (int angle){if (elbow_stat-angle >= ELBOW_HIGH){elbow_stat-=angle; moveServo (SERVO_ELBOW_ID, elbow_stat);}}
void MoveElbowDown (int angle){if (elbow_stat+angle <= ELBOW_LOW){elbow_stat+=angle; moveServo (SERVO_ELBOW_ID, elbow_stat);}}
void MoveElbow (int angle){if (angle >= ELBOW_HIGH and angle <= ELBOW_LOW){ moveServo (SERVO_ELBOW_ID, angle); elbow_stat = angle;}}

//Wrist Horizontal Movements Left - Right
void TurnWristLeft (void){ moveServo (SERVO_WRIST_HOR_ID, WRIST_LEFT); wrist_hor_stat = WRIST_LEFT; }
void TurnWristRight (void){ moveServo (SERVO_WRIST_HOR_ID, WRIST_RIGHT); wrist_hor_stat = WRIST_RIGHT;}
void TurnWristCenter (void){ moveServo (SERVO_WRIST_HOR_ID, WRIST_CENTER); wrist_hor_stat = WRIST_CENTER;}
void MoveWristLeft (int angle){if (angle+wrist_hor_stat <= WRIST_LEFT){wrist_hor_stat+=angle; moveServo (SERVO_WRIST_HOR_ID, wrist_hor_stat);}}
void MoveWristRight (int angle){if (wrist_hor_stat-angle >= WRIST_RIGHT){wrist_hor_stat-=angle; moveServo (SERVO_WRIST_HOR_ID, wrist_hor_stat);}}

//Wrist Vertical Movements HIGH - LOW
void RaiseWrist (void){ moveServo (SERVO_WRIST_VERT_ID, WRIST_HIGH); wrist_vert_stat = WRIST_HIGH;}
void LowWrist (void){ moveServo (SERVO_WRIST_VERT_ID, WRIST_LOW); wrist_vert_stat = WRIST_LOW;}
void RaiseHalfWrist (void){ moveServo (SERVO_WRIST_VERT_ID, WRIST_MID); wrist_vert_stat = WRIST_MID;}
void MoveWristUp (int angle){if (wrist_vert_stat-angle >= WRIST_HIGH){wrist_vert_stat-=angle; moveServo (SERVO_WRIST_VERT_ID, wrist_vert_stat);}}
void MoveWristDown (int angle){if (wrist_vert_stat+angle <= WRIST_LOW){wrist_vert_stat+=angle; moveServo (SERVO_WRIST_VERT_ID, wrist_vert_stat);}}
void MoveWrist (int angle){ if (angle >= WRIST_HIGH and angle <= WRIST_LOW){ moveServo (SERVO_WRIST_VERT_ID, angle); wrist_vert_stat = angle;}}

//Hand Movements
void OpenHand (void){  moveServo (SERVO_HAND_ID, HAND_OPEN); hand_stat=HAND_OPEN;}
void CloseHand (void){ moveServo (SERVO_HAND_ID, HAND_CLOSE); hand_stat=HAND_CLOSE;}
void OpenHalfHand (void){ moveServo (SERVO_HAND_ID, HAND_SEMI_OPEN); hand_stat=HAND_SEMI_OPEN;}
void MoveHand2Open (int angle){if (hand_stat-angle >= HAND_OPEN){hand_stat-=angle; moveServo (SERVO_HAND_ID, hand_stat);}}
void MoveHand2Close (int angle){if (hand_stat+angle <= HAND_CLOSE){hand_stat+=angle; moveServo (SERVO_HAND_ID, hand_stat);}}


void moveServo (uint8_t n_servo, int angle) {
  int duty=map(angle,SERVO_MIN_ANGLE,SERVO_MAX_ANGLE,POS_0, POS_180);
  servos.setPWM(n_servo, 0, duty);  
}

void InitRobot (void){
  RaiseElbow ();
  delay (SERVO_DELAY);
  OpenHalfHand ();
  delay (SERVO_DELAY);
  RaiseWrist ();
  delay (SERVO_DELAY);
  TurnWristCenter ();
  delay (SERVO_DELAY);
  TurnCenter ();
  delay (SERVO_DELAY);
}

void Movement_1 (void){
  TurnLeft ();
  delay (SERVO_DELAY);
  OpenHalfHand ();
  delay (SERVO_DELAY);
  //TurnWristCenter ();
  //delay (SERVO_DELAY);
  RaiseWrist ();
  delay (SERVO_DELAY);
  for (int i= WRIST_HIGH; i<= WRIST_MID_LOW; i+=WRIST_ANGLE_INC) {MoveWrist (i); delay (SERVO_DELAY);}
  for (int i = ELBOW_HIGH; i<= ELBOW_MID; i+=ELBOW_ANGLE_INC) {MoveElbow (i); delay (SERVO_DELAY);}
  CloseHand (); delay (SERVO_DELAY);
  RaiseElbow (); delay (SERVO_DELAY);
  //for (int i= WRIST_MID_LOW; i<= WRIST_MID; i-=WRIST_ANGLE_INC) {MoveWrist (i); delay (SERVO_DELAY);}
  //RaiseWrist (); delay (SERVO_DELAY);
  TurnCenter();delay (SERVO_DELAY);
  TurnRight (); delay (SERVO_DELAY);
  //for (int i= WRIST_HIGH; i<= WRIST_MID; i+=WRIST_ANGLE_INC) {MoveWrist (i); delay (SERVO_DELAY);}
  //for (int i = ELBOW_HIGH; i<= ELBOW_MID_HIGH; i+=ELBOW_ANGLE_INC) {MoveElbow (i); delay (SERVO_DELAY);}
  OpenHand (); delay (SERVO_DELAY);
  TurnCenter();delay (SERVO_DELAY);
}

void Movement_2 (void){
  TurnLeft ();
  delay (SERVO_DELAY);
  
  CloseHand ();
  delay (SERVO_DELAY);
  RaiseHalfElbow ();
  delay (SERVO_DELAY);
  RaiseHalfWrist ();
  delay (SERVO_DELAY);
}
//END: Functions related to Robot's movements

//START: FUNCTIONS RELATED TO WEB SERVER
//Check if header is present and correct
bool is_authentified(){
  Serial.println("Enter is authentified");
  if (server.hasHeader("Cookie")){
    Serial.print("Found cookie: ");
    String cookie = server.header("Cookie");
    Serial.println(cookie);
    if (cookie.indexOf("ESPSESSIONID=1") != -1) {
      Serial.println("Authentification Successful");
      return true;
    }
  }
  Serial.println("Authentification Failed");
  return false;
}

//login page, also called for disconnect
void handleLogin(){
  String msg;
  if (server.hasHeader("Cookie")){
    Serial.print("Found cookie: ");
    String cookie = server.header("Cookie");
    Serial.println(cookie);
  }
  if (server.hasArg("DISCONNECT")){
    Serial.println("Disconnection");
    server.sendHeader("Location","/login");
    server.sendHeader("Cache-Control","no-cache");
    server.sendHeader("Set-Cookie","ESPSESSIONID=0");
    server.send(301);
    return;
  }
  if (server.hasArg("USERNAME") && server.hasArg("PASSWORD")){
    if (server.arg("USERNAME") == "admin" &&  server.arg("PASSWORD") == "root" ) // enter ur username and password you want
    {
      server.sendHeader("Location","/");
      server.sendHeader("Cache-Control","no-cache");
      server.sendHeader("Set-Cookie","ESPSESSIONID=1");
      server.send(301);
      Serial.println("Log in Successful");
      return;

      }

  msg = "Wrong username/password! try again.";
  Serial.println("Log in Failed");
  }
  String content = "<html><body style='background-color:MediumAquaMarine'><form action='/login' method='POST'><p  align ='center' style='font-size:300%;'><u><b><i>  Log In  </i></b></u></p><br>";
  content += " <p   align ='center' style='font-size:160%'><b> UserName:<input type='text' name='USERNAME' placeholder='user name' required></b></p><br>";
  content += "<p  align ='center' style='font-size:160%'><b>Password:<input type='password' name='PASSWORD' placeholder='password' required></b></p><br>";
  content += "<p  align ='center' style='font-size:160%'><input type='submit' name='SUBMIT' value='Submit'></form>" + msg + "</p><br> </body></html>";
  server.send(200, "text/html", content);
}

//root page can be accessed only if authentification is ok
void handleRoot(){
  Serial.println("Enter handleRoot");
  String header;
  if (!is_authentified()){
    server.sendHeader("Location","/login");
    server.sendHeader("Cache-Control","no-cache");
    server.send(301);
    return;
  }
  String content =  "<body style='background: #80c6f7'><h1 align ='center'><b><u><i><strong>ROBOT AUTOMATION</strong></i></u></b></h1>";
  content +="<br><p align ='center'>BASE -> <a href=\"base_left\"><button>LEFT</button></a>&nbsp;<a href=\"base_center\"><button>CENTER</button></a>&nbsp;<a href=\"base_right\"><button>RIGHT</button></a>&nbsp;&nbsp;&nbsp;<a href=\"base_left_step\"><button>LEFT 10 DEG</button></a><a href=\"base_right_step\"><button>RIGHT 10 DEG</button></a></p>";
  content += "<p align='center'>ELBOW -> <a href=\"elbow_up\"><button>UP</button></a>&nbsp;<a href=\"elbow_center\"><button>CENTER</button></a>&nbsp;<a href=\"elbow_down\"><button>DOWN</button></a>&nbsp;&nbsp;&nbsp;<a href=\"elbow_step_up\"><button>UP 10 DEG</button></a><a href=\"elbow_step_down\"><button>DOWN 10 DEG</button></a></p>";
  content += "<p align='center'>WRIST VERT -> <a href=\"wrist_up\"><button>UP</button></a>&nbsp;<a href=\"wrist_v_center\"><button>CENTER</button></a>&nbsp;<a href=\"wrist_down\"><button>DOWN</button></a>&nbsp;&nbsp;&nbsp;<a href=\"wrist_step_up\"><button>UP 10 DEG</button></a><a href=\"wrist_step_down\"><button>DOWN 10 DEG</button></a></p>";
  content +="<p align ='center'>WRIST HORIZ -> <a href=\"wrist_left\"><button>LEFT</button></a>&nbsp;<a href=\"wrist_h_center\"><button>CENTER</button></a>&nbsp;<a href=\"wrist_right\"><button>RIGHT</button></a>&nbsp;&nbsp;&nbsp;<a href=\"wrist_left_step\"><button>LEFT 10 DEG</button></a><a href=\"wrist_right_step\"><button>RIGHT 10 DEG</button></a></p>";
  content += "<p  align ='center'>HAND OPEN > <a href=\"hand_open\"><button>OPEN</button></a>&nbsp;<a href=\"hand_semi_open\"><button>SEMI OPEN</button></a>&nbsp;<a href=\"hand_close\"><button>CLOSE</button></a>&nbsp;&nbsp;&nbsp; <a href=\"hand_step_open\"><button>OPEN 10 DEG</button></a>&nbsp; <a href=\"hand_step_close\"><button>CLOSE 10 DEG</button></a></p>";
  content += "<br><p><marquee direction='right'>Developed by   ACME STUDIOS </marquee></p>";
  
   if (server.hasHeader("User-Agent")){
    content += "the user agent used is : " + server.header("User-Agent") + "<br><br>";
    
    
  }
  content += "You can access this page until you <a href=\"/login?DISCONNECT=YES\">disconnect</a></body></html>";
  server.send(200, "text/html", content);
}

//no need authentification
void handleNotFound(){
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i=0; i<server.args(); i++){
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}

//END: FUNCTIONS RELATED TO WEB SERVER

//START: FUNCTIONS RELATED TO THINGWORX

void UpdateDigitalTwinRobotStatus (void){
  UpdateThingworxValues(client,TW_HOST,TW_PORT,TW_APP_KEY,TW_THING_NAME,TW_SERVICE_NAME,base_stat, elbow_stat, wrist_vert_stat, wrist_hor_stat, hand_stat);
}

void UpdateThingworxValues(WiFiClientSecure &client, char *server, int port, char *aKey, char *thing, char *service, int base, int elbow, int wrist_vert, int wrist_hor, int hand)
{
 if (!client.connect(server, 1010)) {
    Serial.println("connection failed to Thingworx");
    return;
 }
 Serial.println("Connected to Thingworx");
//    // send HTTP PUT request to send sensor values one by one to ThingWorx:
//    for(i=0; i<sensCount; i++){
//      String putRequest = "PUT /Thingworx/Things/" + thing + "/Properties/" + sensNames[i] +"?method=put&appKey=" + aKey + "&value=" + String(sensorValue[i]);
//      client.println(putRequest);
//      client.println(" HTTP/1.1");
//      client.print("Host: " + server + ":" + String(port));
//      client.println();
//    }
    
      
    // send the HTTP POST request for services to send all sensor values at once:
    String postString = "POST /Thingworx/Things/" + String(thing) + "/Services/" + String(service) + "?appKey=" + aKey + "&method=post&x-thingworx-session=true<";
    
// +"Content-Type: text/xml"
    postString = postString + "&" + "base" + "=" + base;
    postString = postString + "&" + "elbow" + "=" + elbow;
    postString = postString + "&" + "wrist_vert" + "=" + wrist_vert;
    postString = postString + "&" + "wrist_horiz" + "=" + wrist_hor;
    postString = postString + "&" + "hand" + "=" + hand;
    postString = postString + ">";
    client.println(postString);
    Serial.println(postString);
    
    client.println(" HTTP/1.1");
    Serial.println(" HTTP/1.1");
    client.print("Host: " + String(server) + ":" + String(port) );
    Serial.print ("Host: " + String(server) + ":" + String(port) );
    client.println();
    Serial.println(); 
    client.println("Content-Type: text/html");
    Serial.println ("Content-Type: text/html");
    client.println();
    Serial.println ();
    client.stop();

//// send HTTP GET request to receive values from ThingWorx:
//    String getRequest = "GET /Thingworx/Things/" + String(thing) + "/Properties?method=get&appKey=" + aKey;
//    client.println(getRequest);
//    client.println(" HTTP/1.1");
//    client.print("Host: " + String(server) + ":" + String(port));
//    client.println();
    
//    while (client.available()) {
//      char c = client.read();
//      Serial.print(c);
//    }
  
//  client.stop();
 
}

//START: FUNCTIONS RELATED TO THINGWORX



void setup(void){  
  //Start serial port
  Serial.begin(115200);

  //Start WiFi
  WiFi.begin(ssid, password);
  Serial.println("");

  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  //Init PCA 9685
  servos.begin();  
  servos.setPWMFreq(PWM_FREQ); 

  //Setup initial position of the robot
  InitRobot ();

  //Update Thingworx
  UpdateDigitalTwinRobotStatus ();
  
  //Start Web Server
  if (mdns.begin("esp8266", WiFi.localIP())) {
    Serial.println("MDNS responder started");
  }
  server.on("/", handleRoot);
  server.on("/login", handleLogin);
  server.on("/inline", [](){server.send(200, "text/plain", "this works without need of authentification");});
  server.onNotFound(handleNotFound);

  //here the list of headers to be recorded
  const char * headerkeys[] = {"User-Agent","Cookie"} ;
  size_t headerkeyssize = sizeof(headerkeys)/sizeof(char*);
  //ask server to track these headers
  server.collectHeaders(headerkeys, headerkeyssize );

  //Web Server handling Base Movements
  server.on("/",[](){});
  server.on("/base_left", [](){TurnLeft (); UpdateDigitalTwinRobotStatus (); delay(1000); handleRoot ();});
  server.on("/base_center", [](){TurnCenter ();UpdateDigitalTwinRobotStatus (); delay(1000); handleRoot ();});
  server.on("/base_right", [](){ TurnRight();UpdateDigitalTwinRobotStatus (); delay(1000); handleRoot ();});
  server.on("/base_left_step", [](){ MoveBaseLeft(BASE_STEP_ANGLE);UpdateDigitalTwinRobotStatus (); delay(1000); handleRoot ();});
  server.on("/base_right_step", [](){ MoveBaseRight(BASE_STEP_ANGLE);UpdateDigitalTwinRobotStatus (); delay(1000); handleRoot ();});

  //Web Server handling Elbow Movements
  server.on("/elbow_up", [](){ RaiseElbow(); UpdateDigitalTwinRobotStatus (); delay(1000);handleRoot ();});
  server.on("/elbow_down", [](){LowElbow(); UpdateDigitalTwinRobotStatus (); delay(1000);handleRoot ();});
  server.on("/elbow_center", [](){RaiseHalfElbow(); UpdateDigitalTwinRobotStatus (); delay(1000); handleRoot ();});
  server.on("/elbow_step_up", [](){MoveElbowUp(ELBOW_STEP_ANGLE); UpdateDigitalTwinRobotStatus (); delay(1000); handleRoot ();});
  server.on("/elbow_step_down", [](){MoveElbowDown(ELBOW_STEP_ANGLE); UpdateDigitalTwinRobotStatus (); delay(1000); handleRoot ();});

  //Web Server handling Wrist Vertical Movements
  server.on("/wrist_up", [](){ RaiseWrist(); UpdateDigitalTwinRobotStatus (); delay(1000);handleRoot ();});
  server.on("/wrist_down", [](){LowWrist(); UpdateDigitalTwinRobotStatus (); delay(1000);handleRoot ();});
  server.on("/wrist_v_center", [](){RaiseHalfWrist(); UpdateDigitalTwinRobotStatus (); delay(1000); handleRoot ();});
  server.on("/wrist_step_up", [](){MoveWristUp(WRIST_V_STEP_ANGLE); UpdateDigitalTwinRobotStatus (); delay(1000); handleRoot ();});
  server.on("/wrist_step_down", [](){MoveWristDown(WRIST_V_STEP_ANGLE); UpdateDigitalTwinRobotStatus (); delay(1000); handleRoot ();});

  //Web Server handling Wrist Horizontal Movements
  server.on("/wrist_left", [](){TurnWristLeft (); UpdateDigitalTwinRobotStatus (); delay(1000); handleRoot ();});
  server.on("/wrist_h_center", [](){TurnWristCenter ();UpdateDigitalTwinRobotStatus (); delay(1000); handleRoot ();});
  server.on("/wrist_right", [](){ TurnWristRight();UpdateDigitalTwinRobotStatus (); delay(1000); handleRoot ();});
  server.on("/wrist_left_step", [](){ MoveWristLeft(WRIST_H_STEP_ANGLE);UpdateDigitalTwinRobotStatus (); delay(1000); handleRoot ();});
  server.on("/wrist_right_step", [](){ MoveWristRight(WRIST_H_STEP_ANGLE);UpdateDigitalTwinRobotStatus (); delay(1000); handleRoot ();});

 //Web Server handling Hand Movements
  server.on("/hand_open", [](){OpenHand (); UpdateDigitalTwinRobotStatus (); delay(1000); handleRoot ();});
  server.on("/hand_semi_open", [](){OpenHalfHand ();UpdateDigitalTwinRobotStatus (); delay(1000); handleRoot ();});
  server.on("/hand_close", [](){ CloseHand();UpdateDigitalTwinRobotStatus (); delay(1000); handleRoot ();});
  server.on("/hand_step_open", [](){ MoveHand2Open(HAND_STEP_ANGLE);UpdateDigitalTwinRobotStatus (); delay(1000); handleRoot ();});
  server.on("/hand_step_close", [](){ MoveHand2Close(HAND_STEP_ANGLE);UpdateDigitalTwinRobotStatus (); delay(1000); handleRoot ();});

  server.begin();
  Serial.println("HTTP server started");
}

void loop(void){
  server.handleClient();
}
