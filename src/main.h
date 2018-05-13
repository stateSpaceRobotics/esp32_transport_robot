#include <PID_v1.h>
#include <Arduino.h>

// WiFi and OTA libraries
#include <WiFi.h> // ESP32 WiFi drivers
#include <ESPmDNS.h> // ESP32 mDNS support
#include <WiFiUdp.h> // ESP32 WiFi UDP
#include <ArduinoOTA.h> // Arduino OTA updates
#include <Update.h> // Needed for OTA

// Identify which core has the Arduino stuff
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

// Function declarations. Definitions are at the bottom for readability
// Inlined to encourage compiler optimization
void WiFi_loop(void * parameter);

// Global variables
// Set ssid and password here
const char* ssid = "NASA RMC Test Network";
const char* password = "StateSpaceRobotics";
const char* hostname = "transport_robot";

// enable or disable X4 mode for encoders. Doubles interrupt frequency
#define X4

// encoder pins:
// A, B
// 32,33
// 25, 26
// 27, 14
// 12, 13
#define FR_A 32//12
#define FR_B 33//13

#define BR_A 25//27
#define BR_B 26//14

#define FL_A 27//25
#define FL_B 14//26

#define BL_A 12//32
#define BL_B 13//33

// PWM/direction pins
// 15, 2, 0, 4, 16, 17, 5, 18, 19

#define FR_PWM_PIN 4//19
#define RIGHT_EN1 0//18
#define RIGHT_EN2 2//5
#define BR_PWM_PIN 15//17

#define BL_PWM_PIN 19//4
#define LEFT_EN1 18//0
#define LEFT_EN2 5//2
#define FL_PWM_PIN 17//15

#define FL_PWM_CH 1
#define FR_PWM_CH 2
#define BL_PWM_CH 3
#define BR_PWM_CH 4

#ifdef X4
  const double ENC_PER_REV = 12*64*4; // Cycles per revolution
#else
  const double ENC_PER_REV = 12*64*2; // Cycles per revolution
#endif

// arguably not needed for angular velocity control
const double WHEEL_RAD = 0.11771 / 2; // in m

// UDP
const int udp_setpoint_port = 3233;
const int udp_status_port = 3234;

WiFiUDP setpoint_server;
WiFiUDP status_server;

// mutexes
SemaphoreHandle_t velocity_mutex;
portMUX_TYPE FL_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE FR_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE BL_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE BR_mux = portMUX_INITIALIZER_UNLOCKED;

volatile int32_t front_left_count = 0;  //Interrupt counters
volatile int32_t front_right_count = 0; //------------------
volatile int32_t back_left_count = 0;   //------------------
volatile int32_t back_right_count = 0;  //------------------

double front_left_velocity = 0;   //Measured Velocities
double front_right_velocity = 0;  //-------------------
double back_left_velocity = 0;    //-------------------
double back_right_velocity = 0;   //-------------------

double front_left_pwm = 0;   //PWM Values
double front_right_pwm = 0;  //----------
double back_left_pwm = 0;    //----------
double back_right_pwm = 0;   //----------

double front_left_setpoint = 0.00;   //Speed setpoints
double front_right_setpoint = 0.00;  //---------------
double back_left_setpoint = 0.00;    //---------------
double back_right_setpoint = 0.00;   //---------------

#define KP 60
#define KI 80
#define KD 0


double front_left_kp = KP;
double front_left_ki = KI;
double front_left_kd = KD;

double front_right_kp = KP;
double front_right_ki = KI;
double front_right_kd = KD;

double back_left_kp = KP;
double back_left_ki = KI;
double back_left_kd = KD;

double back_right_kp = KP;
double back_right_ki = KI;
double back_right_kd = KD;

// PID controllers
PID front_left_PID(&front_left_velocity, &front_left_pwm, &front_left_setpoint, front_left_kp, front_left_ki, front_left_kd, DIRECT);
PID front_right_PID(&front_right_velocity, &front_right_pwm, &front_right_setpoint, front_right_kp, front_right_ki, front_right_kd, DIRECT);
PID back_left_PID(&back_left_velocity, &back_left_pwm, &back_left_setpoint, back_left_kp, back_left_ki, back_left_kd, DIRECT);
PID back_right_PID(&back_right_velocity, &back_right_pwm, &back_right_setpoint, back_right_kp, back_right_ki, back_right_kd, DIRECT);



void inline start_WiFi() {
  // Start up wifi and connect
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Restart until successful connect
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  // Print the IP
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("BAM!!");
}

void inline start_OTA() {
  // Port for OTA server
  ArduinoOTA.setPort(3232);

  // Hostname for mDNS resolving
  ArduinoOTA.setHostname(hostname);

  // Password for performing OTA update
  ArduinoOTA.setPassword("OTApass");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  // Setup handler functions
  ArduinoOTA
    // Runs this function on start
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    // Runs this function on end
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    // Runs this function during update
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    // Runs this if there's an error
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  // Starts the OTA server
  ArduinoOTA.begin();

}

void IRAM_ATTR FL_A_interrupt(){
  static volatile int enc;
  enc = digitalRead(FL_A) ^ digitalRead(FL_B);
  portENTER_CRITICAL_ISR(&FL_mux);
  if(enc == 0b1){
    front_left_count++;
  }
  if(enc == 0b0){
    front_left_count--;
  }
  portEXIT_CRITICAL_ISR(&FL_mux);
}
void IRAM_ATTR FR_A_interrupt(){
  static volatile int enc;
  enc = digitalRead(FR_A) ^ digitalRead(FR_B);
  portENTER_CRITICAL_ISR(&FR_mux);
  if(enc == 0b1){
    front_right_count++;
  }
  if(enc == 0b0){
    front_right_count--;
  }
  portEXIT_CRITICAL_ISR(&FR_mux);
}
void IRAM_ATTR BL_A_interrupt(){
  static volatile int enc;
  enc = digitalRead(BL_A) ^ digitalRead(BL_B);
  portENTER_CRITICAL_ISR(&BL_mux);
  if(enc == 0b1){
    back_left_count++;
  }
  if(enc == 0b0){
    back_left_count--;
  }
  portEXIT_CRITICAL_ISR(&BL_mux);
}
void IRAM_ATTR BR_A_interrupt(){
  static volatile int enc;
  enc = digitalRead(BR_A) ^ digitalRead(BR_B);
  portENTER_CRITICAL_ISR(&BR_mux);
  if(enc == 0b1){
    back_right_count++;
  }
  if(enc == 0b0){
    back_right_count--;
  }
  portEXIT_CRITICAL_ISR(&BR_mux);
}

// Only declare and define if X4 mode is desired
#ifdef X4
void IRAM_ATTR FL_B_interrupt(){
  static volatile int enc;
  enc = digitalRead(FL_A) ^ digitalRead(FL_B);
  portENTER_CRITICAL_ISR(&FL_mux);
  if(enc == 0b0){
    front_left_count++;
  }
  if(enc == 0b1){
    front_left_count--;
  }
  portEXIT_CRITICAL_ISR(&FL_mux);
}
void IRAM_ATTR FR_B_interrupt(){
  static volatile int enc;
  enc = digitalRead(FR_A) ^ digitalRead(FR_B);
  portENTER_CRITICAL_ISR(&FR_mux);
  if(enc == 0b0){
    front_right_count++;
  }
  if(enc == 0b1){
    front_right_count--;
  }
  portEXIT_CRITICAL_ISR(&FR_mux);
}
void IRAM_ATTR BL_B_interrupt(){
  static volatile int enc;
  enc = digitalRead(BL_A) ^ digitalRead(BL_B);
  portENTER_CRITICAL_ISR(&BL_mux);
  if(enc == 0b0){
    back_left_count++;
  }
  if(enc == 0b1){
    back_left_count--;
  }
  portEXIT_CRITICAL_ISR(&BL_mux);
}
void IRAM_ATTR BR_B_interrupt(){
  static volatile int enc;
  enc = digitalRead(BR_A) ^ digitalRead(BR_B);
  portENTER_CRITICAL_ISR(&BR_mux);
  if(enc == 0b0){
    back_right_count++;
  }
  if(enc == 0b1){
    back_right_count--;
  }
  portEXIT_CRITICAL_ISR(&BR_mux);
}
#endif

void pinInit(uint32_t wait_time){
  // Initialize and disable direction pins
  pinMode(LEFT_EN1, OUTPUT);
  pinMode(LEFT_EN2, OUTPUT);
  pinMode(RIGHT_EN1, OUTPUT);
  pinMode(RIGHT_EN2, OUTPUT);

  digitalWrite(LEFT_EN1, LOW);
  digitalWrite(LEFT_EN2, LOW);
  digitalWrite(RIGHT_EN1, LOW);
  digitalWrite(RIGHT_EN2, LOW);

  // initialize pwm pins/channels
  ledcSetup(FL_PWM_CH, 20000, 8);
  ledcAttachPin(FL_PWM_PIN, FL_PWM_CH);
  ledcSetup(FR_PWM_CH, 20000, 8);
  ledcAttachPin(FR_PWM_PIN, FR_PWM_CH);
  ledcSetup(BL_PWM_CH, 20000, 8);
  ledcAttachPin(BL_PWM_PIN, BL_PWM_CH);
  ledcSetup(BR_PWM_CH, 20000, 8);
  ledcAttachPin(BR_PWM_PIN, BR_PWM_CH);

  ledcWrite(FL_PWM_CH, 0);
  ledcWrite(FR_PWM_CH, 0);
  ledcWrite(BR_PWM_CH, 0);
  ledcWrite(BL_PWM_CH, 0);
  delay(wait_time);

  // initialize encoder pins and their interrupts
  pinMode(FL_A, INPUT);
  pinMode(FL_B, INPUT);
  pinMode(FR_A, INPUT);
  pinMode(FR_B, INPUT);
  pinMode(BR_A, INPUT);
  pinMode(BR_B, INPUT);
  pinMode(BL_A, INPUT);
  pinMode(BL_B, INPUT);

  attachInterrupt(FL_A, FL_A_interrupt, CHANGE);
  attachInterrupt(FR_A, FR_A_interrupt, CHANGE);
  attachInterrupt(BL_A, BL_A_interrupt, CHANGE);
  attachInterrupt(BR_A, BR_A_interrupt, CHANGE);

  #ifdef X4
    attachInterrupt(FL_A, FL_B_interrupt, CHANGE);
    attachInterrupt(FR_A, FR_B_interrupt, CHANGE);
    attachInterrupt(BL_A, BL_B_interrupt, CHANGE);
    attachInterrupt(BR_A, BR_B_interrupt, CHANGE);
  #endif
}


void updateVel(){
  static long then=0, now;
  static double del_time = 0;
  static int32_t FL_enc, FR_enc, BL_enc, BR_enc;
  static int32_t prev_FL, prev_FR, prev_BL, prev_BR;
  static double enc_per_sec_to_rad_per_sec = 2.0*3.14 / ENC_PER_REV;//constant

  now = millis();
  del_time = ((double) (now - then)) / 1000.0;
  then = now;

  // read encoders
  portENTER_CRITICAL(&FL_mux);
  FL_enc = front_left_count;
  portEXIT_CRITICAL(&FL_mux);
  portENTER_CRITICAL(&FR_mux);
  FR_enc = front_right_count;
  portEXIT_CRITICAL(&FR_mux);
  portENTER_CRITICAL(&BL_mux);
  BL_enc = back_left_count;
  portEXIT_CRITICAL(&BL_mux);
  portENTER_CRITICAL(&BR_mux);
  BR_enc = back_right_count;
  portEXIT_CRITICAL(&BR_mux);

  //Calculate the measured velocity by (interrupt count/ sample time) * a constant value defined earlier.
  xSemaphoreTake(velocity_mutex, portMAX_DELAY);
  front_left_velocity = ((double) (FL_enc - prev_FL)) / del_time * enc_per_sec_to_rad_per_sec;
  front_right_velocity = ((double) (FR_enc - prev_FR)) / del_time * enc_per_sec_to_rad_per_sec;
  back_left_velocity = ((double) (BL_enc - prev_BL)) / del_time * enc_per_sec_to_rad_per_sec;
  back_right_velocity = ((double) (BR_enc - prev_BR)) / del_time * enc_per_sec_to_rad_per_sec;
  xSemaphoreGive(velocity_mutex);

  prev_FL = FL_enc;
  prev_FR = FR_enc;
  prev_BL = BL_enc;
  prev_BR = BR_enc;
}

void do_PWM() {
  // left side
  double sgn_left = 0;
  if (abs(front_left_pwm) > abs(back_left_pwm)) {
    sgn_left = (front_left_pwm > 0) ? 1 : -1;
  } else {
    sgn_left = (back_left_pwm > 0) ? 1 : -1;
  }
  // right side
  double sgn_right = 0;
  if (abs(front_right_pwm) > abs(back_right_pwm)) {
    sgn_right = (front_right_pwm > 0) ? 1 : -1;
  } else {
    sgn_right = (back_right_pwm > 0) ? 1 : -1;
  }

  // set direction and PWM for left
  if (sgn_left < 0) {
    digitalWrite(LEFT_EN1, HIGH);
    digitalWrite(LEFT_EN2, LOW);
  } else {
    digitalWrite(LEFT_EN1, LOW);
    digitalWrite(LEFT_EN2, HIGH);
  }
  // multiplication ensures positive pwm is written
  // additionally, zeros out wheels that oppose
  ledcWrite(FL_PWM_CH, (abs(front_left_setpoint) > 0.25 ) ? (sgn_left*front_left_pwm * (sgn_left*front_left_pwm > 0) ) : 0 );
  ledcWrite(BL_PWM_CH, (abs(back_left_setpoint) > 0.25 ) ? (sgn_left*back_left_pwm * (sgn_left*back_left_pwm > 0) ) : 0 );

  // set direction and PWM for left
  if (sgn_right < 0) {
    digitalWrite(RIGHT_EN1, HIGH);
    digitalWrite(RIGHT_EN2, LOW);
  } else {
    digitalWrite(RIGHT_EN1, LOW);
    digitalWrite(RIGHT_EN2, HIGH);
  }
  ledcWrite(FR_PWM_CH, (abs(front_right_setpoint) > 0.25 ) ? (sgn_right*front_right_pwm * (sgn_right*front_right_pwm > 0) ) : 0 );
  ledcWrite(BR_PWM_CH, (abs(back_right_setpoint) > 0.25 ) ? (sgn_right*back_right_pwm * (sgn_right*back_right_pwm > 0) ) : 0 );
}

void inline setup_UDP() {
  setpoint_server.begin(udp_setpoint_port);
  setpoint_server.flush();
  status_server.begin(udp_status_port);
  status_server.flush();
}

void inline do_UDP() {
  static char buf[1024] = {'\0'};
  static int read_status = 0;
  static int packet_size;
  // start setpoint server processing
  packet_size = setpoint_server.parsePacket();
  if ((packet_size < 1024) && (packet_size > 0)){
    read_status = setpoint_server.read(buf, 1024);
    buf[packet_size] = '\0';
    // echo as ack
    setpoint_server.beginPacket(setpoint_server.remoteIP(), setpoint_server.remotePort());
    setpoint_server.printf(buf);
    setpoint_server.endPacket();
    setpoint_server.flush();
    // parse the packet
    // Values to extract
    int index = 0;
    double setpoint = 0;
    // intermediate character processing
    char temp_buf[1024];
    int idx = 0;
    // go through whole packet, looking for index,setpoint pairs
    for(int c = 0; c < packet_size; c++){
      // look for index
      while ((buf[c] != ',') && (c<packet_size)) {
        temp_buf[idx++] = buf[c++];
      }
      temp_buf[idx] = '\0';
      index = atoi(temp_buf);
      idx = 0;
      // look for setpoint
      while ((buf[++c] != '\n') && (c<packet_size)) {
        temp_buf[idx++] = buf[c];
      }
      temp_buf[idx] = '\0';
      setpoint = atof(temp_buf);
      idx = 0;
      switch(index){
        case 0:
          front_left_setpoint = setpoint;
          break;
        case 1:
          front_right_setpoint = setpoint;
          break;
        case 2:
          back_right_setpoint = setpoint;
          break;
        case 3:
          back_left_setpoint = setpoint;
          break;
        default:
          // TODO: error packet here
          break;
      }
    }
  } // end setpoint server processing
  // start status server processing
  packet_size = status_server.parsePacket();
  if ((packet_size < 1024) && (packet_size > 0)){
    read_status = status_server.read(buf, 1024);
    buf[packet_size] = '\0';
    if (buf[0] == '?') {
      status_server.beginPacket(status_server.remoteIP(), status_server.remotePort());
      // pwm (effort), velocity, position
      status_server.printf("0,%lf,%lf,%li\n", front_left_pwm, front_left_velocity, front_left_count);
      status_server.printf("1,%lf,%lf,%li\n", front_right_pwm, front_right_velocity, front_right_count);
      status_server.printf("2,%lf,%lf,%li\n", back_right_pwm, back_right_velocity, back_right_count);
      status_server.printf("3,%lf,%lf,%li\n", back_left_pwm, back_left_velocity, back_left_count);
      status_server.endPacket();
      status_server.flush();
    } else {
      // echo as ack for bad packet
      status_server.beginPacket(status_server.remoteIP(), status_server.remotePort());
      status_server.printf(buf);
      status_server.endPacket();
      status_server.flush();
    }
  }
  // end status server processing
}
