#include <Arduino.h>
#include <main.h>


void setup() {
  Serial.begin(115200);
  Serial.println("Booted. Waiting 2 seconds to start WiFi");

  velocity_mutex = xSemaphoreCreateMutex();

  // delay to allow power caps to charge and stop motors
  pinInit(2000);

  // set positive/negative pwm to account for direction changing
  front_left_PID.SetOutputLimits(-255,255);
  front_right_PID.SetOutputLimits(-255,255);
  back_left_PID.SetOutputLimits(-255,255);
  back_right_PID.SetOutputLimits(-255,255);
  // set to 10 ms (100 Hz) sample time
  front_left_PID.SetSampleTime(50);
  front_right_PID.SetSampleTime(50);
  back_left_PID.SetSampleTime(50);
  back_right_PID.SetSampleTime(50);

  front_left_PID.SetMode(AUTOMATIC);
  front_right_PID.SetMode(AUTOMATIC);
  back_left_PID.SetMode(AUTOMATIC);
  back_right_PID.SetMode(AUTOMATIC);

  // Spin up task on WiFi core for actual PID
  start_WiFi();
  start_OTA();
  setup_UDP();

  xTaskCreatePinnedToCore(
    WiFi_loop, // Task function
    "WiFi_loop", // Task name
    10000, // Stack size for task (in words)
    NULL,  // Parameter passed as input
    2,     // Task priority
    NULL,  // Task handle
    0 // Run on Core 0
  );
}

void loop() {
  static unsigned long start;
  start = millis();

  updateVel();     // Update Velocity

  delay( 50 - (millis()-start) );
}

// WiFi and high level async task loop
// This loop shares Core 0 with the WiFi handling and contains all processes
//    which are not time-critical, leaving Core 1 for synchronous tasks
//    with strict timing requirements.
void WiFi_loop(void * parameter) {
  while (1){
    static unsigned long start;
    static int serial_count = 0;
    start = millis();
    if (serial_count) {
      serial_count = (serial_count+1)%10;
    } else {
      serial_count++;
      Serial.printf("0,%lf,%lf,%li,%lf\n", front_left_pwm, front_left_velocity, front_left_count, front_left_setpoint);
      Serial.printf("1,%lf,%lf,%li,%lf\n", front_right_pwm, front_right_velocity, front_right_count, front_right_setpoint);
      Serial.printf("2,%lf,%lf,%li,%lf\n", back_right_pwm, back_right_velocity, back_right_count, back_right_setpoint);
      Serial.printf("3,%lf,%lf,%li,%lf\n\n", back_left_pwm, back_left_velocity, back_left_count, back_left_setpoint);
    }

    // Handles OTA updates
    ArduinoOTA.handle();

    // PID compute
    xSemaphoreTake(velocity_mutex, portMAX_DELAY);
    front_left_PID.Compute();
    front_right_PID.Compute();
    back_left_PID.Compute();
    back_right_PID.Compute();
    xSemaphoreGive(velocity_mutex);

    // PWM write
    do_PWM();

    // UDP stuff
    do_UDP();


    yield(); // yield to let the WiFi drivers do their thing
    delay( 50 - (millis()-start) );
  }
}
