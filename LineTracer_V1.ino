#include "chassis.h"
#include "config.h"

#include <QTRSensors.h>
#include <EEPROM.h>

QTRSensorsRC qtrrc((unsigned char[]) {A0, A1, A2, A3, A4, A5},
  NUM_SENSORS, TIMEOUT);

unsigned int qtrValues[NUM_SENSORS];



pid_config cross = {
  .p = 1.0,
  .i = 0,
  .d = 0.01,
  .i_max = 1
};

void setup() {
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  
  //chassis_init(spd, cross);
  motors_init();
  enc_init();
  
  Serial.begin(9600);
  delay(100);

  //return; 
  // calibrate sensors

  // check if we need to calibrate
  if (HIGH == LOW) {
    delay(500);
    digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
    for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
    {
      qtrrc.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
    }
    digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

    // save data to EEPROM
    uint8_t *minimum = (uint8_t *) qtrrc.calibratedMinimumOn;
    uint8_t *maximum = (uint8_t *) qtrrc.calibratedMaximumOn;
    
    for (int i = 0; i < 2 * NUM_SENSORS; i++) {
      EEPROM.update(i, minimum[i]);
      EEPROM.update(i + 2 * NUM_SENSORS, maximum[i]);
    }

  } else {
    // read calibrated values
    static uint8_t minValues[NUM_SENSORS * sizeof(unsigned int)];
    static uint8_t maxValues[NUM_SENSORS * sizeof(unsigned int)];
    
    qtrrc.calibratedMinimumOn = (unsigned int *) minValues;
    qtrrc.calibratedMaximumOn = (unsigned int *) maxValues;

    for (int i = 0; i < 2 * NUM_SENSORS; i++) {
      minValues[i] = EEPROM.read(i);
      maxValues[i] = EEPROM.read(i + 2 * NUM_SENSORS);
    }
  }

  motors_write(0, 0);
  enc_reset();
}

float p_coef = 0.05;
float d_coef = 0.0005;

int prev_error = 0;

int base_speed = 160;

pid_config spd_config = {
  .p = 3.0,
  .i = 0.04,
  .d = -0.01,
  .i_max = 4000
};

pid_config path_config = {
  .p = 3.5,
  .i = 0.0,
  .d = 0.13,
  .i_max = 1
};

pid_state path_state;
pid_state spd_state;

long timer = 0;

void loop() {
  timer = millis();

  static int val = 10;
  if (Serial.available() > 0) {
    val = (unsigned char) Serial.read();
  }

  Serial.print(enc_getPath(LEFT));
  Serial.print(' ');
  Serial.print(enc_getPath(RIGHT));

  static int target = 0;

  static int count = 0;
  if (count == 51) {
    count = 0;
    target++;
  }

  count++;

  
    long l_error = enc_getPath(LEFT) - target;
    long r_error = enc_getPath(RIGHT) - target;

    motors_write(constrain(r_error * 50, -255, 255), constrain(l_error * 50, -255, 255));
    
  delay(1);

  Serial.println();
  return;
  

  long path_err = pid_step(enc_getPath(RIGHT) - enc_getPath(LEFT), 10, path_config, path_state);

  long spd = (enc_getSpeed(LEFT) + enc_getSpeed(RIGHT)) / 2;
  long spd_err = pid_step(15 - spd, 5, spd_config, spd_state);

  val = spd_err;

  Serial.print(' ');
  Serial.print(path_err);

  motors_write(constrain(val - path_err, 0, 255), constrain(val + path_err, 0, 255));

  // delay for 5 ms just to keep on track in time
  while ((millis() - timer) < 5);;;
  
  timer = millis() - timer;

  Serial.print(' ');
  Serial.print(timer);

  Serial.println("");
  return;

  int pos = qtrrc.readLine(qtrValues);

  int error = pos - 3000;

  int p = error * p_coef;
  int d = (error - prev_error) * d_coef;
  prev_error = error;

  int sum = p - d;

  //motors_write(base_speed + sum, base_speed - sum);

  //Serial.println(pos);
  
  //chassis_write(255, 255);
}
