#include <Adafruit_LSM6DSO32.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h> 
//added for SD logging
#include <SPI.h>
#include <SD.h>

Adafruit_LSM6DSO32 dso32;

Servo esc;
const int ESC_PIN = PA0;
const int LED_PIN = PB13; 
const int ESC_REVERSE_MAX = 1000;
const int ESC_NEUTRAL     = 1500;
const int ESC_FORWARD_MAX = 2000;

// ===== ADDED FOR SD LOGGING =====
// This is the chip-select pin for the SD card module.
// IMPORTANT: change PA4 if your SD module CS pin is wired somewhere else.
const int SD_CS_PIN = PA4;

// This is the file object used to write data into telemetry.csv on the SD card.
File telemetryFile;

// This flag prevents the program from trying to write to the SD card if initialization failed.
bool sd_ok = false;

// This stores the last time the file was flushed.
// Flushing forces buffered data to actually be written to the SD card.
unsigned long lastFlushTime = 0;

// The SD file will be flushed once every 1000 ms.
// This reduces the risk of losing data while avoiding a slow flush every loop.
const unsigned long FLUSH_INTERVAL_MS = 1000;

// This stores the actual PWM command sent to the ESC.
// It is logged so you can compare sensor/PID behavior to the motor command.
int escPulseWidth = ESC_NEUTRAL;

// ===== END ADDED FOR SD LOGGING =====
enum { prelaunch, launch, coast, descend } state = prelaunch;

typedef struct {
  float Q_angle;
  float Q_bias;
  float R_measure;
  float angle;
  float bias;
  float rate;
  float P[2][2];
} Kalman_t;

Kalman_t KalmanX;
Kalman_t KalmanY;
uint32_t timer;

float gyroX_offset = 0;
float gyroY_offset = 0;
float gyroZ_offset = 0;
int num_samples = 500;

float current_error, previous_error = 0;
float prop_error = 0;
float total_integrated_error, integral_error = 0; 
float derivative_error = 0; // Un-commented this so the PID compiles
float motor_control = 0; 

float Kp = -2.5873;
float Ki = -5.9426;
float Kd = 0;
const float MAX_PID_OUTPUT = 12.0f; //Battery Voltage

float launch_thresh = 9.81 * 1.5;

float PIDController(float error, float prev_error, double dt);
void LSM6DSO32Setup(); 
void calibrategyro(); 
void Kalman_Init(Kalman_t *kf); 
float Kalman_GetAngle(Kalman_t *kf, float newAngle, float newRate, float dt); 
void armESC(); 
void sendESCPWM(float control_signal); 

// ===== ADDED FOR SD LOGGING =====
// Function prototype for converting the numeric flight state into readable text.
// Example: instead of logging "0", the SD card logs "prelaunch".
const char* stateName(int s);

// Function prototype for initializing the SD card and opening telemetry.csv.
void setupSD();

// Function prototype for writing one complete telemetry row to the SD card.
// The function receives the current sensor values, Kalman angles, and loop dt.
void logTelemetry(
  sensors_event_t accel,
  sensors_event_t gyro,
  sensors_event_t temp,
  float accel_mag,
  float gyroRateX,
  float gyroRateY,
  float roll,
  float pitch,
  double dt
);
// ===== END ADDED FOR SD LOGGING =====

void setup(void) {
  // 1. INSTANT ESC ARMING (Bidirectional = 1500 Neutral)
  // Doing this first prevents the ESC from timing out while the sensor calibrates
  esc.attach(ESC_PIN, ESC_REVERSE_MAX, ESC_FORWARD_MAX);
  armESC();

  // 2. Serial Initialization
  Serial.begin(115200);
  
  // Explicitly set the RX and TX pins for the built-in Serial2 (Bluetooth)
  Serial2.setRx(PA3);
  Serial2.setTx(PA2);
  Serial2.begin(9600); 

  // Wait for USB Serial to connect so you don't miss the boot sequence
  uint32_t timeout = millis();
  while (!Serial && (millis() - timeout < 3000)) {}

  Serial.println("System Booting...");

  // ===== ADDED FOR SD LOGGING =====
  // This initializes the SD card and opens/creates telemetry.csv.
  // It happens after Serial starts so SD card errors can be printed to Serial.
  // It happens before sensor setup/calibration so the file is ready before flight logic runs.
  setupSD();
  // ===== END ADDED FOR SD LOGGING =====

  // 3. Sensor Initialization
  LSM6DSO32Setup();
  Serial.println("Calibrating Gyro...");
  calibrategyro();
  Serial.println("Calibration Complete.");

  Kalman_Init(&KalmanX);
  Kalman_Init(&KalmanY);
  timer = micros();

  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  sensors_event_t accel, gyro, temp;
  static unsigned long last_time = 0;
  double dt = (double)(micros() - timer) / 1000000.0;
  timer = micros();

  dso32.getEvent(&accel, &gyro, &temp);

  float gyroRateX = (gyro.gyro.x - gyroX_offset) * 57.29578f;
  float gyroRateY = (gyro.gyro.y - gyroY_offset) * 57.29578f;

  float accel_mag = sqrt(pow(accel.acceleration.x,2) + pow(accel.acceleration.y,2) + pow(accel.acceleration.z,2));
  float accRoll  = atan2(accel.acceleration.y, accel.acceleration.z) * 57.29578f;
  float accPitch = atan2(-accel.acceleration.x, sqrt(pow(accel.acceleration.y,2) + pow(accel.acceleration.z, 2))) * 57.29578f;

  float roll  = Kalman_GetAngle(&KalmanX, accRoll, gyroRateX, dt);
  float pitch = Kalman_GetAngle(&KalmanY, accPitch, gyroRateY, dt);

  current_error = (gyro.gyro.z - gyroZ_offset);

  switch(state){
    case prelaunch:
      if(millis() - last_time > 1000) {
        Serial.println("STATE: PRELAUNCH"); // Added to USB Serial
        
        Serial2.print("Acc[X,Y,Z]: [");
        Serial2.print(accel.acceleration.x); Serial2.print(", ");
        Serial2.print(accel.acceleration.y); Serial2.print(", ");
        Serial2.print(accel.acceleration.z); Serial2.print("]\t");
        Serial2.print("Mag: "); Serial2.print(accel_mag); Serial2.print("\t");

        Serial2.print("Temp: "); Serial2.print(temp.temperature); Serial2.print("C\t");
        Serial2.print("Roll: "); Serial2.print(roll); Serial2.print("\t");
        Serial2.print("Pitch: "); Serial2.println(pitch); // Fixed to println
        last_time = millis();
      }

      if (abs(accel_mag - 9.81) > 2.0) {
       Serial2.print("[FLAG: BAD_PAD_CALIBRATION] ");
      }

      if(accel_mag >= launch_thresh){
        state = launch;
        Serial.println("LAUNCH DETECTED!");
        Serial2.println("LAUNCH DETECTED!");
      }
    break;
    
    case launch:
      // Add transition logic to coast here
      state = coast; 
    break;

    case coast:
      motor_control = PIDController(current_error, previous_error, dt);
      
      if (motor_control < 0) {
        digitalWrite(LED_PIN, HIGH); // Just lighting up an LED for debugging
      } else {
        digitalWrite(LED_PIN, LOW);
      }
      
      sendESCPWM(motor_control);
    break;
    
    case descend:
      esc.writeMicroseconds(ESC_NEUTRAL); // Kill motor on descent

      // ===== ADDED FOR SD LOGGING =====
      // Since the motor is being commanded to neutral on descent,
      // this keeps the logged ESC PWM value accurate.
      escPulseWidth = ESC_NEUTRAL;
      // ===== END ADDED FOR SD LOGGING =====
    break;
  }
  // ===== ADDED FOR SD LOGGING =====
  // This writes one full row of telemetry to telemetry.csv every time loop() runs.
  // It is placed after the state machine so the SD card logs the latest state,
  // latest PID values, and latest ESC command from this loop cycle.
  logTelemetry(
    accel,
    gyro,
    temp,
    accel_mag,
    gyroRateX,
    gyroRateY,
    roll,
    pitch,
    dt
  );
  // ===== END ADDED FOR SD LOGGING =====
  previous_error = current_error;
}

void armESC() {
  // ===== ADDED FOR SD LOGGING =====
  // The ESC is armed at neutral, so this records neutral as the current ESC command.
  // Without this, the SD log may not know what PWM was sent during arming.
  escPulseWidth = ESC_NEUTRAL;
  // ===== END ADDED FOR SD LOGGING =====
  esc.writeMicroseconds(ESC_NEUTRAL);
  delay(1000); 
}

void sendESCPWM(float control_signal) {
  // Deadband to prevent motor jittering when near zero
  if (abs(control_signal) < 0.05) {
    // ===== ADDED FOR SD LOGGING =====
    // If the control signal is inside the deadband, the ESC is commanded to neutral.
    // This makes the logged PWM match what is actually sent to the ESC.
    escPulseWidth = ESC_NEUTRAL;
    // ===== END ADDED FOR SD LOGGING =====

    esc.writeMicroseconds(ESC_NEUTRAL);
    return;
  }

  // 1. Convert voltage (-12V to +12V) into a ratio (-1.0 to 1.0)
  float throttle_ratio = control_signal / MAX_PID_OUTPUT; 
  
  // 2. Map the ratio to the Bidirectional PWM range
  // 0 * 500 = 0 (1500us center)
  // 1.0 * 500 = 500 (2000us max forward)
  // -1.0 * 500 = -500 (1000us max reverse)
  int pulseWidth = ESC_NEUTRAL + (int)(throttle_ratio * 500);
  
  // 3. Constrain for safety
  pulseWidth = constrain(pulseWidth, ESC_REVERSE_MAX, ESC_FORWARD_MAX);


  // ===== ADDED FOR SD LOGGING =====
  // Save the final constrained PWM command so it can be written to the SD card.
  // This is better than logging only motor_control because motor_control is in volts,
  // while this is the actual PWM command sent to the ESC.
  escPulseWidth = pulseWidth;
  // ===== END ADDED FOR SD LOGGING =====
  
  esc.writeMicroseconds(pulseWidth);
}

void LSM6DSO32Setup() {
  // Force the I2C pins BEFORE calling begin_I2C()
  Wire.setSCL(PB6);
  Wire.setSDA(PB7);
  Wire.begin();

  if (!dso32.begin_I2C()) {
    while (1) {
      Serial.println("I2C Not Found");
      delay(500);
    }
  }

  dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_16_G);
  dso32.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  dso32.setAccelDataRate(LSM6DS_RATE_833_HZ);
  dso32.setGyroDataRate(LSM6DS_RATE_833_HZ);
}

void calibrategyro() {
  float x_sum = 0, y_sum = 0, z_sum = 0;

  for (int i = 0; i < num_samples; i++) {
    sensors_event_t accel, gyro, temp;
    dso32.getEvent(&accel, &gyro, &temp);

    x_sum += gyro.gyro.x;
    y_sum += gyro.gyro.y;
    z_sum += gyro.gyro.z;
    
    delay(10); // Wait 10ms for a new reading
  }

  gyroX_offset = x_sum / num_samples;
  gyroY_offset = y_sum / num_samples;
  gyroZ_offset = z_sum / num_samples;
}

float PIDController(float error, float prev_error, double dt) {
  if (dt <= 0.0) return 0.0;

  prop_error = Kp * error;

  total_integrated_error += (error * dt);
  if (Ki != 0) {
      float max_accum = abs(MAX_PID_OUTPUT / Ki); 
      total_integrated_error = constrain(total_integrated_error, -max_accum, max_accum);
      integral_error = Ki * total_integrated_error;  
  } else {
      integral_error = 0;
  }

  derivative_error = Kd * ((error - prev_error) / dt);

  float control_output = prop_error + integral_error + derivative_error;
  
  return constrain(control_output, -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
}

void Kalman_Init(Kalman_t *kf) {
    kf->Q_angle = 0.001f;
    kf->Q_bias = 0.003f;
    kf->R_measure = 0.03f;
    kf->angle = 0.0f;
    kf->bias = 0.0f;
    kf->P[0][0] = 0.0f; kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f; kf->P[1][1] = 0.0f;
}

float Kalman_GetAngle(Kalman_t *kf, float newAngle, float newRate, float dt) {
    kf->rate = newRate - kf->bias;
    kf->angle += dt * kf->rate;

    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    float S = kf->P[0][0] + kf->R_measure;
    float K[2];
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    float y = newAngle - kf->angle;

    kf->angle += K[0] * y;
    kf->bias  += K[1] * y;

    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];

    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    return kf->angle;
}
// ===== ADDED FOR SD LOGGING =====

// This function converts the enum state into a readable word for the CSV file.
// Without this, the SD card would only store numbers like 0, 1, 2, or 3.
const char* stateName(int s) {
  switch (s) {
    case prelaunch:
      return "prelaunch";
    case launch:
      return "launch";
    case coast:
      return "coast";
    case descend:
      return "descend";
    default:
      return "unknown";
  }
}

// This function initializes the SD card and creates/opens telemetry.csv.
// It also writes the first CSV row, which is the column header.
void setupSD() {
  Serial.println("Initializing SD card...");

  // Try to start communication with the SD card.
  // If this fails, the program continues running, but SD logging is disabled.
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD CARD FAILED OR NOT PRESENT");
    sd_ok = false;
    return;
  }

  // Open telemetry.csv in append/write mode.
  // If the file does not exist, it will be created.
  // If it already exists, new data is added to the end.
  telemetryFile = SD.open("telemetry.csv", FILE_WRITE);

  // If the file cannot be opened, disable SD logging.
  if (!telemetryFile) {
    Serial.println("Could not open telemetry.csv");
    sd_ok = false;
    return;
  }

  // Mark SD logging as available.
  sd_ok = true;

  // Write column labels so the data is readable when opened in Excel, MATLAB, Python, etc.
  telemetryFile.println(
    "millis,micros,state,"
    "accel_x,accel_y,accel_z,accel_mag,"
    "gyro_x_raw,gyro_y_raw,gyro_z_raw,"
    "gyro_x_offset,gyro_y_offset,gyro_z_offset,"
    "gyro_rate_x_dps,gyro_rate_y_dps,"
    "temp_c,roll_deg,pitch_deg,"
    "current_error,previous_error,"
    "prop_error,integral_error,derivative_error,total_integrated_error,"
    "motor_control,esc_pwm_us,dt"
  );

  // Immediately force the header to be saved to the SD card.
  telemetryFile.flush();

  Serial.println("SD logging ready.");
}

// This function writes one complete telemetry sample to the SD card.
// Each call writes one row in telemetry.csv.
void logTelemetry(
  sensors_event_t accel,
  sensors_event_t gyro,
  sensors_event_t temp,
  float accel_mag,
  float gyroRateX,
  float gyroRateY,
  float roll,
  float pitch,
  double dt
) {
  // If the SD card failed to initialize or the file failed to open,
  // exit immediately so the flight code does not crash trying to write.
  if (!sd_ok || !telemetryFile) {
    return;
  }

  // Time since board startup in milliseconds.
  telemetryFile.print(millis());
  telemetryFile.print(",");

  // Time since board startup in microseconds.
  // This is useful for higher-resolution timing analysis.
  telemetryFile.print(micros());
  telemetryFile.print(",");

  // Current flight state as readable text.
  telemetryFile.print(stateName(state));
  telemetryFile.print(",");

  // Raw accelerometer readings in m/s^2 from the LSM6DSO32.
  telemetryFile.print(accel.acceleration.x, 6);
  telemetryFile.print(",");

  telemetryFile.print(accel.acceleration.y, 6);
  telemetryFile.print(",");

  telemetryFile.print(accel.acceleration.z, 6);
  telemetryFile.print(",");

  // Magnitude of acceleration vector.
  // This is what your launch detection threshold uses.
  telemetryFile.print(accel_mag, 6);
  telemetryFile.print(",");

  // Raw gyro readings from the sensor.
  // Adafruit reports these in rad/s.
  telemetryFile.print(gyro.gyro.x, 6);
  telemetryFile.print(",");

  telemetryFile.print(gyro.gyro.y, 6);
  telemetryFile.print(",");

  telemetryFile.print(gyro.gyro.z, 6);
  telemetryFile.print(",");

  // Gyro offsets found during calibration.
  // These are useful later to verify the gyro bias correction.
  telemetryFile.print(gyroX_offset, 6);
  telemetryFile.print(",");

  telemetryFile.print(gyroY_offset, 6);
  telemetryFile.print(",");

  telemetryFile.print(gyroZ_offset, 6);
  telemetryFile.print(",");

  // Bias-corrected gyro X and Y rates converted to degrees per second.
  // These are the rates sent into the Kalman filters.
  telemetryFile.print(gyroRateX, 6);
  telemetryFile.print(",");

  telemetryFile.print(gyroRateY, 6);
  telemetryFile.print(",");

  // IMU temperature reading in degrees Celsius.
  telemetryFile.print(temp.temperature, 6);
  telemetryFile.print(",");

  // Kalman-filtered roll angle in degrees.
  telemetryFile.print(roll, 6);
  telemetryFile.print(",");

  // Kalman-filtered pitch angle in degrees.
  telemetryFile.print(pitch, 6);
  telemetryFile.print(",");

  // Current yaw-rate error used by the reaction wheel PID.
  // In your code this is gyro Z minus the calibrated Z offset.
  telemetryFile.print(current_error, 6);
  telemetryFile.print(",");

  // Previous loop's yaw-rate error.
  // This is used for the derivative term.
  telemetryFile.print(previous_error, 6);
  telemetryFile.print(",");

  // PID proportional contribution.
  telemetryFile.print(prop_error, 6);
  telemetryFile.print(",");

  // PID integral contribution.
  telemetryFile.print(integral_error, 6);
  telemetryFile.print(",");

  // PID derivative contribution.
  telemetryFile.print(derivative_error, 6);
  telemetryFile.print(",");

  // Stored accumulated error before multiplying by Ki.
  // Useful for checking integral windup behavior.
  telemetryFile.print(total_integrated_error, 6);
  telemetryFile.print(",");

  // Final constrained PID output.
  // Your code treats this like a voltage command from -12 V to +12 V.
  telemetryFile.print(motor_control, 6);
  telemetryFile.print(",");

  // Actual PWM pulse width sent to the ESC in microseconds.
  telemetryFile.print(escPulseWidth);
  telemetryFile.print(",");

  // Loop time step in seconds.
  telemetryFile.println(dt, 6);

  // Flush once per second so data is periodically committed to the SD card.
  // This is safer than only flushing at the end, because the rocket may lose power.
  // It is faster than flushing every single row.
  if (millis() - lastFlushTime > FLUSH_INTERVAL_MS) {
    telemetryFile.flush();
    lastFlushTime = millis();
  }
}

// ===== END ADDED FOR SD LOGGING =====