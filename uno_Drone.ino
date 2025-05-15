#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>

// -------------------- PPM CONFIG --------------------
#define PPM_PIN 2
#define NUMBER_OF_PPM_CHANNELS 6


volatile bool ppm_read = false, ppm_sync = false;
volatile uint32_t current_ppm_clock = 0, last_ppm_clock = 0, ppm_dt = 0;
volatile uint8_t ppm_current_channel = 0;
volatile uint32_t ppm_channels[NUMBER_OF_PPM_CHANNELS + 1];

// -------------------- MPU6050 CONFIG --------------------
MPU6050 mpu;

#define MPU_INTERRUPT_PIN 2
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

// Filtered angles
float filteredYaw = 0, filteredPitch = 0, filteredRoll = 0;
const float alpha = 0.1;  // Low-pass filter smoothing factor (0.0 - 1.0)

// Low-pass filter function
float lowPassFilter(float previous, float current, float alpha) {
  return previous * (1 - alpha) + current * alpha;
}

volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; }

// -------------------- PID CONFIG --------------------
struct PID {
  float Kp, Ki, Kd;
  float setpoint, input, output;
  float prevError, integral;

  float compute(float dt) {
    float error = setpoint - input;
    integral += error * dt;
    float derivative = (error - prevError) / dt;
    output = Kp * error + Ki * integral + Kd * derivative;
    prevError = error;
    return output;
  }
};

PID pidRoll = {5.0, 0.01, 0.5, 0, 0, 0, 0};  // Adjust as needed
PID pidPitch = {5.0, 0.01, 0.5, 0, 0, 0, 0}; // Adjust as needed

// -------------------- ESC CONFIG --------------------
Servo ESC1, ESC2, ESC3, ESC4;

void setupESCs() {
  ESC1.attach(9, 1000, 2000);  // Front Left
  ESC2.attach(10, 1000, 2000); // Front Right
  ESC3.attach(11, 1000, 2000); // Rear Right
  ESC4.attach(6, 1000, 2000);  // Rear Left

  ESC1.writeMicroseconds(1000);
  ESC2.writeMicroseconds(1000);
  ESC3.writeMicroseconds(1000);
  ESC4.writeMicroseconds(1000);
  delay(2000);  // Allow ESCs to arm
}

// -------------------- FLAGS --------------------
struct Flags {
  bool ARMED = false;
} f;

// -------------------- INTERRUPT: PPM INPUT --------------------
void ppmRising() {
  current_ppm_clock = micros();
  ppm_dt = current_ppm_clock - last_ppm_clock;

  if (ppm_dt >= 3500) {  // Sync pulse
    ppm_sync = true;
    ppm_current_channel = 0;
  } else if (ppm_sync) {
    ppm_current_channel++;
    if (ppm_current_channel <= NUMBER_OF_PPM_CHANNELS) {
      ppm_channels[ppm_current_channel] = ppm_dt;
    }
    if (ppm_current_channel >= NUMBER_OF_PPM_CHANNELS) {
      ppm_read = true;  // PPM read complete
      ppm_sync = false;
    }
  }

  last_ppm_clock = current_ppm_clock;
}

// -------------------- MPU6050 INIT --------------------
void initMPU6050() {
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C speed

  Serial.println(F("Initializing MPU6050..."));
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println(F("MPU6050 connection failed!"));
    while (1);  // Halt if connection fails
  }

  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);  // Adjust offsets as needed

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();

    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println(F("DMP ready."));
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
    while (1);  // Halt on failure
  }
}

// -------------------- ARM/DISARM --------------------
void go_arm() {
  f.ARMED = true;
  Serial.println("ARMED");
}

void go_disarm() {
  f.ARMED = false;
  ESC1.writeMicroseconds(1000);
  ESC2.writeMicroseconds(1000);
  ESC3.writeMicroseconds(1000);
  ESC4.writeMicroseconds(1000);
  Serial.println("DISARMED");
}

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for serial connection

  setupESCs();
  initMPU6050();

  pinMode(PPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmRising, RISING);

  Serial.println(F("Setup complete."));
}

// -------------------- MAIN LOOP --------------------
void loop() {
  if (!dmpReady) return;

  if (ppm_read) {
    ppm_read = false;

    uint32_t throttle = ppm_channels[1];
    uint32_t yawCmd   = ppm_channels[2];
    uint32_t rollCmd  = ppm_channels[3];
    uint32_t pitchCmd = ppm_channels[4];

    // Map PPM commands to angles (-30° to 30°)
    const int rollOffset = -7;  
    const int pitchOffset = -5; 

    pidRoll.setpoint = map(rollCmd, 1000, 2000, -30, 30) - rollOffset;
    pidPitch.setpoint = map(pitchCmd, 1000, 2000, -30, 30) - pitchOffset;

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      // Convert radians to degrees
      float rawYaw = ypr[0] * 180.0 / M_PI;
      float rawPitch = ypr[1] * 180.0 / M_PI;
      float rawRoll = ypr[2] * 180.0 / M_PI;

      // Apply low-pass filter to the angles
      filteredYaw = lowPassFilter(filteredYaw, rawYaw, alpha);
      filteredPitch = lowPassFilter(filteredPitch, rawPitch, alpha);
      filteredRoll = lowPassFilter(filteredRoll, rawRoll, alpha);

      pidRoll.input = filteredRoll;
      pidPitch.input = filteredPitch;

      float dt = 0.02;  // 20 ms control loop
      float rollCorrection = pidRoll.compute(dt);
      float pitchCorrection = pidPitch.compute(dt);

      // -------------------- ARM/DISARM LOGIC --------------------
      if (throttle <= 1050 && yawCmd >= 1950 && !f.ARMED) go_arm();
      if (throttle <= 1050 && yawCmd <= 1050 && f.ARMED) go_disarm();

      if (!f.ARMED) {
        ESC1.writeMicroseconds(1000);
        ESC2.writeMicroseconds(1000);
        ESC3.writeMicroseconds(1000);
        ESC4.writeMicroseconds(1000);
        return;
      }

      // -------------------- MOTOR MIXING --------------------
      uint16_t m1 = constrain(throttle - rollCorrection + pitchCorrection, 1000, 2000);  // Front Left
      uint16_t m2 = constrain(throttle + rollCorrection + pitchCorrection, 1000, 2000);  // Front Right
      uint16_t m3 = constrain(throttle + rollCorrection - pitchCorrection, 1000, 2000);  // Rear Right
      uint16_t m4 = constrain(throttle - rollCorrection - pitchCorrection, 1000, 2000);  // Rear Left

      ESC1.writeMicroseconds(m1);
      ESC2.writeMicroseconds(m2);
      ESC3.writeMicroseconds(m3);
      ESC4.writeMicroseconds(m4);

      // -------------------- DEBUG OUTPUT --------------------
      Serial.print("Roll Set: "); Serial.print(pidRoll.setpoint);
      Serial.print(" | Roll: "); Serial.print(filteredRoll);
      Serial.print(" | Pitch Set: "); Serial.print(pidPitch.setpoint);
      Serial.print(" | Pitch: "); Serial.println(filteredPitch);
    }
  }
}
