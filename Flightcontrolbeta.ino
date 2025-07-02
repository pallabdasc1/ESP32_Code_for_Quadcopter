/*
====================================================================================
🛩️  ESP32 Quadcopter Flight Controller (Carbon Aeronautics Port)
====================================================================================

📘 INTRODUCTION:

This project implements a basic **flight controller** for a quadcopter using the 
**ESP32-WROOM DevKit v3** microcontroller in the Arduino environment. It is based 
on the original architecture and logic from **Carbon Aeronautics**, which was 
originally written for the **Teensy 4.0** platform. This version has been carefully 
ported and adapted for use with the ESP32 using FreeRTOS for multitasking and 
ESP32-native features like `ledcWrite()` for motor PWM control.

🎯 FUNCTIONAL OVERVIEW:

- Reads RC commands using a **FlySky PPM receiver** via the `PPMReader` library.
- Collects real-time sensor data from an **MPU6050 IMU** over I2C.
- Uses a **dual-loop PID controller** (angle + rate) to stabilize roll, pitch, and yaw.
- Computes Kalman filter for estimating stable orientation.
- Outputs **PWM signals** to four ESCs (Electronic Speed Controllers) using ESP32’s 
  hardware timers.
- Monitors **battery voltage** through an ADC pin using a voltage divider and 
  warns the user when voltage drops below safe levels.
- Handles tasks in **parallel** using FreeRTOS: flight control and battery monitoring.

📡 CHANNEL MAPPING:
- Channel 1: Roll  
- Channel 2: Pitch  
- Channel 3: Throttle  
- Channel 4: Yaw  
- Channels 5 & 6: Reserved for future use (e.g., arming switch, flight mode)

💡 FEATURES:
- Hardware PWM control with `ledcAttach()` on GPIO 5, 18, 19, and 15.
- FreeRTOS task-based architecture for smooth and responsive performance.
- Safety: shuts off motors if PPM signal is lost.
- Optional Wi-Fi PID tuning interface (commented in current version for simplicity).
- Modular Kalman and PID control implementation.

📜 LICENSE:
This code is provided for **educational and research** purposes only. It is adapted 
from public material by **Carbon Aeronautics** and does **not** claim original 
ownership of core flight control logic. Redistribution is allowed with attribution.

🧑‍💻 Maintained by: **Pallab Das**
📬 Contact: pallabdashere@gmail.com

====================================================================================
*/
#include <Wire.h>
#include <Arduino.h>
#include <PPMReader.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/semphr.h>

SemaphoreHandle_t i2cMutex;

// Variables to store received values
float PRate = 0.08, IRate = 0.04, DRate = 0.001;
float PAngle = 4, IAngle = 0.04, DAngle = 0.0;

// Global
TaskHandle_t autoTuneTaskHandle = NULL;
bool isAutoTuningActive = false;

struct PIDGains {
  float P, I, D;
};

PIDGains tunedAngleRoll, tunedAnglePitch;
PIDGains tunedRateRoll, tunedRatePitch, tunedRateYaw;

//analog read to read voltages using esp32, I am trying to read battery voltages
const int analogPin = 25; // Use the desired ADC pin
float referenceVoltage = 3.3; // Reference voltage in volts (may need calibration)
float r1Value = 77600.0; // Resistance of R1 in ohms
float r2Value = 29400.0; // Resistance of R2 in ohms
float battery_voltage=10;

//PWM signal generator pins and These no.s are GPIOs
const int PWM_PIN_M1 = 5 ; //m1 
const int PWM_PIN_M2 = 18; //m2
const int PWM_PIN_M3 = 19 ;//m3
const int PWM_PIN_M4 = 15 ;//m4
//const int PWM_CHANNEL = 0;
const int PWM_FREQUENCY = 250; //hertz
const int PWM_RESOLUTION = 12;  // 12-bit resolution (0-4095)


//PPM signal receiver
//Initialize a PPMReader on digital pin 4 with 6 expected channels.
byte interruptPin = 4;
byte channelAmount = 6;
PPMReader ppm(interruptPin, channelAmount);
int ppmData[6]={0,0,0,0,0,0}; //Initializing the values

//PPM timeout
const unsigned long ppmTimeout = 5000000; //timeout
unsigned long lastPPMUpdateTime = 0;
bool ppmSignalLost = true; // Assume signal is lost initially
int lastPPMData[6] = {0}; // Store previous PPM values
const int changeThreshold = 10; // Minimum change required to consider signal active

//Code for flight controller
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int   RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float LoopTimer;

float DesiredRateRoll, DesiredRatePitch,DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[]={0, 0, 0};

//PID Rate
float PRateRoll=PRate; float PRatePitch=PRateRoll; float PRateYaw=0.100;
float IRateRoll=IRate; float IRatePitch=IRateRoll; float IRateYaw=0.030;
float DRateRoll=DRate; float DRatePitch=DRateRoll; float DRateYaw=0.000;
/*
float PRateRoll=0.7; float PRatePitch=PRateRoll; float PRateYaw=2;
float IRateRoll=0.0; float IRatePitch=IRateRoll; float IRateYaw=12;
float DRateRoll=0.01; float DRatePitch=DRateRoll; float DRateYaw=0;
*/
float MotorInput1=0, MotorInput2=0, MotorInput3=0, MotorInput4=0;

float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};

float DesiredAngleRoll, DesiredAnglePitch;
float ErrorAngleRoll, ErrorAnglePitch;

float PrevErrorAngleRoll, PrevErrorAnglePitch;
float PrevItermAngleRoll, PrevItermAnglePitch;


//PID Angle
float PAngleRoll=PAngle; float PAnglePitch=PAngleRoll;
float IAngleRoll=IAngle; float IAnglePitch=IAngleRoll;
float DAngleRoll=DAngle; float DAnglePitch=DAngleRoll;
/*
float PAngleRoll=1.5; float PAnglePitch=PAngleRoll;//1
float IAngleRoll=0; float IAnglePitch=IAngleRoll;
float DAngleRoll=0.6; float DAnglePitch=DAngleRoll;//0.5
*/

void kalman_1d(float &KalmanState, float &KalmanUncertainty, float KalmanInput, float KalmanMeasurement) 
{
   // Prediction step
   KalmanState += 0.004 * KalmanInput;
   KalmanUncertainty += 0.004 * 0.004 * 16.0f;  // (4*4 = 16)
   // Update step
   float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 9.0f);  // 3*3 = 9
   KalmanState += KalmanGain * (KalmanMeasurement - KalmanState);
   KalmanUncertainty *= (1 - KalmanGain);
   // Output for logging/debug
   Kalman1DOutput[0] = KalmanState;
   Kalman1DOutput[1] = KalmanUncertainty;
}


void reset_motors()
{
   MotorInput1=1000, MotorInput2=1000, MotorInput3=1000, MotorInput4=1000;  
}


void ppmloop() 
{   
    bool valuesChanged = false;

    for (byte channel = 0; channel < channelAmount; ++channel) 
    {
        int value = ppm.latestValidChannelValue(channel + 1, -1); // Channel numbers start from 1
        
        if (value != -1) 
        {
            ppmData[channel] = value;

            // Check if values have changed significantly (threshold-based)
            if (abs(ppmData[channel] - lastPPMData[channel]) > changeThreshold) 
            {
                valuesChanged = true;
            }
        }
        Serial.print(ppmData[channel]);
        Serial.print("\t"); // Tab for spacing
    }

    Serial.print(" | "); // Separator for readability

    unsigned long currentTime = micros();
    unsigned long elapsedTime = currentTime - lastPPMUpdateTime;

    // If values change significantly, update lastPPMUpdateTime and reset signal lost flag
    if (valuesChanged) 
    {
        lastPPMUpdateTime = currentTime;
        ppmSignalLost = false; // Signal is present
    }
    // If values have NOT changed for 15 seconds, consider signal lost
    else if (elapsedTime > ppmTimeout) 
    {
        ppmSignalLost = true;
    }

    // Print debug information
    Serial.print("Elapsed Time: ");
    Serial.print(elapsedTime / 1000000.0); // Convert to seconds for readability
    Serial.print(" s | PPM Signal Lost: ");
    Serial.println(ppmSignalLost ? "true" : "false");

    // Save current values for the next loop
    for (byte channel = 0; channel < channelAmount; ++channel) 
    {
        lastPPMData[channel] = ppmData[channel];
    }

    // Handle PPM signal loss
    if (ppmSignalLost) 
    {
        Serial.println("PPM Signal Lost! Resetting Controls...");
        
        // Set throttle and other channels to a safe value
        ppmData[2] = 1000; // Set throttle to minimum (Idle)
        ppmData[0] = 1500; // Centered Roll
        ppmData[1] = 1500; // Centered Pitch
        ppmData[3] = 1500; // Centered Yaw

        reset_pid();       // Reset PID to avoid sudden movements
        reset_motors();
    }
    
}


void pwmsetup() 
{
   pinMode(PWM_PIN_M1, OUTPUT); 
   pinMode(PWM_PIN_M2, OUTPUT);
   pinMode(PWM_PIN_M3, OUTPUT);
   pinMode(PWM_PIN_M4, OUTPUT);

   ledcAttach(PWM_PIN_M1, PWM_FREQUENCY, PWM_RESOLUTION);
   ledcAttach(PWM_PIN_M2, PWM_FREQUENCY, PWM_RESOLUTION);
   ledcAttach(PWM_PIN_M3, PWM_FREQUENCY, PWM_RESOLUTION);
   ledcAttach(PWM_PIN_M4, PWM_FREQUENCY, PWM_RESOLUTION);
}

void pwmloop(int motor_input,int PWM_PIN) 
{
   ledcWrite(PWM_PIN, motor_input);
}

float batteryvoltage() 
{
  int rawValue = analogRead(analogPin);
  float voltage = (rawValue / 4095.0) * referenceVoltage;
  // Calculate actual input voltage using voltage divider formula
  float inputVoltage = voltage * ((r1Value + r2Value) / r2Value);
  //Serial.print("Raw ADC Value: ");
  //Serial.println(rawValue);
  //Serial.print("Input Voltage: ");
  //Serial.print(inputVoltage, 3); // Print voltage with 3 decimal places
  //Serial.println(" V");
  //delay(1); // Wait for a second before taking the next reading
  return inputVoltage;
}

void pid_equation(float Error, float P , float I, float D, float PrevError, float PrevIterm) 
{
   float Pterm=P*Error;
   float Iterm=PrevIterm+I*(Error+PrevError)*0.004/2;
   
   if (Iterm > 400) 
       Iterm=400;
   else if (Iterm <-400) 
       Iterm=-400;
   
   float Dterm=D*(Error-PrevError)/0.004;
   float PIDOutput= Pterm+Iterm+Dterm;
   
   if (PIDOutput>400) 
       PIDOutput=400;
   else if (PIDOutput <-400) 
       PIDOutput=-400;
   
   PIDReturn[0]=PIDOutput;
   PIDReturn[1]=Error;
   PIDReturn[2]=Iterm;
}

void reset_pid(void)
{  
   //Author: Pallab das
   PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
   PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
   
   PrevErrorAngleRoll=0; PrevErrorAnglePitch=0; //for outer pid loop
   PrevItermAngleRoll=0; PrevItermAnglePitch=0; //-------------------
}

void gyro_signals(void) 
{ 
    if (i2cMutex != NULL) 
    {
        if (xSemaphoreTake(i2cMutex, (TickType_t)10) == pdTRUE) 
        {
            Wire.beginTransmission(0x68);          
            Wire.write(0x1A);
            Wire.write(0x05);
            Wire.endTransmission();
            Wire.beginTransmission(0x68);
            Wire.write(0x1C);
            Wire.write(0x10);
            Wire.endTransmission();
            Wire.beginTransmission(0x68);
            Wire.write(0x3B);
            Wire.endTransmission();
            Wire.requestFrom(0x68,6);
            int16_t AccXLSB = Wire.read() << 8 |Wire.read();
            int16_t AccYLSB = Wire.read() << 8 |Wire.read();
            int16_t AccZLSB = Wire.read() << 8 |Wire.read();
            Wire.beginTransmission(0x68);
            Wire.write(0x1B);
            Wire.write(0x8);
            Wire.endTransmission();
            Wire.beginTransmission(0x68);
            Wire.write(0x43);
            Wire.endTransmission();
            Wire.requestFrom(0x68,6);
            int16_t GyroX=Wire.read()<<8 | Wire.read();
            int16_t GyroY=Wire.read()<<8 | Wire.read();
            int16_t GyroZ=Wire.read()<<8 | Wire.read();
            RateRoll=(float)GyroX/65.5;
            RatePitch=(float)GyroY/65.5;
            RateYaw=(float)GyroZ/65.5;
            AccX=(float)AccXLSB/4096-0.02;
            AccY=(float)AccYLSB/4096;
            AccZ=(float)AccZLSB/4096-0.08;
            AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
            AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
            xSemaphoreGive(i2cMutex);
        }
        else
        {
             Serial.println("I2C mutex busy");  
        }
    }
    else
    {
        Serial.println("I2C mutex not created");
    }
}



void batteryMonitorTask(void *pvParameters)
{
   while(1)
   {
      Serial.print("\nBATTERY MONITOR TASK\n"); 
      battery_voltage = batteryvoltage();
      if (battery_voltage < 9) 
      {   // Check if voltage is below 9V
         Serial.println("Battery level low please Recharge the Battery");
         digitalWrite(2, HIGH); // Turn on the LED
         delay(500); 
         digitalWrite(2, LOW); // Turn off the LED
         delay(500);
      } 
      else 
      {
         digitalWrite(2, HIGH); // Ensure the LED is off if voltage is above 9V
      }
   }
}
void flightControlTask(void *pvParameters)
{
    while(1)
    {
        gyro_signals(); //connect to gyro
        RateRoll-=RateCalibrationRoll;
        RatePitch-=RateCalibrationPitch;
        RateYaw-=RateCalibrationYaw;
        kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);  //Roll
        KalmanAngleRoll=Kalman1DOutput[0];
        KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
        kalman_1d(KalmanAnglePitch,KalmanUncertaintyAnglePitch, RatePitch, AnglePitch); //pitch
        KalmanAnglePitch=Kalman1DOutput[0];
        KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
        ppmloop();
        DesiredAngleRoll=0.10*(ppmData[0]-1500); // desired roll
        DesiredAnglePitch=0.10*(ppmData[1]-1500); // desired pitch
       
        /*show angle for testing purpose*/
        Serial.print(" Roll Angle [°] ");
        Serial.print(KalmanAngleRoll);
        Serial.print(" Pitch Angle [°] ");
        Serial.println(KalmanAnglePitch);
        
        InputThrottle=ppmData[2];   //desired throttle
        DesiredRateYaw=0.15*(ppmData[3]-1500); //yaw
        ErrorAngleRoll=DesiredAngleRoll-KalmanAngleRoll;
        ErrorAnglePitch=DesiredAnglePitch-KalmanAnglePitch;
    
        pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll);
        DesiredRateRoll=PIDReturn[0];
        PrevErrorAngleRoll=PIDReturn[1];
        PrevItermAngleRoll=PIDReturn[2];
    
        pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
        DesiredRatePitch=PIDReturn[0];
        PrevErrorAnglePitch=PIDReturn[1];
        PrevItermAnglePitch=PIDReturn[2];
        
        ErrorRateRoll=DesiredRateRoll-RateRoll;
        ErrorRatePitch=DesiredRatePitch-RatePitch;
        ErrorRateYaw=DesiredRateYaw-RateYaw;
    
        pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
        InputRoll=PIDReturn[0];
        PrevErrorRateRoll=PIDReturn[1];
        PrevItermRateRoll=PIDReturn[2];
    
        pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
        InputPitch=PIDReturn[0];
        PrevErrorRatePitch=PIDReturn[1];
        PrevItermRatePitch=PIDReturn[2];
    
        pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
        InputYaw=PIDReturn[0];
        PrevErrorRateYaw=PIDReturn[1];
        PrevItermRateYaw=PIDReturn[2];
    
        if (InputThrottle > 1800) InputThrottle = 1800;
       
        //input throtle value multiplied with 1.024
        MotorInput1= 1.024*(InputThrottle-InputRoll-InputPitch-InputYaw); //m1
        MotorInput2= 1.024*(InputThrottle-InputRoll+InputPitch+InputYaw); //m2
        MotorInput3= 1.024*(InputThrottle+InputRoll+InputPitch-InputYaw); //m3
        MotorInput4= 1.024*(InputThrottle+InputRoll-InputPitch+InputYaw); //m4
    
        if (MotorInput1 > 2000) MotorInput1 = 1989;
        if (MotorInput2 > 2000) MotorInput2 = 1989;
        if (MotorInput3 > 2000) MotorInput3 = 1989;
        if (MotorInput4 > 2000) MotorInput4 = 1989;
    
    
        int ThrottleIdle=1180;
        if (MotorInput1 < ThrottleIdle) MotorInput1 = ThrottleIdle;
        if (MotorInput2 < ThrottleIdle) MotorInput2 = ThrottleIdle;
        if (MotorInput3 < ThrottleIdle) MotorInput3 = ThrottleIdle;
        if (MotorInput4 < ThrottleIdle) MotorInput4 = ThrottleIdle;
    
        int ThrottleCutOff=1000;
        if (ppmData[2]<1050) 
        { 
           MotorInput1=ThrottleCutOff;
           MotorInput2=ThrottleCutOff;
           MotorInput3=ThrottleCutOff;
           MotorInput4=ThrottleCutOff;
           reset_pid();
        }
        /*
        Serial.print("  error Roll Angle [°] ");
        Serial.print( ErrorAngleRoll);
        Serial.print("  error Pitch Angle [°] ");
        Serial.println( ErrorAnglePitch);
        */
        /*
        Serial.print("\nFLIGHT CONTROL TASK\n");
        Serial.print("\n  Motor 1 out ");
        Serial.println( MotorInput1);
        Serial.print("\n  Motor 2 out ");
        Serial.println( MotorInput2);
        Serial.print("\n  Motor 3 out ");
        Serial.println( MotorInput3);
        Serial.print("\n  Motor 4 out ");
        Serial.println( MotorInput4);
        */
        /*These lines are used to send pwm signals to the escs connected to the microcontroller esp32*/
        if (!isAutoTuningActive) 
        {
        // Only write motor signals from flight controller if tuning is OFF
    
            pwmloop(MotorInput1,PWM_PIN_M1);  //gpio5  output to esc m1 -- anti clkwise
            pwmloop(MotorInput2,PWM_PIN_M2);  //gpio18 output to esc m2 -- clkwise
            pwmloop(MotorInput3,PWM_PIN_M3);  //gpio19 output to esc m3 -- anti clkwise
            pwmloop(MotorInput4,PWM_PIN_M4);  //gpio15 output to esc m4 -- clkwise
        }
            
        while ((micros() - LoopTimer) < 4000);
        LoopTimer=micros();
    }
}

void gyroscope_calibration()
{
    for(RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber++)  //gyroscope calibration
    {
        gyro_signals();
        RateCalibrationRoll+=RateRoll;
        RateCalibrationPitch+=RatePitch;
        RateCalibrationYaw+=RateYaw;
        delay(1);
    }
    RateCalibrationRoll/=2000; 
    RateCalibrationPitch/=2000;
    RateCalibrationYaw/=2000;
}

/*Autotune setup*/

void startAutoTuneTask() 
{
    if (autoTuneTaskHandle == NULL) 
    {
        xTaskCreatePinnedToCore(autoTunePIDTask, "AutoTuneTask", 8192, NULL, 1, &autoTuneTaskHandle, 1);
    }
}

void autoTunePIDTask(void *pvParameters) 
{
  while (1) 
  {
    if (ppmData[4] > 1700 && !isAutoTuningActive) 
    {
         isAutoTuningActive = true;
         Serial.println("\n[AutoTune] Channel 5 activated. Starting auto-tune...");
   
         // Roll - Angle
         tunedAngleRoll = autoTuneAxis("Angle Roll", &KalmanAngleRoll, true);
         PAngleRoll = tunedAngleRoll.P;
         IAngleRoll = tunedAngleRoll.I;
         DAngleRoll = tunedAngleRoll.D;
   
         // Pitch - Angle
         tunedAnglePitch = autoTuneAxis("Angle Pitch", &KalmanAnglePitch, true);
         PAnglePitch = tunedAnglePitch.P;
         IAnglePitch = tunedAnglePitch.I;
         DAnglePitch = tunedAnglePitch.D;
   
         // Roll - Rate
         tunedRateRoll = autoTuneAxis("Rate Roll", &RateRoll, false);
         PRateRoll = tunedRateRoll.P;
         IRateRoll = tunedRateRoll.I;
         DRateRoll = tunedRateRoll.D;
   
         // Pitch - Rate
         tunedRatePitch = autoTuneAxis("Rate Pitch", &RatePitch, false);
         PRatePitch = tunedRatePitch.P;
         IRatePitch = tunedRatePitch.I;
         DRatePitch = tunedRatePitch.D;
   
         // Yaw - Rate
         tunedRateYaw = autoTuneAxis("Rate Yaw", &RateYaw, false);
         PRateYaw = tunedRateYaw.P;
         IRateYaw = tunedRateYaw.I;
         DRateYaw = tunedRateYaw.D;
   
         Serial.println("[AutoTune] Complete. Updated all PID values.");
         isAutoTuningActive = false;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

PIDGains autoTuneAxis(String label, float *sensorValue, bool isAngle) 
{
     float testP[] = {0.5, 1.0, 1.5, 2.0};
     float testI[] = {0.01, 0.03, 0.05};
     float testD[] = {0.001, 0.01, 0.05};
     PIDGains best = {0, 0, 0};
     float bestScore = -1e6;
   
     for (int p = 0; p < sizeof(testP) / sizeof(testP[0]); p++) 
     {
       for (int i = 0; i < sizeof(testI) / sizeof(testI[0]); i++) 
       {
         for (int d = 0; d < sizeof(testD) / sizeof(testD[0]); d++) 
         {
           float score = runPIDTest(testP[p], testI[i], testD[d], sensorValue, isAngle);
           if (score > bestScore) 
           {
               bestScore = score;
               best.P = testP[p];
               best.I = testI[i];
               best.D = testD[d];
           }
         }
       }
     }
   
     Serial.printf("%s | Best PID: P=%.2f I=%.3f D=%.3f\n", label.c_str(), best.P, best.I, best.D);
     return best;
}

float runPIDTest(float P, float I, float D, float *feedback, bool isAngle) 
{
    float error = 0, prevError = 0, iTerm = 0, output = 0;
    float input = 0, responseError = 0;
    float score = 0;    
    float sweepFrequency = 0.5;
    float sweepAmplitude = 5.0;
    int testThrottle = 1300;
    int durationSec = 3;
    unsigned long startTime = millis();    
    KalmanAngleRoll = 0;
    PrevErrorAngleRoll = 0;
    PrevItermAngleRoll = 0;    
    while ((millis() - startTime) < durationSec * 1000) 
    {
         gyro_signals();
         RateRoll -= RateCalibrationRoll;
         RatePitch -= RateCalibrationPitch;
         RateYaw -= RateCalibrationYaw;    
         kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
         KalmanAngleRoll = Kalman1DOutput[0];
         KalmanUncertaintyAngleRoll = Kalman1DOutput[1];    
         kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
         KalmanAnglePitch = Kalman1DOutput[0];
         KalmanUncertaintyAnglePitch = Kalman1DOutput[1];    
         float time = (millis() - startTime) / 1000.0;
         input = sweepAmplitude * sin(2 * PI * sweepFrequency * time);    
         if (isAngle) 
         {
             error = input - KalmanAngleRoll;
             pid_equation(error, P, I, D, PrevErrorAngleRoll, PrevItermAngleRoll);
             output = PIDReturn[0];
             PrevErrorAngleRoll = PIDReturn[1];
             PrevItermAngleRoll = PIDReturn[2];
         } 
         else 
         {
             error = input - *feedback;
             iTerm += I * error * 0.004;
             float dTerm = D * (error - prevError) / 0.004;
             output = P * error + iTerm + dTerm;
             prevError = error;
         }    
         MotorInput1 = testThrottle - output;
         MotorInput2 = testThrottle - output;
         MotorInput3 = testThrottle + output;
         MotorInput4 = testThrottle + output; 

         if (xSemaphoreTake(motorMutex, (TickType_t)5) == pdTRUE) 
         {
             pwmloop(MotorInput1, PWM_PIN_M1);
             pwmloop(MotorInput2, PWM_PIN_M2);
             pwmloop(MotorInput3, PWM_PIN_M3);
             pwmloop(MotorInput4, PWM_PIN_M4);
             xSemaphoreGive(motorMutex);
         }    
         //responseError = fabs(input - (isAngle ? KalmanAngleRoll : *feedback));
         responseError = fabs(input - *feedback);
         score -= responseError;
         delay(20);
    }    
    return score;
}

void setup()  
{
    i2cMutex = xSemaphoreCreateMutex();
    motorMutex = xSemaphoreCreateMutex();
    Serial.begin(115200); 

    if (motorMutex == NULL) 
    {
         Serial.println("Failed to create motor mutex!");
         while(1);
    }
    
    if (i2cMutex == NULL) 
    {
        Serial.println("Failed to create I2C mutex!");
        while(1); // halt or handle error
    }
    pinMode(2, OUTPUT); //lights the led
    analogReadResolution(12); // Set ADC resolution to 12 bits (0-4095)  
    pinMode(analogPin, INPUT);
    //Wire.setClock(400000);
    Wire.setClock(50000);
    Wire.begin();
    
 
    delay(250);
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
    gyroscope_calibration();
    pwmsetup();
    LoopTimer=micros();
    startAutoTuneTask()
    xTaskCreatePinnedToCore(batteryMonitorTask, "Battery monitor Task", 4096, NULL, 1, NULL,0);
    //xTaskCreatePinnedToCore(flightControlTask, "Flight Control Task", 9216, NULL, 1, NULL,1);
    xTaskCreatePinnedToCore(flightControlTask, "Flight Control Task", 9216, NULL, 1, &flightControlHandle, 1);
    
}

void loop() 
{    
    //Serial.println("Main loop running...");
    //delay(1000);  // Simulate main loop work
}
    
    
 
