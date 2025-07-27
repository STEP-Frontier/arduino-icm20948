#include <Arduino.h>
#include <Wire.h>
#include "icm.h"

// I2C設定
#define SDA_PIN 4 // SDA pin for I2C
#define SCL_PIN 5 // SCL pin for I2C

// Create an instance of the ICM20948 class
ICM20948 mySensor;

void setup()
{
    Serial.begin(115200);

    // Initialize ICM20948 with I2C
    while (!mySensor.init(SDA_PIN, SCL_PIN))
    {
        Serial.println("ICM-20948 initialization failed. Retrying...");
        delay(1000); // Wait before retrying
    }
    Serial.println("ICM-20948 initialization successful.");
    // if (mySensor.init(SDA_PIN, SCL_PIN))
    // {
    //     Serial.println("ICM-20948 initialization successful.");
    // }
    // else
    // {
    //     Serial.println("ICM-20948 initialization failed. Stopping.");
    //     while (1)
    //     {
    //         Serial.println("ICM-20948 initialization failed. Stopping.");
    //         delay(500); // Wait indefinitely
    //     }
    // }

    Serial.println("ICM-20948 ready for data reading.");
}

void loop()
{
    Vector accel, gyro, mag;

    // Read sensor data
    mySensor.get_accel(accel);
    mySensor.get_gyro(gyro);
    mySensor.get_mag(mag);
    float temperature = mySensor.get_temperature();

    // Display sensor data
    Serial.println("--- Sensor Data ---");

    // 加速度 (g)
    Serial.print("Accel (m/s^2): ");
    Serial.print("X=");
    Serial.print(accel.x, 3);
    Serial.print(" | Y=");
    Serial.print(accel.y, 3);
    Serial.print(" | Z=");
    Serial.println(accel.z, 3);

    // ジャイロ (dps)
    Serial.print("Gyro (rad/s): ");
    Serial.print("X=");
    Serial.print(gyro.x, 3);
    Serial.print(" | Y=");
    Serial.print(gyro.y, 3);
    Serial.print(" | Z=");
    Serial.println(gyro.z, 3);

    // 磁力計データ (μT)
    Serial.print("Magnetometer (μT): ");
    Serial.print("X=");
    Serial.print(mag.x, 3);
    Serial.print(" | Y=");
    Serial.print(mag.y, 3);
    Serial.print(" | Z=");
    Serial.println(mag.z, 3);

    // 温度 (°C)
    Serial.print("Temperature (°C): ");
    Serial.println(temperature, 2);

    // 3軸角度計算（ピッチ、ロール、ヨー）
    // ピッチ (X軸周りの回転): -90度から+90度
    float pitch = atan2(-accel.x, sqrt(accel.y * accel.y + accel.z * accel.z)) * 180.0 / PI;

    // ロール (Y軸周りの回転): -180度から+180度
    float roll = atan2(accel.y, accel.z) * 180.0 / PI;

    // ヨー (Z軸周りの回転、磁力計から): 0度から360度
    float yaw = atan2(mag.y, mag.x) * 180.0 / PI;
    if (yaw < 0)
    {
        yaw += 360.0;
    }

    Serial.println("--- 3-Axis Orientation ---");
    Serial.print("Pitch (X-axis rotation): ");
    Serial.print(pitch, 2);
    Serial.println(" degrees");

    Serial.print("Roll (Y-axis rotation): ");
    Serial.print(roll, 2);
    Serial.println(" degrees");

    Serial.print("Yaw (Z-axis rotation): ");
    Serial.print(yaw, 2);
    Serial.println(" degrees");

    Serial.println();

    delay(500);
}