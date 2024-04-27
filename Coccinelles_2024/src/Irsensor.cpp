#include <Wire.h>
#include <Arduino.h>
#include <Irsensor.h>

#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX

// Create an object for our VL53L5CX sensor

Irsensor::Irsensor(int Ir_PIN)
{
    m_IR_PIN = Ir_PIN;
    dataReady = false;
}

// Interruption part
void Irsensor::interruptRoutine()
{ // Just set the flag that we have updated data and return from the ISR
    dataReady = true;
}

void Irsensor::setup()
{
    m_time = millis();
    Serial.begin(115200);
    delay(1000);
    Serial.println("SparkFun VL53L5CX Imager Example");

    Wire.begin();           // This resets I2C bus to 100kHz
    Wire.setClock(1000000); // Sensor has max I2C freq of 1MHz

    // myImager.setWireMaxPacketSize(128); //Increase default from 32 bytes to 128 - not supported on all platforms

    Serial.println("Initializing sensor board. This can take up to 10s. Please wait.");

    // Time how long it takes to transfer firmware to sensor
    long startTime = millis();
    bool startup = myImager.begin();
    long stopTime = millis();

    if (startup == false)
    {
        Serial.println(F("Sensor not found - check your wiring. Freezing"));
        while (1)
            ;
    }

    Serial.print("Firmware transfer time: ");
    float timeTaken = (stopTime - startTime) / 1000.0;
    Serial.print(timeTaken, 3);
    Serial.println("s");

    myImager.setResolution(8 * 8); // Enable all 64 pads

    imageResolution = myImager.getResolution(); // Query sensor for current resolution - either 4x4 or 8x8
    imageWidth = sqrt(imageResolution);         // Calculate printing width

    myImager.startRanging(); // Start continuous ranging
}

void Irsensor::loop()
{
    if (millis() - m_time >= dt) // 10ms delay between readings
    {

        // Poll sensor for new data
        if (myImager.isDataReady() == true)
        {
            if (myImager.getRangingData(&measurementData)) // Read distance data into array
            {
                // Calcul le minimum de la distance
                int minDistance = measurementData.distance_mm[0];

                // Iteration a travers la matrice de distance
                for (int y = 0; y <= imageWidth * (imageWidth - 1); y += imageWidth)
                {
                    for (int x = imageWidth - 1; x >= 0; x--)
                    {
                        int currentDistance = measurementData.distance_mm[x + y];
                        if (currentDistance < minDistance)
                        {
                            minDistance = currentDistance;
                        }
                    }
                }
                m_minimum_distance = minDistance;
                // Print minimum distance
                // Serial.print("Minimum distance: ");
                // Serial.println(m_minimum_distance);
            }
        }
        m_time = millis();
    }
}

/*
void Irsensor::loop()
{
    // Poll sensor for new data
    if (myImager.isDataReady() == true)
    {
        if (myImager.getRangingData(&measurementData)) // Read distance data into array
        {
            // The ST library returns the data transposed from zone mapping shown in datasheet
            // Pretty-print data with increasing y, decreasing x to reflect reality

                        for (int y = 0; y <= imageWidth * (imageWidth - 1); y += imageWidth)
            {
                for (int x = imageWidth - 1; x >= 0; x--)
                {
                    Serial.print("\t");
                    Serial.print(measurementData.distance_mm[x + y]);
                }
                Serial.println();
            }
            Serial.println();
        }
    }

    delay(5); // Small delay between polling
}
*/