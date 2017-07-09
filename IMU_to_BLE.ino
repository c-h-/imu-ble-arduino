/*
   Copyright (c) 2015 Intel Corporation.  All rights reserved.

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*
   This sketch example partially implements the standard Bluetooth Low-Energy Heart Rate service.
   For more information: https://developer.bluetooth.org/gatt/services/Pages/ServicesHome.aspx
*/

#include <CurieBLE.h>
#include "CurieIMU.h"

// IMU Setup
int ax, ay, az;         // accelerometer values
int gx, gy, gz;         // gyrometer values

const int ledPin = 13;      // activity LED pin
boolean blinkState = false; // state of the LED

int calibrateOffsets = 1; // int to determine whether calibration takes place or not

// BLE Setup
BLEPeripheral blePeripheral;       // BLE Peripheral Device (the board you're programming)
BLEService axService("180D"); // BLE Heart Rate Service

// BLE Heart Rate Measurement Characteristic"
BLECharacteristic axChar("2A37",  // standard 16-bit characteristic UUID
    BLERead | BLENotify, 2);  // remote clients will be able to get notifications if this characteristic changes
                              // the characteristic is 2 bytes long as the first field needs to be "Flags" as per BLE specifications
                              // https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.heart_rate_measurement.xml
BLECharacteristic ayChar("2A38", BLERead | BLENotify, 2);
BLECharacteristic azChar("2A39", BLERead | BLENotify, 2);
BLECharacteristic gxChar("2A40", BLERead | BLENotify, 2);
BLECharacteristic gyChar("2A41", BLERead | BLENotify, 2);
BLECharacteristic gzChar("2A42", BLERead | BLENotify, 2);

int axo = 0;  // last ax reading from input
int axfo = 0;
int ayo = 0;
int ayfo = 0;
int azo = 0;
int azfo = 0;
int gxo = 0;
int gxfo = 0;
int gyo = 0;
int gyfo = 0;
int gzo = 0;
int gzfo = 0;
long previousMillis = 0;  // last time the ax was checked, in ms

void setup() {
  Serial.begin(9600);    // initialize serial communication
  while (!Serial);    // wait for the serial port to open

  // IMU Setup
  // initialize device
  Serial.println("Initializing IMU device...");
  CurieIMU.begin();

  // verify connection
  Serial.println("Testing device connections...");
  if (CurieIMU.begin()) {
    Serial.println("CurieIMU connection successful");
  } else {
    Serial.println("CurieIMU connection failed");
  }

  
  // use the code below to calibrate accel/gyro offset values
  if (calibrateOffsets == 1) {
    Serial.println("Internal sensor offsets BEFORE calibration...");
    Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));
    Serial.print("\t"); // -76
    Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));
    Serial.print("\t"); // -235
    Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS));
    Serial.print("\t"); // 168
    Serial.print(CurieIMU.getGyroOffset(X_AXIS));
    Serial.print("\t"); // 0
    Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
    Serial.print("\t"); // 0
    Serial.println(CurieIMU.getGyroOffset(Z_AXIS));

    // To manually configure offset compensation values,
    // use the following methods instead of the autoCalibrate...() methods below
    //CurieIMU.setAccelerometerOffset(X_AXIS,495.3);
    //CurieIMU.setAccelerometerOffset(Y_AXIS,-15.6);
    //CurieIMU.setAccelerometerOffset(Z_AXIS,491.4);
    //CurieIMU.setGyroOffset(X_AXIS,7.869);
    //CurieIMU.setGyroOffset(Y_AXIS,-0.061);
    //CurieIMU.setGyroOffset(Z_AXIS,15.494);

    Serial.println("About to calibrate. Make sure your board is stable and upright");
    delay(5000);

    // The board must be resting in a horizontal position for
    // the following calibration procedure to work correctly!
    Serial.print("Starting Gyroscope calibration and enabling offset compensation...");
    CurieIMU.autoCalibrateGyroOffset();
    Serial.println(" Done");

    Serial.print("Starting Acceleration calibration and enabling offset compensation...");
    CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
    Serial.println(" Done");

    Serial.println("Internal sensor offsets AFTER calibration...");
    Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));
    Serial.print("\t"); // -76
    Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));
    Serial.print("\t"); // -2359
    Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS));
    Serial.print("\t"); // 1688
    Serial.print(CurieIMU.getGyroOffset(X_AXIS));
    Serial.print("\t"); // 0
    Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
    Serial.print("\t"); // 0
    Serial.println(CurieIMU.getGyroOffset(Z_AXIS));
  }
  
  pinMode(ledPin, OUTPUT);   // initialize the LED on pin 13 to indicate when a central is connected

  // BLE Setup
  /* Set a local name for the BLE device
     This name will appear in advertising packets
     and can be used by remote devices to identify this BLE device
     The name can be changed but maybe be truncated based on space left in advertisement packet */
  blePeripheral.setLocalName("IMUBLESketch");
  blePeripheral.setAdvertisedServiceUuid(axService.uuid());  // add the service UUID
  blePeripheral.addAttribute(axService);   // Add the BLE Heart Rate service
  blePeripheral.addAttribute(axChar); // add the Heart Rate Measurement characteristic

  /* Now activate the BLE device.  It will start continuously transmitting BLE
     advertising packets and will be visible to remote BLE central devices
     until it receives a new connection */
  blePeripheral.begin();
  
  Serial.println("Bluetooth device active, waiting for connections...");
  Serial.println(axService.uuid());
}

void loop() {
  // listen for BLE peripherals to connect:
  BLECentral central = blePeripheral.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(13, HIGH);

    // check the heart rate measurement every 200ms
    // as long as the central is still connected:
    while (central.connected()) {
      long currentMillis = millis();
      // if 200ms have passed, check the heart rate measurement:
      if (currentMillis - previousMillis >= 100) {
        previousMillis = currentMillis;
        updateTX();
      }
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(13, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

void updateTX() {
  // read raw accel/gyro measurements from device
  CurieIMU.readMotionSensor(ax, ay, az, gx, gy, gz);

  // these methods (and a few others) are also available

  //CurieIMU.readAcceleration(ax, ay, az);
  //CurieIMU.readRotation(gx, gy, gz);

  //ax = CurieIMU.readAccelerometer(X_AXIS);
  //ay = CurieIMU.readAccelerometer(Y_AXIS);
  //az = CurieIMU.readAccelerometer(Z_AXIS);
  //gx = CurieIMU.readGyro(X_AXIS);
  //gy = CurieIMU.readGyro(Y_AXIS);
  //gz = CurieIMU.readGyro(Z_AXIS);

  // display tab-separated accel/gyro x/y/z values
  Serial.print("a/g:\t");
  Serial.print(ax);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.print(az);
  Serial.print("\t");
  Serial.print(gx);
  Serial.print("\t");
  Serial.print(gy);
  Serial.print("\t");
  Serial.println(gz);
  int axm = ax + 32768; // move up the measurement so that range is entirely positive
  // Range of a/g values is [-32768, +32767]
  // we can send 2 bytes easily
  // let's lossily compress this a bit
  int axf = ceil(((double)axm)/256); // divide to a double then ceiling the result into an int
  int axmc = axm/axf; // stored as int
  if (axmc != axo || axf != axfo) {      // if the heart rate has changed
    Serial.print("AX is now: "); // print it
    Serial.println(axm);
    const unsigned char axCharArray[2] = { (char)axf, (char)axmc };
    axChar.setValue(axCharArray, 2);  // and update the heart rate measurement characteristic
    axo = axmc;    // save the level for next comparison
    axfo = axf;
  }

  // now for the other 5 values
  // AY
  int aym = ay + 32768;
  int ayf = ceil(((double)aym)/256);
  int aymc = aym/ayf;
  if (aymc != ayo || ayf != ayfo) {
    const unsigned char ayCharArray[2] = { (char)ayf, (char)aymc };
    ayChar.setValue(ayCharArray, 2);
    ayo = aymc;
    ayfo = ayf;
  }
  // AZ
  int azm = az + 32768;
  int azf = ceil(((double)azm)/256);
  int azmc = azm/azf;
  if (azmc != azo || azf != azfo) {
    const unsigned char azCharArray[2] = { (char)azf, (char)azmc };
    azChar.setValue(azCharArray, 2);
    azo = azmc;
    azfo = azf;
  }
  // GX
  int gxm = gx + 32768;
  int gxf = ceil(((double)gxm)/256);
  int gxmc = gxm/gxf;
  if (gxmc != gxo || gxf != gxfo) {
    const unsigned char gxCharArray[2] = { (char)gxf, (char)gxmc };
    gxChar.setValue(gxCharArray, 2);
    gxo = gxmc;
    gxfo = gxf;
  }
  // GY
  int gym = gy + 32768;
  int gyf = ceil(((double)gym)/256);
  int gymc = gym/gyf;
  if (gymc != gyo || gyf != gyfo) {
    const unsigned char gyCharArray[2] = { (char)gyf, (char)gymc };
    gyChar.setValue(gyCharArray, 2);
    gyo = gymc;
    gyfo = gyf;
  }
  // GZ
  int gzm = gz + 32768;
  int gzf = ceil(((double)gzm)/256);
  int gzmc = gzm/gzf;
  if (gzmc != gzo || gzf != gzfo) {
    const unsigned char gzCharArray[2] = { (char)gzf, (char)gzmc };
    gzChar.setValue(gzCharArray, 2);
    gzo = gzmc;
    gzfo = gzf;
  }
}
