![Picture of the constructed quadcopter, without propellers](/images/constructed_quad.jpg)

### In this project, I am designing a quadcopter drone and flight computer that is capable of FPV-style flight, camera drone 'hover'-style flight, and a GPS return flight mode. Dubbed the Flying Opossum, I have developed this quadcopter as much from scratch as I am personally capable of. 

### I designed the drone's frame in Fusion 360, and 3D Printed it in PETG plastic. I designed an ESP32 powered flight computer, fitted with a gyroscope/accelerometer/magnetometer, barometer, and GPS modules, all housed on a custom-designed PCB. I am writing the flight computer software with minimal additional libraries, writing drivers for the gyroscope, accelerometer, magnetometer, and barometer myself.

## Completed Features
3D printing, soldering, construction
All sensor drivers
Custom PWM input driver
FPV-Style flight mode
Mahoney Filter for Attitude calculation
PID Loops to pursue level flight in 'Hover' mode

## WIP Features
GPS Home setting and return mode

## Components Used
- 1x ESP32D 38 Pin - https://a.co/d/9OTzeMg
- 1x BMP-280 Module - https://a.co/d/dHeIxgT
- 1x Adafruit LSM9DS0 Module - https://www.adafruit.com/product/2021 (superseded by the LSM9DS1 - https://www.adafruit.com/product/3387)
- 1x GT-U7 GPS Module - https://a.co/d/90ruPHi
- 1x Flysky FS-i6X Radio Transmitter and Receiver - https://a.co/d/17PjGrW
- 1x (4-pack) Readytosky 2-4S 40A ESCs - https://a.co/d/7DUserc
- 2x CW & 2x CCW EMAX MT2205II 2300KV Motors - https://emaxmodel.com/products/mt2205%E2%85%B1-2300kv-brushless-motor-with-cw-ccw-thread-options
- 1x Set of 7.5x3.7x3 Propellers - https://a.co/d/4cGYdIU
- 1x FPV Drone Camera - https://a.co/d/aPriDum
- 1x Cheap FPV Receiver - https://a.co/d/4WXVRuR
- 1x 3S Lipo Battery (in all honesty, this battery is really big for this quad) - https://a.co/d/b9KI3hO
- 1x Set of M2 & M3 Anti-Vibration Standoffs - https://a.co/d/4dxyd9B
- 1x Set of M5 CW & CCW Nylon lock nuts - https://a.co/d/5ZPtKfp
- 1x Set of 3.5mm Bullet Connectors - https://a.co/d/6E5sy8v
- 1x Set of M3 Threaded inserts - https://a.co/d/2u8rPgd
- Set of M3 and M2 screws of varying lengths
- 1kg PETG Filament - https://a.co/d/fzmQgO0
