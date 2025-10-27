# Bachelor
[Thesis](./Bachelor_Rapport.pdf)

src includes the final code.
To run project, download src files into workspace and setup CMake (or use CMakeLists.txt from src folder).
Code is flashed in ubuntu using the Raspberry Pi Pico extension in VScode. 
Link: https://www.raspberrypi.com/news/get-started-with-raspberry-pi-pico-series-and-vs-code/

The code uses a fatfs library to datalog onto a microSD card. This library is accessed through git in the pico_sdk_import.cmake file.

The project used the programmable ESC, Reely Sky-Series 20A. The test_functions.cpp uses a programming_ESC function which is used to program this specific ESC model.

The BLDC motor used in this project is the EMAX GT2218/09 1100Kv.



