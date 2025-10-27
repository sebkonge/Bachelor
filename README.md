# Bachelor
[Thesis](./Bachelor_Rapport.pdf)

[YouTube](https://www.youtube.com/watch?v=11JPB92M4Zs&list=PLOyESx9PWsiB_6e26h_a_aPU2y4m0XSPj)

src includes the final code.
To run project, download src files into workspace and setup CMake (or use CMakeLists.txt from src folder).
Code is flashed in ubuntu using the [Raspberry Pi Pico extension](https://www.raspberrypi.com/news/get-started-with-raspberry-pi-pico-series-and-vs-code/) in VScode. 

The code uses a fatfs library to datalog onto a microSD card. This library is accessed through git in the pico_sdk_import.cmake file.

The project used the programmable ESC, Reely Sky-Series 20A. The test_functions.cpp uses a [programming_ESC](./src/test_functions.cpp) function which is used to program this specific ESC model.

The BLDC motor used in this project is the EMAX GT2218/09 1100Kv.



