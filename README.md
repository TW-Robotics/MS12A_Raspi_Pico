# Raspberry Pico Code for control of the MS12A SmartServos

Assuming you have your pico environment already set up. Otherwise check out [this link to the official getting started guide](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf).

**Keep in mind a level shifter/voltage regulator is needed for the Pico (3,3volts) to communicate (via UART) with the servos (5,0volts)!**

1. `git clone https://github.com/TW-Robotics/MS12A_Raspi_Pico.git` to your home directory or something
2. `mkdir build && cd build` in root folder of repo
3. `cmake ..`
4. `make -j`
5. Copy *ms12apico.uf2* file from the build folder to the Raspberry Pico (must be connected via mass storage mode)
