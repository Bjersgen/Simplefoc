# Simplefoc
Update 2023.3.9:

Firstly, there are some facts that need to be clarified, which I was not aware of when I initially started this project. Although the performance of these PCBs is normal at low speeds. After repeated verification, it was found that the project based on ESP32 can only reach a speed of 1000RPM when running a BLDC motor with an internal resistance of 1.5 ohms. When the speed continues to increase, the motor will exhibit symptoms such as jitter, increased current, reduced efficiency, and heat generation. This is due to several reasons working together.

1、The code of SimpleFOC platform itself has certain limitations. Because SimpleFOC is based on platforms such as Arduino, the core of the code, which is the SVPWM part, is placed in the main loop. This causes the waveform of the final output of the three phases to become stair-shaped when the speed is too high, and the processing frequency of the MCU cannot guarantee a sinusoidal waveform.

2、Do not use I2C communication for the AS5600 encoder as it is slow to trigger and has a delay, and the communication frequency has a great impact on the main loop of FOC.

3、Esp32 or Arduino itself does not have powerful features such as interrupts and DMA like stm32, and adding these functions to the underlying code of SimpleFOC is also difficult.

To address the issue of FOC being unable to run at high speeds, the following improvement solutions are recommended:

1、Change the encoder to AS5047 and use SPI or PWM, paying attention to signal integrity.

2、Use stm32 series as the main control MCU, utilize interrupts to process the SVPWM part, and trigger the interrupt every time the encoder's reading is obtained to process the direction and positioning of the magnetic field, which can greatly improve the robustness of motor control.




Feature——MAX_120W(24V*5A)——Faster(1KHZ for closed loop)

           SimpleFOC——
                            | ——BLDCmotor(self made DIY motor)
                            | ——Documents
                            | ——STM32H7_FOC
                                 |——circle(PCBs & libraries)
                            		  |——Shell(a PCB shell)
                                 |——code(using Keil5 & STM32CubeMX)
                            		  |——Inc
                            		  |——Src
                            		  |——Drivers
                            		  |——MDK-ARM
                                 |——datasheet(ICs datasheets)
                                 |——image
                                 |——readme
                            | ——pico(basic demo)                 
                            | ——pico-drv8313
                            | ——pico-drv8313 with foc
                            | ——readme


​                        


Most projects are based on Arduino, esp32-pico D4 and one project is based on stm32h743.


