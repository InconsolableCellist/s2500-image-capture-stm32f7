# S-2500 Image Capture

This is firmware for the STM32F769NI to act as a digital image acquisition device for a Hitachi S-2500 scanning electron microscope from 1987.

* It was developed and tested on a 32F769IDISCO Discovery board by ST and connected to the S-2500 via specific points on the video amplifier board, which will be detailed in a schematic

* A THS7374 video amplifier was also connected between the DC-coupled SEM video amplifier output and the ADC in line on the STM32, mostly for voltage protection and removing the DC bias without excessive filtering of the signal

* The video signal as well as the X+Y and Y pulse lines were first attenuated using a potentiometer to ensure they were approximately in the TTL 0-3.3 voltage range for the THS7374 and STM32F7

* It uses six ADC configurations to optimize the image sampling time to provide the same resolution as the machine displays on its CRT. The modes are selected via host software

* All communication to/from the host is over USB HS (2.0)

* It transmits information about the row and pulse timing and its current mode, but this version doesn't integrate with the digital logic on the SEM to capture other useful parameters (working distance, acceleration voltage, magnification, emission current, etc.)

* It's controlled and its data is displayed with my [S-2500 Image Control Program](https://github.com/InconsolableCellist/s2500-image-viewer) program

* Debugging information is exposed by three pins on the STM32, useful for a logic analyzer to monitor program flow control or error states

* This is likely useful to absolutely no one else
