# Chameleon_SoC_with-SST26VF080A-flash-
this project uses a python script to establish a communication between the SST26VF080A flash and the open Source [chameleon SoC](https://github.com/shalan/Chameleon_SoC) in two modes. 
First in the SPI mode to program the flash and second in the QUAD SPI mode to read the instruction from the flash.

## Flash Writer and UART master integration to the Chameleon SoC:
To establish a communication between the SoC and to the flash the Chameleon SoC architecture was updated as follow: 
![FW_diagram]()

## Program the flash (SPI mode): 
