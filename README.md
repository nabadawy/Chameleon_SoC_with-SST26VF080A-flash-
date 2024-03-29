# Chameleon_SoC_with-SST26VF080A-flash-
This project uses a python script to establish a communication between the SST26VF080A flash and the open Source [chameleon SoC](https://github.com/shalan/Chameleon_SoC) in two modes. 
First in the SPI mode to program the flash and second in the QUAD SPI mode to read the instruction from the flash.

## Flash Writer and UART master integration to the Chameleon SoC FROM [here](https://github.com/nabadawy/AMEN2S-SoC):
To establish a communication between the SoC and to the flash the Chameleon SoC architecture was updated as follow: 

![FW_diagram](https://github.com/nabadawy/Chameleon_SoC_with-SST26VF080A-flash-/blob/main/docs/Updated.drawio.png)

## Flash connections : 
As mentioned above we are communicating with the flash using the UART master. So, we are connecting the UART master bins ( the Rx and Tx) with a serial communication through the USB-TTL which is connected to the PC to read the instructions and the commands through a python script.

![connections](https://github.com/nabadawy/Chameleon_SoC_with-SST26VF080A-flash-/blob/main/docs/connections.png)

 ## Reading The JEDEC ID:
To test our connections, we initialize a JEDEC ID Read command by driving the select bin low then sends the JEDEC ID command cycle (9FH). Following the command cycle, SST26VF080A output data on the falling edge of the SCK signal and by using the python script with the mentioned connections we read the output ID. The following is the output of the analog Discovery kit which was used to debug and test the SPI signals: 

![JEDEC](https://github.com/nabadawy/Chameleon_SoC_with-SST26VF080A-flash-/blob/main/docs/ID.png)

## program the Flash (SPI Mode):
After connecting all wires, the python script can be used to program the flash in the SPI mode by using the page program command (02h) with the mem file ( mem file can be generated by [RISC-V GNU Compiler Toolchain](https://github.com/riscv-collab/riscv-gnu-toolchain)). 
To be able to program the flash, you should first unlock memory locations to be able to write to them. Unlocking those locations can be done be first writing the BP0, BP1, and BP2 to zero in the Status register. Then, the data for the selected page address must be in the erased state (FFH) before initiating the Page Program operation. So, a block erase command should be initialized first. Prior to the program operation, execute the WREN instruction. To execute a Page Program operation, the host drives CE# low then sends the Page Program command cycle (02H), three address cycles followed by the data to be programmed, then drives CE# high.
 

## Flash Fast Read (SPI Quad I/O Read):
The AHB-Lite Quad I/O flash reader is Intended to be used to execute from an external Quad I/O SPI Flash Memory. And the flash controller suppose that the external flash is in the SPI Quad I/O mood. SST26VF080A requires the IOC bit in the Configuration register to be set to ‘1’ prior to executing the command for the Fast Read. So, you should configure the configuration register first to set the IOC bit. 
## Note: 
To monitor the QSPI mode in the analyzer a [customized script](https://github.com/nabadawy/Chameleon_SoC_with-SST26VF080A-flash-/blob/main/docs/QuadSPI.txt) was written to be applied in the analyzer to be able to read the instructions. 

