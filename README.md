## Electronic Control Module for PUT Solar Dynamics
Based on STM32F406VGT6 microprocessor

The module is receiving data from the whole CAN network and then automatically transfers every data received over USB-C interface according to pattern `CAN-ID[DLC]DATA`. The interface works as virtual COM port and can be read by every serial port reading software. 
Besides CAN, the `OPTO-INPUT' pins are connected to light controlling panel with interrupt on every state change. 

# To-do list
- [ ] implement SD card service in order to store bus traffic
- [ ] conversion onto RTOS (optional)
- [ ] continue developing `CANopen` functions in order to extend network management


# Short description
- Configuration is fully done in `CubeMX` and can be there continued without conflicts.
- Workflow is done according to `Gitflow` standard.
- The source codes are available in `Core/Src` and `Core/Inc` folders.
- Electrical schematics can be found in `psd_ecm_hardware.pdf` file. The hardware was projected by by Konrad Gieregowski and Jan Węgrzynowski from PSD.
- Nodes naming and network organization is done according to `CANopen` standard. For more info visit `https://www.csselectronics.com/screen/page/canopen-tutorial-simple-intro/language/en`

# Usage
- All of the peripherial functions can be found inside appropriate source file, e.g. CAN functions can be found in `can.c` file. 
- Currently data are being stored by `SYNC` frame sent through an interrupt. It is important to set interrupt priority.
