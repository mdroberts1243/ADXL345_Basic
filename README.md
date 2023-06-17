
   This program provides a simple polling mode driver and example
   for the ADXL345 Accelerometer from Analog Devices.
  
   It is designed for ESP-IDF and tailored to the ESP32S2 on the FSPI
   interface (if you don't modify the .h file)
 
   It is adapted from the original no OS driver from Analog Devices
   (license in files).
  
   It is very basic and stripped down in functionality.
  
   Some things to improve:
    -- Get rid of need for global adxl handle
    -- Develop a device structure to hold the handle, and the device settings for easy reference/checking
    -- De-initialize to unallocate memory, etc. if used above.
    -- Add interrupt support
    -- Add feature support (activity, tap, inactivity, freefall, etc.)
  
   Mark Roberts (mdroberts1243@gmail.com)
   Modifications are Copyright (c) 2023 Mark Roberts
 
