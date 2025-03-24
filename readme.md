# Goal 

To make the two way communication work between both devices
Nice to have: WOR wake-up reception

# Folders

- cc1101_only -> Rx loop
- cc1101_tx -> Tx loop (same driver, same ESP model, only different serial chip)
- CC1101_RF-master -> driver basis from Arduino. I changed:
    - Got rid of multiple while(1) cycles inside the code
    - Changed strobe function to read its state
    - Changed the configuration a bit, but I also tried without changes. If you want to revert its config, change all the functions called by the cc1101_begin to the original values provided by this driver
- SmartRC-CC1101-Driver-Lib-master -> alternative driver. Although I did not use this one, I saw that they used the status from GDO0 and GDO2 a bit more. But I don't like as much the driver itself
-> cc1101.pdf -> the datasheet of the transceiver
- Pinout of ESP32 device used
- Pinout of the transceiver

# Hardware
- 2xESP32 (DevKitC1, if I recall correctly. Pinout picture attached)
- This cc1101 chip (only works at 500 kHz unlike specified in the cc1101 datasheet) -> https://www.amazon.co.uk/Ebyte-E07-M1101D-TH-Wireless-Transmitter-Receiver/dp/B07P6L8ZLZ?th=1
- I have two extra units (still in the package), which I can change to if you think the problem is the transceivers I am using

# Pin Connections

- CSN -> GPIO18
- GDO0 -> GPIO4 (unused in the driver, feel free to use it if it makes a difference)
- GDO2 -> GPIO5 (unused in the driver, feel free to use it if it makes a difference)
- MISOpin -> GPIO15 (extra pin that checks if MISO is low)
- MISO -> GPIO12
- MOSI -> GPIO13
- SCK -> GPIO14

# Other Implementation Details

- I checked the communication with SPI via a logic analyser, and so far everything looked OK. Values were read properly. One of the most important functions is getState. You can also add a log there and you will see that the SPI frames are fine
- Only thing I never saw was the GDO0 asserting to 1. But I saw that pin return a square wave when left uninitialized
- Underlying SPI driver uses mutexes because it's shared between two devices on the same bus, that use different tasks. So far, that never caused trouble

# Possible Clues

- FIFO size threshold
- I have tried to configure the device both to detect preamble on WOR and complete packets, neither worked

