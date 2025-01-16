Rez BT

A recreation of the Playstation 2 (PS2) Trance Vibrator for the game Rez that connects via Bluetooth BLE to Amorelie Joy vibrators.

![grafik](https://github.com/user-attachments/assets/52550f91-dbe0-48b3-9552-bc67efa5f204)
![grafik](https://github.com/user-attachments/assets/09dcf29f-b90f-4467-a844-ee38471de083)


Implemented using esp-idf for the Waveshare ESP32-S3-Zero:
https://www.waveshare.com/esp32-s3-zero.htm

The code makes use of the onboard LED for showing changes and bluetooth pairing status.

Additionally the boot button is used to support the following functions:
* Long press to restart the pairing mode
* Press once to toggle the game mode:
  * Direct: Will keep the last received vibration strength (like the original device)
  * Fade: Will ignore any "off" signal from the game and instead fade out the current vibration level
 
The STL folder contains a 3d printable case for the device that exposes the LED and allows pressing the boot button.

The implementation currently supports the following amorelie devices:
* 0x4D02 Amorelie Joy Move
* 0x4D05 Amorelie Joy Cha-Cha
* 0x4D06 Amorelie Joy Boogie
* 0x4D01 Amorelie Joy Shimmer
* 0x4D03 Amorelie Joy Grow
* 0x4D04 Amorelie Joy Shuffle
* 0x4D07 Amorelie Joy Salsa

![grafik](https://github.com/user-attachments/assets/5004fa70-69af-4b4b-9b45-9fe71b7b7a72)
