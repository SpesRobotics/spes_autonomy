# Arm Firmware

## Getting Started

#### Install Arduino CLI and dependencies
```bash
sudo adduser $USER dialout
sudo snap install arduino-cli
sudo apt remove brltty
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli lib install circularbuffer servo
```

#### Compile and upload
```bash
arduino-cli compile --upload --port /dev/ttyUSB0 -b arduino:avr:mega
```
