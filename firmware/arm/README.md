# Arm Firmware

## Getting Started

#### Install Arduino CLI and dependencies
```bash
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/.local/bin sh
echo "~/.local/bin" >> ~/.bashrc && source ~/.bashrc
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli lib install circularbuffer servo
```

#### Compile and upload
```bash
arduino-cli compile --upload --port /dev/ttyUSB0 -b arduino:avr:mega
```