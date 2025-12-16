# 🎯 ESP32 Project Work

![Projects](https://img.shields.io/badge/Projects-1-blue)
![ESP--IDF](https://img.shields.io/badge/ESP--IDF-v5.5-green)
![Platform](https://img.shields.io/badge/Platform-ESP32-red)

Comprehensive collection of ESP32 microcontroller projects using ESP-IDF framework, covering Wi-Fi, Bluetooth, IoT applications, and peripheral interfacing.

---

## 📑 Quick Links
- [Development Environment Setup](#development-environment-setup)
- [Repository Structure](#repository-structure)
- [Getting Started](#getting-started)
- [Common Commands](#common-commands)
- [Hardware Requirements](#hardware-requirements)
- [Troubleshooting](#troubleshooting)
- [Documentation](#documentation)

---

## Development Environment Setup

### ESP-IDF Installation (Ubuntu 24.04)

**Required Version:** ESP-IDF v5.5

#### Step 1: Install Prerequisites
```bash
sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv \
    cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
```

#### Step 2: Clone ESP-IDF
```bash
mkdir -p ~/esp
cd ~/esp
git clone -b v5.5 --recursive https://github.com/espressif/esp-idf.git
```

#### Step 3: Install ESP-IDF Tools
```bash
cd ~/esp/esp-idf
./install.sh esp32
```

#### Step 4: Set Up Environment Variables
```bash
# Add to ~/.bashrc
echo 'alias idf-5.5=". $HOME/esp/esp-idf/export.sh"' >> ~/.bashrc
source ~/.bashrc
```

#### Step 5: Activate ESP-IDF
```bash
idf-5.5
```

**Verification:**
```bash
idf.py --version
# Expected output: ESP-IDF v5.5.x
```

---

## Repository Structure
```
esp32/
├── 001_Hello_world/              # Basic LED blink using GPIO
└── README.md                     # This file
```

---

## Getting Started

### Prerequisites
- Ubuntu 24.04 LTS (or compatible Linux distribution)
- ESP-IDF v5.5
- ESP32 Development Board
- USB cable (USB-C or Micro-USB depending on board)
- Python 3.8 or higher

### Clone Repository
```bash
git clone https://github.com/rahulbari717/embedded-systems-toolkit.git
cd embedded-systems-toolkit/esp32
```

### Build and Flash Project

#### 1. Navigate to Project Directory
```bash
cd 001_Hello_world
```

#### 2. Activate ESP-IDF Environment
```bash
idf-5.5
```

#### 3. Configure Project (Optional)
```bash
idf.py menuconfig
```

#### 4. Build Project
```bash
idf.py build
```

#### 5. Flash to ESP32
```bash
idf.py -p /dev/ttyUSB0 flash
```

#### 6. Monitor Output
```bash
idf.py -p /dev/ttyUSB0 monitor
```

#### Combined Command (Flash + Monitor)
```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

**Exit Monitor:** Press `Ctrl + ]`

---

## Common Commands

### Project Management
```bash
# Create new project
idf.py create-project <project-name>

# Set target chip
idf.py set-target esp32

# Clean build
idf.py fullclean
```

### Configuration
```bash
# Open menuconfig
idf.py menuconfig

# Save configuration
idf.py save-defconfig
```

### Building & Flashing
```bash
# Build only
idf.py build

# Flash only
idf.py -p /dev/ttyUSB0 flash

# Erase flash
idf.py -p /dev/ttyUSB0 erase-flash

# Monitor serial output
idf.py -p /dev/ttyUSB0 monitor
```

### Debugging
```bash
# Check partition table
idf.py partition-table

# View app size
idf.py size

# Generate documentation
idf.py docs
```

---

## Hardware Requirements

| Component | Specification |
|-----------|--------------|
| **Development Board** | ESP32-DevKitC, ESP32-WROOM-32 |
| **USB Cable** | USB-C or Micro-USB |
| **Power Supply** | 5V via USB or 3.3V regulated |
| **Optional** | Breadboard, jumper wires, sensors |

---

## Troubleshooting

### Common Issues

#### 1. Port Not Found
```bash
# List available ports
ls /dev/tty*

# Add user to dialout group
sudo usermod -aG dialout $USER
# Logout and login
```

#### 2. Permission Denied
```bash
sudo chmod 666 /dev/ttyUSB0
```

#### 3. Flash Failed
```bash
# Hold BOOT button and press RESET
# Then release BOOT button
idf.py -p /dev/ttyUSB0 flash
```

#### 4. Old Tools Warning
```bash
# Remove outdated tools
python ~/esp/esp-idf/tools/idf_tools.py uninstall

# Remove installation packages
python ~/esp/esp-idf/tools/idf_tools.py uninstall --remove-archives
```

#### 5. Python Dependencies Error
```bash
cd ~/esp/esp-idf
./install.sh esp32
```

---

## Documentation

- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
- [ESP32 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf)
- [ESP32 Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf)
- [ESP-IDF API Reference](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/index.html)

---

## 🤝 Connect With Me

- **LinkedIn:** [linkedin.com/in/rahul-bari-embeddeddeveloper](https://linkedin.com/in/rahul-bari-embeddeddeveloper)
- **GitHub:** [@rahulbari717](https://github.com/rahulbari717)
- **Email:** rahulbari717@gmail.com

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](../LICENSE) file for details.

---

## ⭐ Support

If you find this repository helpful:
- ⭐ Star this repo
- 🔀 Fork and contribute
- 📢 Share with fellow IoT enthusiasts
- 💬 Open issues for discussions

---

**Happy ESP32 Programming! 🚀**

*Last Updated: December 2025*
```