# Embedded Systems Toolkit

Comprehensive collection of STM32 and ARM Cortex-M projects covering embedded C, peripheral drivers, FreeRTOS, DMA, and real-time systems. Complete course implementations with documented code examples.

---

## 🎯 About This Repository

This repository contains hands-on implementations and practical examples for embedded systems development on STM32 microcontrollers, covering fundamental concepts to advanced topics like RTOS and DMA programming.

---

## 🛠 Development Environment Setup

### STM32CubeIDE Installation (Ubuntu 24.04)

**Required Version:** STM32CubeIDE 1.18.0 (Build: 24413_20250227_1633)
```bash
# Download from ST official website
# https://www.st.com/en/development-tools/stm32cubeide.html

# Install the .deb package
sudo dpkg -i st-stm32cubeide_1.18.0_24413_20250227_1633_amd64.deb

# Fix dependencies if needed
sudo apt-get install -f

# Launch
stm32cubeide
```

**Post-Installation:**
- Configure udev rules for ST-Link: `sudo usermod -aG dialout $USER`
- Logout and login for group changes to take effect
- Verify version: Help → About STM32CubeIDE

---

## 📁 Repository Structure
```
embedded-systems-toolkit/
│
├── README.md
├── LICENSE
│
├── 01-embedded-c-fundamentals/
│   ├── README.md
│   ├── basic-syntax/
│   ├── pointers/
│   ├── bit-manipulation/
│   ├── structures-unions/
│   └── volatile-const/
│
├── 02-arm-cortex-m4-architecture/
│   ├── README.md
│   ├── processor-modes/
│   ├── stack-operations/
│   ├── interrupts-exceptions/
│   ├── memory-map/
│   └── register-programming/
│
├── 03-peripheral-driver-development/
│   ├── README.md
│   ├── gpio-driver/
│   ├── spi-driver/
│   ├── i2c-driver/
│   ├── uart-driver/
│   └── interrupt-handling/
│
├── 04-timers-pwm-can-rtc/
│   ├── README.md
│   ├── timer-basics/
│   ├── pwm-generation/
│   ├── input-capture/
│   ├── can-communication/
│   ├── rtc-implementation/
│   └── low-power-modes/
│
├── 05-freertos-implementation/
│   ├── README.md
│   ├── task-management/
│   ├── queue-communication/
│   ├── semaphores-mutex/
│   ├── software-timers/
│   ├── memory-management/
│   └── debugging-techniques/
│
├── 06-dma-programming/
│   ├── README.md
│   ├── dma-basics/
│   ├── memory-to-memory/
│   ├── peripheral-to-memory/
│   ├── dma-interrupts/
│   └── circular-mode/
│
├── 07-lcd-tft-lvgl/
│   ├── README.md
│   ├── ltdc-configuration/
│   ├── lcd-interfacing/
│   ├── lvgl-basics/
│   ├── touchscreen/
│   └── gui-projects/
│
└── projects/
    ├── integrated-projects/
    ├── real-world-applications/
    └── course-assignments/
```

---

## 📚 Topics Covered

### 1️⃣ Embedded C Fundamentals
- C language basics for embedded systems
- Pointers and memory management
- Bit manipulation techniques
- Structures and unions
- Volatile and const keywords

### 2️⃣ ARM Cortex-M4 Architecture
- Processor architecture and modes
- Stack operations (MSP/PSP)
- Exception and interrupt handling
- Memory mapping
- Register-level programming

### 3️⃣ Peripheral Driver Development
- GPIO driver implementation
- SPI protocol and driver
- I²C protocol and driver
- UART/USART communication
- Interrupt handling

### 4️⃣ Timers, PWM, CAN & RTC
- Timer configurations
- PWM generation
- Input capture techniques
- CAN bus communication
- RTC implementation
- Low-power modes

### 5️⃣ FreeRTOS Implementation
- Task management and scheduling
- Queue-based communication
- Semaphores and mutexes
- Software timers
- Memory management
- Debugging with SEGGER SystemView

### 6️⃣ DMA Programming
- DMA controller basics
- Memory-to-memory transfers
- Peripheral-to-memory operations
- Interrupt handling
- Circular mode implementation

### 7️⃣ LCD-TFT & LVGL
- LTDC configuration
- LCD interfacing
- LVGL graphics library
- Touchscreen integration
- GUI development

---

## 🚀 Getting Started

### Prerequisites
- STM32CubeIDE 1.18.0 or later
- STM32 Development Board (F4 series recommended)
- ST-Link debugger
- Git installed

### Clone Repository
```bash
git clone https://github.com/rahulbari717/embedded-systems-toolkit.git
cd embedded-systems-toolkit
```

### Open Projects
1. Launch STM32CubeIDE
2. File → Import → Existing Projects into Workspace
3. Select desired project folder
4. Build and flash to your board

---

## 🛠 Hardware Requirements

- **Development Boards:** STM32F407 Discovery, STM32F446RE Nucleo
- **Debugger:** ST-Link V2 or higher
- **Tools:** Logic Analyzer, Oscilloscope, Multimeter (optional)
- **Peripherals:** Sensors, displays, communication modules

---

## 🤝 Connect With Me

- **LinkedIn:** [linkedin.com/in/rahul-bari-embeddeddeveloper](https://linkedin.com/in/rahul-bari-embeddeddeveloper)
- **GitHub:** [@rahulbari717](https://github.com/rahulbari717)
- **Email:** rahulbari717@gmail.com

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## ⭐ Support

If you find this repository helpful:
- ⭐ Star this repo
- 🔀 Fork and contribute
- 📢 Share with fellow embedded enthusiasts
- 💬 Open issues for discussions

---

**Happy Embedded Programming! 🚀**

*Last Updated: November 2025*
