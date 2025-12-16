
# Embedded Systems Toolkit: STM32 & ESP32 Development work

![Platforms](https://img.shields.io/badge/Platforms-STM32%20%7C%20ESP32-blue)
![Skills](https://img.shields.io/badge/Skills-Embedded%20C%20%7C%20BareMetal%20%7C%20HAL%20%7C%20Drivers%20%7C%20ESP--IDF-green)
![License](https://img.shields.io/github/license/rahulbari717/embedded-systems-toolkit?color=orange)

A unified, hands-on workspace demonstrating core concepts of professional firmware development across **ARM Cortex-M (STM32)** and **modern IoT platforms (ESP32)**. This repository focuses on strong fundamentals, custom driver development, and vendor-specific toolchains.

---

## 🎯 Core Focus & Architecture

This repository is structured as a mono-repo toolkit, isolating platform-specific development in separate folders, each with its own comprehensive documentation.

| Platform Track | Core Architecture | Key Focus Areas |
| :--- | :--- | :--- |
| **STM32** | ARM Cortex-M4 (STM32F446RE) | **Bare-Metal Drivers**, Register-Level Programming, GPIO, SPI, I2C, RTC, DMA, CAN, Timers, RTOS concepts, HAL.  |
| **ESP32** | Xtensa LX6 (ESP32) | **ESP-IDF v5.5**, Build/Flash Workflow, Wi-Fi/BLE Foundation, Industrial IoT implementation. |

---

## 🌟 Featured Skillsets

### 1. STM32 Track: Deep Dive into Microcontroller Internals

This track focuses on writing efficient, low-level firmware for the STM32F446RE.

* **Custom Driver Development:** Complete register-level drivers for **GPIO, SPI, I2C, USART, and RCC** (Clock Control).
* **Peripheral Mastery:** Hands-on projects covering DMA, Timers, CAN communication, and advanced clock configuration.
* **Architecture:** Practical exploration of Cortex-M4 features like Operational Modes, Access Levels, and Fault Generation.

**➡️ Start Here:** **`stm32/README.md`** (Includes environment setup using STM32CubeIDE)

### 2. ESP32 Track: Modern Connectivity & IoT

This track uses the official Espressif IoT Development Framework (ESP-IDF) for robust, production-ready IoT solutions.

* **ESP-IDF Workflow:** Mastery of the full `idf.py` lifecycle: setup, configuration, building, flashing, and monitoring.
* **Networking Foundation:** Essential setup and structure for Wi-Fi and Bluetooth Low Energy (BLE) applications.
* **Toolchain:** Tested and verified on **ESP-IDF v5.5** (Linux/Ubuntu).

**➡️ Start Here:** **`esp32/README.md`** (Includes ESP-IDF installation guide)

---


## 🛠 Development Environment

| Platform | IDE/SDK | OS |
| :--- | :--- | :--- |
| **STM32** | STM32CubeIDE 1.18.0 | Ubuntu 24.04 |
| **ESP32** | ESP-IDF v5.5 | Ubuntu 24.04 |

---

## 🚀 Getting Started

To dive in, simply clone the repository and navigate to your desired platform folder:

```bash
git clone [https://github.com/rahulbari717/embedded-systems-toolkit.git](https://github.com/rahulbari717/embedded-systems-toolkit.git)
cd embedded-systems-toolkit

# Choose a platform:
cd stm32  OR  cd esp32
```

## 🎯 About This Repository

This repository is structured as a **toolkit**, not a single project. Each folder is a focused learning and implementation track:

* **STM32**: Cortex-M4 architecture, register-level drivers, HAL, DMA, timers, CAN, RTC, and FreeRTOS concepts.
* **ESP32**: ESP-IDF based projects covering GPIO, build/flash workflows, and foundations for Wi-Fi, BLE, and IoT systems.

---

## 🧭 Repository Overview

```
embedded-systems-toolkit/
│
├── README.md              # You are here
├── LICENSE
├── .gitignore
│
├── stm32/                 # STM32 Cortex-M projects & drivers
│   └── README.md          # Detailed STM32 documentation
│
├── esp32/                 # ESP32 ESP-IDF based projects
│   └── README.md          # Detailed ESP32 documentation
│
└── .vscode/               # Editor configuration
```

Each platform has its **own README** with setup steps, project structure, and usage instructions. Start here, then dive into the platform you want.

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

*Last Updated: December 2025*
