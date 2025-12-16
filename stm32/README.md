# 🎯 STM32 Project Work

![Projects](https://img.shields.io/badge/Projects-20+-blue)
![IDE](https://img.shields.io/badge/IDE-STM32CubeIDE%20v1.18-green)
![Platform](https://img.shields.io/badge/Platform-STM32F446RE-red)

Comprehensive collection of **STM32F446RE** microcontroller projects covering basics to advanced peripheral programming, **HAL drivers**, **bare-metal programming**, and custom driver development.

---

## 📑 Quick Links
- [Development Environment Setup](#development-environment-setup)
- [Repository Structure](#repository-structure)
- [Custom Driver Development](#custom-driver-development)
- [Getting Started](#getting-started)
- [Hardware Requirements](#hardware-requirements)
- [Documentation](#documentation)

---

## Development Environment Setup

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

## Repository Structure
```
stm32/
├── 001_Hello_world/                      # Basic LED blink (bare-metal)
├── 002_Operational_modes/                # Processor modes exploration
├── 003_Access_level/                     # Privileged vs unprivileged access
├── 004_Fault_gen/                        # Fault generation and handling
├── 005_Task_schedular/                   # Simple round-robin scheduler
├── 006_baremetal_embedded_c/             # Bare-metal programming
├── 006_Debuging/                         # Debugging techniques
├── 007_HAL/                              # HAL library introduction
├── 008_UART_HAL/                         # UART polling mode
├── 009_UART_HAL_IT/                      # UART interrupt mode
├── 010_HSE_CLK_8M/                       # External 8MHz HSE clock
├── 011_PLL_SYSCLK/                       # PLL clock configuration
├── 012_PLL_SYSCLK_HSE/                   # PLL with HSE source
├── 013_TIM6_100ms/                       # Timer 6 polling mode
├── 014_TIM6_100ms_IT/                    # Timer 6 interrupt mode
├── 015_CAN_LoopBack_Mode/                # CAN loopback testing
├── 016_Sleep_ON_exit_TIM6_100ms_IT2/     # Sleep-on-exit mode
├── 017_RTC_Date_Time/                    # Real-time clock
├── 018_GPIO_HAL_DMA/                     # DMA with GPIO (polling)
├── 019_GPIO_HAL_DMA_IT/                  # DMA with interrupts
├── 01-embedded-c-fundamentals/           # C programming basics
└── stm32f4xx_drivers/                    # Custom peripheral drivers
```

---

## Custom Driver Development
### stm32f4xx_drivers/

Complete peripheral driver suite implementing register-level programming:

### Driver Implementation:

- stm32f446xx_gpio_driver.c - GPIO configuration and control
- stm32f446xx_spi_driver.c - SPI master/slave communication
- stm32f446xx_i2c_driver.c - I2C protocol implementation
- stm32f446xx_usart_driver.c - USART serial communication
- stm32f446xx_rcc_driver.c - Clock control and configuration

### Example Applications:
```
drivers/Src/
├── 001_ledToggle.c              # GPIO driver demonstration
├── 004_spi_tx_testing.c         # SPI loopback test
├── 005_spi_txonly_esp32rx.c     # SPI communication with ESP32
├── 006_i2c_master_tx.c          # I2C master mode transmission
└── 008_DMA_main_m2p_UART2.c     # DMA memory-to-peripheral UART
```

## Getting Started

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

## Hardware Requirements

- **Development Boards:** STM32F446RE Nucleo
- **Debugger:** ST-Link V2 or higher
- **Tools:** Logic Analyzer, Multimeter (optional)
- **Peripherals:** Sensors, displays, communication modules

---

## Documentation

- [STMicroelectronics Official Website](https://www.st.com/content/st_com/en.html)
- [STM32F446RE Datasheet](https://www.st.com/resource/en/datasheet/stm32f446re.pdf)
- [STM32F4 Reference Manual (RM0390)](https://www.st.com/resource/en/reference_manual/rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [STM32CubeIDE User Guide](https://www.st.com/resource/en/user_manual/um2609-stm32cubeide-user-guide-stmicroelectronics.pdf)

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
- 📢 Share with fellow embedded enthusiasts
- 💬 Open issues for discussions

---

**Happy Embedded Programming! 🚀**

*Last Updated: December 2025*
