# Firmware Build & Flashing Guide

This document provides instructions for compiling and flashing the STM32H723 micro-ROS robot firmware.

---

## 1. Prerequisites & Toolchain

- **ARM GCC Toolchain:** `arm-none-eabi-gcc` 12.3 (tested with GNU Tools for STM32 12.3.rel1 or official Arm GNU Toolchain 12.3.rel1).
- **Build System:** GNU Make (v4.0+).
- **Flashing Utility:** `STM32_Programmer_CLI` (from STM32CubeProgrammer) or `openocd`.

---

## 2. Setting Up the micro-ROS Static Library

The project relies on a prebuilt `libmicroros` archive containing ROS 2 Jazzy client libraries and standard message types (`geometry_msgs`, `sensor_msgs`, `std_msgs`).

### Option A — Quick Download (Recommended)

Run the automated fetch script to download and verify the verified release asset:

```bash
# Linux / macOS / Git Bash on Windows
./tools/fetch_libmicroros.sh
```

The script verifies SHA256 integrity and unpacks headers and libraries into `micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/`.

### Option B — Rebuilding via Docker

If you need custom ROS 2 messages or modified colcon build flags:

```bash
./tools/build_libmicroros.sh
```

---

## 3. Building the Firmware

Once `libmicroros` is present:

```bash
make clean
make -j$(nproc)
```

**Build Outputs:**
- `build/microros_H7.elf` — ELF binary with full debug symbols.
- `build/microros_H7.bin` — Raw flash image.
- `build/microros_H7.hex` — Intel HEX format.

---

## 4. Flashing the Board

### Using STM32CubeProgrammer CLI (SWD / ST-LINK)

```bash
STM32_Programmer_CLI -c port=SWD mode=UR -w build/microros_H7.bin 0x08000000 -v -rst
```

### Using OpenOCD

```bash
openocd \
  -f interface/stlink.cfg \
  -f target/stm32h7x.cfg \
  -c "program build/microros_H7.elf verify reset exit"
```

---

## 5. ⚠️ STM32CubeMX Regeneration Warning

If you modify `microros_H7.ioc` and regenerate code from STM32CubeMX, CubeMX will **clobber `Makefile`** and remove the micro-ROS and CMSIS-DSP configurations.

### How to Restore `Makefile` After Regeneration

Ensure the following blocks are present in `Makefile`:

#### 1. Include Paths (`C_INCLUDES`):
```makefile
C_INCLUDES += -Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include
```

#### 2. Extra Source Files (`C_SOURCES`):
```makefile
C_SOURCES += \
micro_ros_stm32cubemx_utils/extra_sources/custom_memory_manager.c \
micro_ros_stm32cubemx_utils/extra_sources/microros_allocators.c \
micro_ros_stm32cubemx_utils/extra_sources/microros_time.c \
micro_ros_stm32cubemx_utils/extra_sources/microros_transports/usb_cdc_transport.c
```

#### 3. DSP Library Directory & Flags (`LDFLAGS`):
```makefile
LIBDIR = -LDrivers/CMSIS/DSP/Lib/GCC
LIBS = -lc -lm -lnosys -larm_cortexM7lfdp_math
LDFLAGS = $(MCU) -specs=nano.specs -u _printf_float -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections
```

#### 4. Static Library Linkage:
```makefile
$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/libmicroros.a -o $@
	$(SZ) $@
```
