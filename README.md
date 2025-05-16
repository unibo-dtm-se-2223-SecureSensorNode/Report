# 1. Introduction

# 1. Architecture Overview

> **Note on Timeline**
> This chapter was written after the initial development work had already been completed.
> In fact, before working on the sensor or architecture, we started with fundamental experiments on LED blinking.
> These served as a warm-up phase to test STM32CubeIDE, verify the build process, and explore three timing strategies — from blocking delay to SysTick interrupts.
>
> These early milestones are fully documented and tagged in the repository (`v1`, `v2`, `v3`).

---

The SecureSensorNode project is a complete end-to-end system designed to demonstrate how a modern engineer can build a modular, secure, and efficient embedded solution — with real hardware in the loop.

At its core, the system consists of:

- a **client node**, implemented on an STM32 microcontroller (Nucleo-F401RE board), tasked with acquiring, encrypting, and sending environmental data from a BMP280 (pressure and temperature) sensor;
- a **server**, written in Python, responsible for receiving, decrypting, and exposing those data via a RESTful API.

The communication between the two platforms happens via UART — a simple serial protocol that intentionally lacks any native encryption: this gives us the opportunity to implement **application-level security**.

This report documents every step of the project — and not just the code itself but every reasoned design choice behind it.
Because in software engineering, **code without rationale is just copy-paste**.

---

## 1.1 Overall Design Approach

From the very beginning, we agreed that:

- The client node would **push** data to the server (rather than wait for requests).
  This allows for:
  - simpler architecture (no command parser on the client),
  - less communication overhead (no request-response roundtrips),
  - easier scaling (clients operate independently),
  - and higher security control (clients own the data flow).

- The server must **verify and expose** data over a REST interface — this makes integration with external systems and dashboards trivial.

- The data must be **sent in encrypted form**, because UART offers no protection. The goal is to ensure confidentiality even if the physical link is intercepted.

- The whole system must be **buildable, flashable, and testable** without relying on STM32CubeIDE. We want automation, portability, and DevOps-aligned workflows.

---

## 1.2 High-Level Structure

The system is intentionally modular:

- `client/firmware/` contains the STM32 HAL-based firmware (written in C),
- `server/` will contain a microservice written in Python (to be implemented),
- `automation/` holds custom scripts (e.g., for flashing via st-link),
- Git is used for version control, with meaningful commits and Git tags marking development stages.

---

## 1.3 Communication Protocol

The client sends plaintext messages over UART using a simple framing format.
Currently, each message contains either:

- the BMP280 **chip ID** (for testing),
- or **raw sensor values** (pressure + temperature).

In the future, these messages will be encrypted and parsed on the server side.
Message frequency is currently fixed (one every 2 seconds) using `HAL_GetTick()`.

---

## 1.4 Roles of STM32CubeIDE and HAL

While STM32CubeIDE is used for initial setup (pin configuration, clock tree, peripheral drivers), we do not depend on it to build or flash the code.
The firmware can be built via `make` and flashed via `st-flash`, allowing headless use (e.g., CI/CD or SSH).

The HAL (Hardware Abstraction Layer) is used throughout — we deliberately avoid writing register-level code.
This is a project about engineering, not bit-twiddling.

---

## 1.5 Why We Start From Here

In our case, we literally started with an LED: before dealing with sensors, UART, or encryption, we explored how to blink an LED using different timing mechanisms.
This gave us an essential first contact with the STM32 HAL libraries and the CubeIDE auto-generated code. It also prepared the ground for the periodic logic we now use in the superloop to time our sensor readings.



# 2. Git Project Structure

The project is composed of **two separate repositories**, both part of the official GitHub organization for the Software Engineering course:

- [`code`](https://github.com/unibo-dtm-se-2223-SecureSensorNode/code): contains all source code, both firmware (C for STM32) and server-side components (Python).
- [`report`](https://github.com/unibo-dtm-se-2223-SecureSensorNode/report): contains this very report, written in Markdown.

This structure reflects a clean **separation of concerns**: the code evolves and is versioned independently from the documentation, which follows its own release cycle.

### 2.1 `code` Repository Structure

The `code` repository is organized as follows:

```bash
SecureSensorNode/
├── client/
│   └── firmware/        ← STM32CubeIDE project for Nucleo board (written in C using HAL)
├── server/              ← Microservice in Python: UART receiver + REST API
├── automation/          ← Bash scripts, Makefiles, CI configs (optional)
├── test/                ← Integration tests (TBD)
├── security/            ← Keys, crypto configs, protocol notes (TBD)
├── README.md
└── LICENSE
```

This layout is not accidental. It is designed to maximize:

- **modularity** (clear directory per component),
- **traceability** (Git-friendly layout, one concern per folder),
- **atomicity** (each module can be developed, tested, and versioned independently).

The use of Git is not limited to versioning — it is used to structure the entire development lifecycle:

- `README.md` describes the purpose and usage of the system.
- Commits follow a clean informative style (`feat`, `fix`, `doc`, `refactor`).
- Future additions may include GitHub Actions for CI/CD simulation and automated testing.

Every action — from toggling a GPIO to creating a REST endpoint — is tracked, justified, and versioned.



# 3. The Client — Engineering a Blinking LED

Before dealing with cryptography, REST APIs, and inter-device protocols, we start by making something blink.
But we do it **with intention, precision, and engineering awareness**.

The client is based on an **STM32 Nucleo-F401RE** board. Using **STM32CubeIDE** , we configure pin `PA5` — connected to the onboard green LED — and make it toggle every two seconds using the **SysTick** hardware timer.
This task may appear simple, but it provides a practical opportunity to explore and apply concepts such as **abstraction**, **non-blocking logic**, **hardware/software layering**, and **toolchain automation** — all central topics in modern **software engineering**.


## 3.1 HAL Abstraction and Custom Naming

Thanks to STM32CubeIDE’s graphical configurator, we selected pin `PA5` and renamed it `Green_Led`. This single action:

- configured the GPIO pin as a digital output,
- generated symbolic macros:

```c
  #define Green_Led_Pin GPIO_PIN_5
  #define Green_Led_GPIO_Port GPIOA
```

-allowed us to toggle the pin cleanly with:

 `HAL_GPIO_TogglePin(Green_Led_GPIO_Port, Green_Led_Pin);`

In just two clicks and one function call, the LED toggles.

This reflects the principle of abstraction, a recurring concept in modern software engineering methodologies. HAL encapsulates hardware interaction into modular APIs, aligning with agile development practices where prototyping speed and modular design are crucial.


## 3.2 HAL vs. Bare-Metal Equivalent

The HAL (Hardware Abstraction Layer) library is an embedded software layer developed by ST for STM32 microcontrollers. It aims to maximize portability across STM32 products by
providing a consistent API. HAL functions, such as HAL_GPIO_TogglePin(), encapsulate multiple low-level operations into a single call, making it easy to perform tasks like toggling
a pin.
However, HAL functions are not always interrupt-safe, and in some cases, they offer limited step-by-step control compared to direct register manipulation. This makes HAL convenient
and easy to read, but not atomic.
For fine-grained control—such as precise ISR timing, low-level synchronization, or performance tuning—it is often necessary to resort to CMSIS or direct register manipulation.

To understand the engineering value of HAL, we reproduce the same LED blinking routine using bare-metal code, based on CMSIS and direct register-level access (commonly referred to
as LL Drivers, or Lower Layer Drivers):

*// 1. Enable GPIOA clock (bit 0 in AHB1ENR)*
`RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;`

*// 2. Configure PA5 as general-purpose output*
```GPIOA->MODER &= ~(0x3 << (5 * 2));   // Clear bits
GPIOA->MODER |=  (0x1 << (5 * 2));   // Set mode to '01' (output)
```

*// 3. Set output type to push-pull*
`GPIOA->OTYPER &= ~(0x1 << 5);`

*// 4. Set output speed to low*
`GPIOA->OSPEEDR &= ~(0x3 << (5 * 2));`

*// 5. Disable pull-up/pull-down*
`GPIOA->PUPDR &= ~(0x3 << (5 * 2));`

*// 6. Toggle PA5 by XOR on the Output Data Register*
`GPIOA->ODR ^= (1 << 5);`

*// 7. Optional delay (for visibility, not recommended in production)*
`for (volatile int i = 0; i < 100000; i++);`

This implementation provides full control and deterministic behavior, but:
- is tightly coupled to the hardware,
- requires detailed knowledge of STM32 hardware design (registers in particular),
- lacks portability and modularity.

This emphasizes some of the core software engineering trade-offs: abstraction vs control, portability vs performance, convenience vs atomicity.


## 3.3 STM32CubeIDE: a powerful abstraction tool for embedded engineers

In this project we use STM32CubeIDE, the official development environment provided by STMicroelectronics. This IDE integrates a full toolchain for STM32 development, including:
- a graphical configuration interface,
- built-in compiler and linker (via `arm-none-eabi-gcc`),
- integration with HAL (Hardware Abstraction Layer) and LL (Low Level) libraries,
- debugging support via ST-Link.

STM32CubeIDE is not mandatory to develop on STM32 platforms, but offers huge benefits in terms of productivity and consistency. More importantly, it supports both HAL and LL (more granular control) libraries, depending on the needs of the engineer.
In our case, we opted for HAL as a deliberate choice: engineers are not paid to reinvent the wheel, but to master it and exploit it properly.

When a new project is created, STM32CubeIDE includes a set of precompiled libraries, provided by the vendors (ST and ARM). These are copied into the project structure but are not dynamically generated by the tool:

-**Drivers/STM32F4xx_HAL_Driver/** This folder contains the High-level drivers (HAL) provided by ST, designed to abstract hardware complexity (i.e. GPIO, USART, RCC, etc.)
-**Drivers/CMSIS/** This folder contains ARM’s Cortex Microcontroller Software Interface Standard (CMSIS) with core definitions, registers, NVIC setup, system startup support, etc. ** HAL itself is built on top of CMSIS** so this folder is mandatory even if we choose to develop with HAL.

These libraries provide portability and readability, and are used across thousands of embedded projects — without modification.

After choose the target microcontroller, peripherals configured graphically, pinout, the clock tree setup and other tips, STM32CubeIDE builds a number of critical dinamically-generated files.

Key generated files include:

| File                      | Role                                                         |
| ------------------------- | ------------------------------------------------------------ |
| `main.c`, `main.h`        | Entry point and main application loop                        |
| `system_stm32f4xx.c`      | System clock setup, oscillator configuration                 |
| `stm32f4xx_it.c`          | Default **interrupt handlers** (including `SysTick_Handler`) |
| `startup_stm32f401retx.s` | Startup assembly file: **vector table**, reset handler       |
| `STM32F401RETx_FLASH.ld`  | **Linker script**: defines flash and RAM regions             |

Without the automation provided by this powerful IDE, the engineer would need to manually initialize the stack pointer, write the vector table in assembly, configure the reset handler,
define RAM and Flash regions using a linker script, write all clock tree setup code from scratch (HSI, PLL, prescalers) and implement HAL’s system tick via direct CMSIS calls.
This means hundreds of lines of fragile code that are error-prone and time-consuming.

This abstraction process is not just "comfort", but an engineering design decision. It allows the developer to focus on system logic, interaction, and optimization, instead of wasting
effort on boilerplate.
Using STM32CubeIDE and HAL supports important Software Engineering principles:
- abstraction: hide hardware complexity while exposing well-documented interfaces,
- component reuse: modular libraries that reduce duplication and improve maintainability,
- modular design: code organization that reflects separation of concerns and layering.

If needed, we can always go lower (LL or CMSIS) — but HAL lets us go faster, safer, and with cleaner code.

STM32CubeIDE promotes a DevOps-friendly development flow even in embedded systems, thanks to:
- deterministic project structure: every CubeIDE project has a known, reproducible format, suitable for versioning and integration in CI pipelines,
- makefile-based build system: under the hood, CubeIDE invokes GNU Make, allowing us to replicate the build process via CLI without the IDE,
- automated code generation: graphical configuration becomes source code, ensuring traceability and eliminating manual errors,
- easy integration with Git: every relevant file (except build artifacts) can be version-controlled, tagged, and reviewed.

This aligns with core DevOps practices like:
- infrastructure as Code (our .ioc file is literally "Hardware as Code"),
- continuous Integration (automated builds with tools like GitHub Actions or GitLab CI),
- test Automation (possible via unit testing on mock drivers or hardware-in-the-loop).

In short, HAL + CubeIDE allows embedded systems to benefit from the same agile and automated workflows used in web or cloud development — a key principle of modern Software Engineering.


## 3.4 Progressive Refinement of LED Blinking Logic

Embedded systems are not about blinking LEDs — but blinking LEDs properly reveals the mindset of the engineer behind the system. To demonstrate the design trade-offs behind embedded
control flow, we developed three progressive versions of a firmware component whose only task is to toggle an onboard LED at regular intervals (one second on, one second off).
Each version was committed, tested, and tagged in Git, where the evolution is visible.


### 3.4.1 Version 1 – The "Blocking Delay" (Tag: `v1`)

The code is composed of a `HAL_GPIO_TogglePin` instruction immediately followed by a blocking `HAL_Delay()` call, both placed inside the infinite superloop (`while(1)`) in the main.c 
file. 

```c
while (1)
{
  HAL_GPIO_TogglePin(Green_Led_GPIO_Port, Green_Led_Pin);
  HAL_Delay(1000);
}
```

This naive approach uses `HAL_Delay()`, which puts the CPU to sleep for the specified time. During that time,**no other task can execute**: no sensor can be read, no UART interrupt
can be serviced, no logic can progress. This approach is equivalent to putting the system in a coma between LED toggles.

(only) Problems:
- blocks the system entirely (no multitasking),
- power consumption remains constant (no low-power entry),
- CPU usage: 0% useful, 100% wasted.

This version was committed and tagged as:
`git tag v1` .


### 3.4.2 Version 2 – The "Polling Strategy" (Tag: v2)

Here, `HAL_Delay()` is removed and replaced with non-blocking logic using `HAL_GetTick()`.
This version avoids freezing the CPU, allowing additional tasks to be inserted in the main loop.

```c
uint32_t lastTick = 0;

while (1)
{
  uint32_t now = HAL_GetTick();
  if ((now - lastTick) >= 1000)
  {
    HAL_GPIO_TogglePin(Green_Led_GPIO_Port, Green_Led_Pin);
    lastTick = now;
  }

    *// Other tasks can be placed here*
}
```

However, it still involves constant polling, i.e. checking a condition over and over, which wastes CPU cycles and power.

Improvements:
- no more blocking,
- allows multitasking inside the main loop.

Still problematic:
- busy-waiting is energy inefficient,
- main loop complexity grows with tasks,
- inefficient in low-power applications.

This version was committed and tagged as:
`git tag v2` .

According with the organization of the project, modifies v1 and v2 take place in ~/projects/SecureSensorNode/client/firmware/Core/Src/main.c .


### 3.4.3 Version 3 – Event-driven via SysTick Interrupt (Tag: v3)

This final version consists of a few lines of code inserted **inside** the SysTick_Handler() function defined in the stm32f4xx_it.c file (~/projects/SecureSensorNode/client/firmware/Core/Src/stm32f4xx_it.c), 
rather than in the superloop of main.c.
This **interrupt-driven** approach detaches the LED blinking logic from the main application flow. It makes the system **fully event-based**, eliminating the need for polling or blocking instructions.

```c
void SysTick_Handler(void)
{
  HAL_IncTick(); // Maintains HAL timebase

  static uint32_t count = 0;
  if (++count >= 2000)
  {
    HAL_GPIO_TogglePin(Green_Led_GPIO_Port, Green_Led_Pin);
    count = 0;
  }
}
```

The **SysTick interrupt** is automatically triggered every 1 ms by the ARM Cortex-M core. We increment a static counter each time the interrupt is fired and toggle the LED every 
2000 ms. The static keyword ensures that the variable count retains its value across multiple executions of the handler.

From a software engineering perspective, this change:
- Frees the superloop, which can now be left completely empty,
- Turns the firmware into an event-driven architecture,
- Improves modularity, allowing the CPU to react only when needed.

This empty superloop could now act as an idle loop, a typical feature in embedded systems. In a more advanced scenario, it could even be combined with low-power features, such as:
- the __WFI() ("Wait For Interrupt") instruction, which halts the CPU until the next interrupt,
- or **Sleep, Stop, or Standby modes**, available on STM32 microcontrollers.

Low-power techniques are not implemented at this stage, but will be discussed in detail in later sections, when real sensor acquisition and data transmission are integrated. 
At that point, reducing CPU activity and energy footprint will become essential.

Key benefits:
- no CPU cycles wasted on polling,
- full responsiveness and multitasking,
- excellent energy profile,
- aligns with event-driven design patterns in embedded and real-time systems.

Tagged as:
`git tag v3` .


## 3.5 Git Versioning of Firmware Evolution

To ensure traceability and document the evolution of the LED blinking logic, each version was committed and tagged in the Git repository. The progression was implemented within a
single STM32 project, and version control was used to capture each stage of refinement.

The files involved are:
- main.c (for versions 1 and 2),
- stm32f4xx_it.c (for version 3).

Git commands used to commit and tag the versions:

### Version 1 – HAL_Delay (blocking)
```bash
git add client/firmware/Core/Src/main.c
git commit -m "v1: Blocking LED blinking with HAL_Delay()"
git tag v1
git push
git push origin v1
```

### Version 2 – HAL_GetTick (polling)
```bash
git add client/firmware/Core/Src/main.c
git commit -m "v2: Non-blocking LED blinking with HAL_GetTick()"
git tag v2
git push
git push origin v2
```

### Version 3 – SysTick interrupt (event-driven)
```bash
git add client/firmware/Core/Src/stm32f4xx_it.c
git commit -m "v3: Event-driven LED blinking via SysTick_Handler"
git tag v3
git push
git push origin v3
```

Each tagged version is visible in the remote repository and can be restored or inspected independently.



## 3.6 Manual Flashing via Command Line

About flashing, instead of using the IDE’s built-in flashing system, we performed firmware upload via command line:

```bash
arm-none-eabi-objcopy -O binary BlinkHAL.elf BlinkHAL.bin
st-flash write BlinkHAL.bin 0x08000000
```

Why did we choose the command line?
- Transparent output: the CLI tools display each operation performed, allowing us to clearly see the memory mapping, flashing progress, and any errors,
- Portability: this method works in headless environments (e.g., remote machines accessed via SSH) and is independent of GUI tools,
- Reproducibility: the process can be scripted and reused, facilitating version-controlled deployment pipelines,
- Logging clarity: unlike IDEs that may abstract away low-level messages, st-flash outputs precise logs, which are helpful for debugging,
- DevOps alignment: this manual process matches the mindset of automation and repeatability, key themes in modern DevOps and agile software engineering practices.



## 3.7 DevOps-aligned Automation: Building and Flashing via Shell Script

Automation and CI/CD pipelines are key pillars of modern software engineering — including embedded development. Embracing these DevOps-aligned practices allows for faster iteration,
reproducibility, and integration into larger toolchains.
To support these principles, we created a lightweight Bash script located at `automation/scripts/flash.sh` which builds and flashes the firmware without relying on STM32CubeIDE.

This is especially useful in:
- headless environments (e.g., SSH or CI runners),
- rapid development cycles,
- debugging low-level build issues with full visibility.

The script performs:
1. A call to `make` within the firmware build directory,
2. A conversion of the `.elf` file into a binary `.bin` with `arm-none-eabi-objcopy`,
3. A flash upload to the STM32 board using `st-flash`,
4. An automatic verification and error check if the ELF/BIN is missing.

This approach reflects a DevOps mindset: automation, portability, and transparency. It was successfully tested on Ubuntu 22.04 with ST-Link v2 and an STM32F401RE board.

### Script: `flash.sh`

```bash
#!/bin/bash

BUILD_DIR=client/firmware/Debug
PROJECT_NAME=BlinkHAL

echo "[1] Building the project..."
make -C $BUILD_DIR

echo "[2] Creating binary image..."
arm-none-eabi-objcopy -O binary $BUILD_DIR/${PROJECT_NAME}.elf $BUILD_DIR/${PROJECT_NAME}.bin

if [ ! -f "$BUILD_DIR/${PROJECT_NAME}.bin" ]; then
  echo "[!] Build failed or ELF file not found!"
  exit 1
fi

echo "[3] Flashing firmware via st-flash..."
st-flash write $BUILD_DIR/${PROJECT_NAME}.bin 0x08000000

echo "[✓] Flash completed successfully!"
```

In contrast to manual GUI-based approaches, this automation is **explicit, repeatable, and self-documented** — all core principles of modern DevOps pipelines.
The script leverages make to build the firmware from STM32CubeIDE-generated Makefiles. It converts the resulting ELF (Executable and Linkable Format) to a raw binary (.bin) using 
arm-none-eabi-objcopy, as this is the format expected by the flash tool: while the .elf includes metadata, debug symbols, and section headers, the .bin file is a clean byte-level 
image suitable for flashing at address 0x08000000.

In DevOps, failing gracefully is good — but failing loudly and clearly is better. Our script doesn’t build blindly and hope for the best: if the .bin file isn’t created, it 
**slams the brakes** and tells you what went wrong.
Because in real engineering, “it seemed to work” is not a valid deployment strategy.
This way, the process stays honest, the logs stay clean, and mistakes are caught early.

Moreover, placing this script in version control means that **the build process itself becomes part of the software**, not just a tool used outside of it.

In short: we’re not just compiling code — **we're engineering process.**



# 4. BMP280 Integration and Raw Data Acquisition

Once the LED had served its purpose (Chapter 3), we were ready to interact with a *real sensor*.
Our goal is acquire and transmit temperature and pressure from a **Bosch BMP280**, using only **STM32 HAL**, robust coding principles, and well-documented evolution steps.

This chapter chronicles the full story — including unexpected sensor failures — while highlighting how good **software engineering practices** shine even when hardware misbehaves.

---

## 4.1 Why BMP280?

The BMP280 was chosen for the following reasons:
- **Widespread and inexpensive**: easy to source, well-documented.
- **Two-in-one**: it provides both temperature and pressure readings.
- **I2C interface**: supported natively by STM32 HAL, reducing wiring complexity.
- **Perfect for experimentation**: allows us to explore I2C, UART, timing, and modular programming — all in one shot.

The sensor also supports **SPI**, but we selected **I2C** for its simplicity: just two lines (SCL, SDA), fewer pins, less code (and the speed is enough for our affairs).

---

## 4.2 Wiring and Setup

We connected the BMP280 to the STM32 Nucleo-F401RE board as follows:

| BMP280 Pin | Signal | STM32 Pin | CubeIDE Label |
|------------|--------|-----------|---------------|
| VCC        | Power  | 3.3V      | 3.3V          |
| GND        | GND    | GND       | GND           |
| SCK        | SCL    | PB8       | I2C1_SCL      |
| SDI        | SDA    | PB9       | I2C1_SDA      |
| CSB        | CS     | GND       | -             |
| SDO        | Addr   | GND       | -             |

> Tying **CSB and SDO to GND** forces the BMP280 into I2C mode with 7-bit address `0x76` (i.e. `0xEC` in write-mode with 8-bit addressing).

In the `.ioc` file, we enabled:
- `I2C1` (PB8/PB9)
- `USART2` (for UART output)

CubeIDE generated the relevant `i2c.c/.h` and `usart.c/.h`.
We verified and corrected the pin alternate functions manually — the IDE sometimes "forgets".

---

## 4.3 Step 1 – Reading the Chip ID (outside the superloop)

Before anything else, we wanted to confirm the sensor was alive. So we know that **chip ID register** of the BMP280 is located at `0xD0` and contains `0x58`, we used 
`HAL_I2C_Mem_Read()` to query it:

```c
/* Step 1: Read BMP280 chip ID */
uint8_t chip_id = 0;
HAL_StatusTypeDef status;

status = HAL_I2C_Mem_Read(&hi2c1, 0xEC, 0xD0, I2C_MEMADD_SIZE_8BIT, &chip_id, 1, HAL_MAX_DELAY);

if (status == HAL_OK) {
    char msg[64];
    snprintf(msg, sizeof(msg), "BMP280 Chip ID: 0x%02X\r\n", chip_id);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
} else {
    char err[] = "Error reading BMP280 ID\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)err, strlen(err), HAL_MAX_DELAY);
}
```
We flashed the firmware and used minicom to observe the output:
```yaml
BMP280 Chip ID: 0x58
```
Because of the reading was a success, we tagged this version as 
`git tag v4`


## 4.4 Step 2 – Polling inside the superloop (every 2 seconds)

To simulate periodic acquisition, we moved the ID reading inside the superloop, using a non-blocking HAL_GetTick() strategy. This avoids HAL_Delay() and keeps the MCU responsive.
```c
uint32_t last_read_time = 0;

while (1) {
    if (HAL_GetTick() - last_read_time >= 2000) {
        last_read_time = HAL_GetTick();
        // Same ID read logic as before
    }
}
```
UART output:
```yaml
BMP280 Chip ID: 0x58
BMP280 Chip ID: 0x58
BMP280 Chip ID: 0x58
...
```
This confirmed:
- I2C works inside the superloop,
- UART stays responsive,
- HAL_GetTick() is viable for periodic polling.

We tagged it as:

`git tag v5`


## 4.5 Step 3 – Reading real raw values (with modular HAL-based function)

Once confident in our setup, we created a dedicated function to read raw temperature and pressure values using HAL and called `bmp280_read.c`:
```c
HAL_StatusTypeDef BMP280_ReadTemperatureAndPressure(I2C_HandleTypeDef *hi2c, int32_t *temperature, int32_t *pressure) {
    uint8_t buffer[6];

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, 0xEC, 0xF7, I2C_MEMADD_SIZE_8BIT, buffer, 6, HAL_MAX_DELAY);
    if (status != HAL_OK)
        return status;

    *pressure = ((uint32_t)(buffer[0]) << 12) | ((uint32_t)(buffer[1]) << 4) | (buffer[2] >> 4);
    *temperature = ((uint32_t)(buffer[3]) << 12) | ((uint32_t)(buffer[4]) << 4) | (buffer[5] >> 4);

    return HAL_OK;
}
```
This function:
- Keeps logic modular and testable,
- Uses HAL only (no bit-banging),
- Promotes code reuse and clarity.

In the superloop, we zero temperature and pressure before each read:
```c
int32_t temperature = 0;
int32_t pressure = 0;
HAL_StatusTypeDef status;
```
This is a defensive programming technique: it ensures no stale or garbage values are printed if a read fails — and it's a good software engineering practice that we emphasized throughout the project.


## 4.6 Final step – event-driven acquisition via SysTick + LED heartbeat

Our final iteration (tag `v6`) transformed the data acquisition from a periodic polling loop to an **event-driven model** using the **SysTick interrupt**.
Some reasons why:
- **No delays**: `HAL_Delay()` and `HAL_GetTick()` are fine, but they rely on periodic checks inside the superloop, which doesn't scale well.
- **Modularity**: Interrupts let us decouple measurement logic from timekeeping.
- **LED heartbeat**: We reused the SysTick to also blink the green LED — a simple but effective “watchdog” that tells us the MCU is alive and responsive.
- **Resilience**: The firmware handles I2C errors gracefully and reports them over UART.


### Superloop in `main.c`

This is the core logic running inside the `while(1)` loop:
```c
/* USER CODE BEGIN WHILE */
while (1)
{
    // If 1 second has passed, proceed with sensor read
    if (tick_1s_elapsed) {
        tick_1s_elapsed = 0; // Reset the flag for next cycle

        // Defensive initialization of variables
        int32_t temperature = 0;
        int32_t pressure = 0;

        // Attempt to read raw temperature and pressure via I2C
        HAL_StatusTypeDef status = BMP280_ReadTemperatureAndPressure(&hi2c1, &temperature, &pressure);

        if (status == HAL_OK) {
            // Format and send the values over UART
            char msg[64];
            snprintf(msg, sizeof(msg), "T(raw): %ld | P(raw): %ld\r\n", temperature, pressure);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        } else {
            // Handle read failure — still notify via UART
            char err[] = "BMP280 read error\r\n";
            HAL_UART_Transmit(&huart2, (uint8_t*)err, strlen(err), HAL_MAX_DELAY);
        }
    }
}
/* USER CODE END WHILE */
```
Note that variables aren't initialized every loop just for paranoia, but for **defensive programming**: if the `HAL_I2C_Mem_Read()` fails, we want to ensure known values, and prevent accidental use of garbage memory. This is exactly what makes code robust and secure, a key concept in Software Engineering.
About the **SysTick Handler** in `stm32f4xx_it.c`, we configured the MCU to generate an interrupt every 1 ms using the SysTick peripheral.
Here’s the modified interrupt service routine:
```c
void SysTick_Handler(void)
{
  HAL_IncTick(); // Always increment the HAL internal millisecond counter

  static uint32_t tick_counter = 0;

  tick_counter++; // Increase our local counter

  if (tick_counter >= 1000) { // Every 1000 ms (1 second)
    tick_counter = 0;

    tick_1s_elapsed = 1; // Signal the superloop to take action

    // Blink the onboard green LED to show activity
    HAL_GPIO_TogglePin(Green_Led_GPIO_Port, Green_Led_Pin);
  }
}
```
Let's break it down:
- `tick_counter` is local to the ISR and counts milliseconds.
- Every 1000 ms, it resets and sets a global flag: `tick_1s_elapsed`.
- The superloop sees this flag and triggers the BMP280 read.
- Simultaneously, we toggle the LED — a non-intrusive way to visualize firmware health.

Benefits of this design were:
- **Responsiveness**: No blocking delays. Other operations (e.g. UART input, future sensors) remain unaffected.
- **Reliability**: All I2C read failures are caught and reported explicitly.
- **Readability**: Code is modular, commented, and consistent with STM32 HAL best practices.
- **Visual feedback**: The blinking LED gives immediate feedback after flashing.

We tagged this robust version as:
```bash
git add client/firmware/Core/Src/main.c client/firmware/Core/Src/stm32f4xx_it.c
git commit -m "v6: Robust and event-driven BMP280 read using SysTick + LED heartbeat"
git tag -f v6
git push origin bmp280
git push -f origin v6
```
Even if the sensor failed, the firmware proved solid.
And that’s the point: **Software Engineering is about writing systems that work — and fail — predictably**.


## 4.7 ...And then the sensor died!

Shortly after testing, we noticed erratic readings:
```yaml
T(raw): 128 | P(raw): 128
T(raw): 128 | P(raw): 128
T(raw): 128 | P(raw): 128
T(raw): 128 | P(raw): 128
T(raw): 128 | P(raw): 128
T(raw): 128 | P(raw): 128
.......
```
Eventually, the sensor returned nothing.
Turns out the sensor was damaged (ESD? miswiring?). But the firmware was stable, modular, and resilient — exactly what software engineering demands.
We switched back to v4 and verified the ID check still worked intermittently:
`git checkout v4`
and confirmed:
```yaml
BMP280 Chip ID: 0x58
```
However, it consistently failed to return valid temperature and pressure readings — always returning the same raw values. This partial responsiveness suggested hardware degradation, likely on the analog front-end. The I2C and identification logic still worked, but the internal ADC or registers were likely damaged.
So we ordered a new sensor, kept the broken one for fault testing, and confidently retained our v6 firmware, which handled all failure scenarios gracefully.

