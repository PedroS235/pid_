# PID Controller

This is a simple to use PID controller that is targeted to be used with PlatformIO, or Arduino. It can be used to control anything you desire, from motors to heaters and more. This implementation of PID has been used on a **motor controller** software, that
controls a differential drive robot. It can be found [here](https://github.com/PedroS235/motor_controller/tree/main)

## TOC

<!--toc:start-->

- [Features](#features)
- [Getting Started](#getting-started)
  - [Installation](#installation)
  - [Usage](#usage)
  <!--toc:end-->

## Features

- Easy to use
- Clamps the output correction to a given range _(Default is [0, 255])_
- Has an integral anti-windup, by clamping down the integral sum to the correction range
- Ability to reset PID when setting a new set-point
- Resets PID when both error and set-point are set to 0, to remove possible unwanted noise.
- Can be set to periodically compute a new correction based on some interval. Useful for unstable systems.

## Getting Started

### Installation

You can add this library to your `platformio.ini`:

```toml
lib_deps=
    https://github.com/PedroS235/pid_controller_cpp#2.0.2
```

Or, if preferred you can also clone this repo manually using:

```sh
git clone https://github.com/PedroS235/pid_controller_cpp.git
```

### Usage

In order to use this PID, the method `compute` **must be called on every loop iteration**.

```cpp
#include "Arduino.h"
#include <pid.hpp>

pid_gains_t gains = {1.0, 0.2, 0.3};
output_limits_t limits = {0, 255};
uint8_t rate = 50; // Hz

// Initializing the PID controller with gains, limits structs
PID pid(gains, limits);
// Or
// Initializing the PID controller with inividual gains and limits
PID pid1(1.0, 0.2, 0.3, 0, 255);
// Or
// Initializing the PID controller to compute corrections at a fixed rate
PID pid2(gains, limits, rate);

void setup() {
  // Setting the setpoint
  pid.set_setpoint(30);
}

void loop() {
  // Note: if rate is set > 0, a new correction will only be computed after
  // a period.
  float correction = pid.compute(20);
  // Do something with correction
}
```
