# Electric Wheelchair Prototype

This repository stores the control code for multiple device options and will soon include documentation for full project.

## Features
- Full 360 degree range of control including turn on the spot
- Overcurrent protection for wheel locks and motor preservation
- Heading drift correction

## Hardware
- VersaCart S Series 180L
- 350W Hoverboard Hub Motors
- STM32F401, STM32F429I, Arduino R3
- Weize 12V 12Ah Lead Acid Batteries
- RioRand 350W ESC
- WitMotion WT901 IMU

#### Contributors
- Sean Kadkhodayan - Electrical and Mechanical Design

## Software

Code is written for both the STM32F401 Nucleo and the Arduino R3. Heading drift correction in progress. Looking into computer vision options.

#### Contributors
- SeanK27 - STM32Implementation, ArduinoR3Implementation/SKKImp
- learner559 - ArduinoR3Implementation/learner559Imp

Pull requests are welcome :)

## Milestones
:white_check_mark: Initial Electircal and Mechanical design with hub motors

:white_check_mark: Initial software for full movement range

:hourglass_flowing_sand: Heading drift correction with IMU

:hourglass_flowing_sand: Computer vision and marker following

:sparkles: GPS and other sensors for path finding

:sparkles: Higher power motors for speed and, in turn, full electrical system upgrade

<br>

Key: :white_check_mark: Complete; :hourglass_flowing_sand: In Progress; :sparkles: Planned
