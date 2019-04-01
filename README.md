# MilkenKnights-DeepSpace

## Intellij
- Before opening the project with IntelliJ, run `./gradlew idea` in Terminal or `gradlew idea` in Command Prompt. This will generate IDE files for IntelliJ IDEA (Java).

- Double click `MilkenKnights-DeepSpace.ipr` or `File - Open Project` in IntelliJ IDEA.

- Click import gradle configuration and select auto-import. Make sure that the Java JDK is version 11 or higher. Wait for the build process to finish.

## Driver Button Map

- Left Trigger Reverse

- Right Trigger Forward

- Left Stick (X axis) Turn

- A Vision Hatch Outtake

- B Vision HP Intake

- X Vision Cargo Outtake

- Y Auto Climb

- Left Bumper Vision Auto-aim On

- Right Bumper Vision Auto-aim Off

## Operator Button Map

- 1 (Trigger) Toggle Manual Ground Hatch Open Loop

- 2 (Thumb Button) Toggle Manual Cargo Arm Open Loop

- 3 Hatch Spear Toggle

- 4 Hatch Spear HP Intake (Limit Switch Enabled)

- 5 Roller Intake

- 6 Roller Outtake

- 7 Toggle Front Climb Actuators

- 8 Toggle Rear Climb Actuators

- 9 Defense Mode

- 10 Zero Cargo Arm & Disable Soft Limits

- 11 Front Cargoship Roller Outtake

- 12 Retract Spear Pancake Actuator

- POV UP Cargo Rocket Level One

- POV DOWN Cargo Rocket Level Two

- POV LEFT Cargo Intake Setpoint

- POV RIGHT Cargo Reverse Cargoship

## Things to check when having issues

- CAN Bus Utilization

- TODO Items

- Loop Times

- Phoenix Tuner

  - Self-test device
  
    - Ensure Limit Switches are in correct state
    
    - Ensure correct sensor is selected and reports the correct values
    
    - Throttle motor controller to verify that it's working

- All CAN devices are up-to-date

- CTRE Phoenix & WPILIB are updated

## Robot Hardware

- Drive
  - Two CIM Motors on each side
  - One SRX Mag Encoder on each side
- Spear Arm
  - Single Solenoid to stow/lower the spear
  - Single Solenoid to retract/extend pancake actuator
  - REV Magnetic Limit Switch connected to CTRE Breakout board
- Cargo Arm
  - Two 775 pro motors
  - SRX Mag Encoder
  - REV Magnetic Limit Switch connected to CTRE Breakout board
- Vision
  - Fisheye camera connected to the Rio on the hatch side
  - Limelight V1 on the hatch side
  - Microsoft Lifecam on the cargo arm side connected to the Limelight
- Misc
  - Pigeon IMU connected to Talon SRX over ribbon cable
  - Four Actuators connected to two single solenoids (left/right linked) for Level 2 Climb

## Current Paths

- [1](https://themilkenknights.github.io/MilkenKnights-DeepSpace/path_visualizer/index.html?%5B%7B%22position%22:%7B%22x%22:68,%22y%22:114%7D,%22theta%22:0,%22comment%22:%22%22%7D,%7B%22position%22:%7B%22x%22:155,%22y%22:150%7D,%22theta%22:0,%22comment%22:%22%22%7D%5D)
- [2](https://themilkenknights.github.io/MilkenKnights-DeepSpace/path_visualizer/index.html?%5B%7B%22position%22:%7B%22x%22:204,%22y%22:150%7D,%22theta%22:0,%22comment%22:%22%22%7D,%7B%22position%22:%7B%22x%22:200,%22y%22:150%7D,%22theta%22:0,%22comment%22:%22%22%7D,%7B%22position%22:%7B%22x%22:170,%22y%22:180%7D,%22theta%22:1.5707499999936907,%22comment%22:%22%22%7D,%7B%22position%22:%7B%22x%22:65,%22y%22:26%7D,%22theta%22:0,%22comment%22:%22%22%7D%5D)
- [3](https://themilkenknights.github.io/MilkenKnights-DeepSpace/path_visualizer/index.html?%5B%7B%22position%22:%7B%22x%22:20,%22y%22:26%7D,%22theta%22:0.05235833333312303,%22comment%22:%22%22%7D,%7B%22position%22:%7B%22x%22:232,%22y%22:92%7D,%22theta%22:0.08726388888853838,%22comment%22:%22%22%7D,%7B%22position%22:%7B%22x%22:262,%22y%22:72%7D,%22theta%22:1.6580138888822291,%22comment%22:%22%22%7D%5D)
