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

- B Vision Cargo Outtake

- X Vision HP Intake

- Y Auto Climb

## Operator Button Map

- 1 (Trigger) Toggle Manual Ground Hatch Open Loop

- 2 (Thumb Button) Toggle Manual Cargo Arm Open Loop

- 3 Hatch Spear Toggle

- 4 Hatch Spear HP Intake (Limit Switch Enabled)

- 5 Roller Intake

- 6 Roller Outtake

- 7 Unmapped

- 8 Zero Arms

- 9 Stop Auto Action

- 10 Defense Mode

- 11 Ground Intake Toggle (Stowed/Ground Setpoints)

- 12 Ground Intake Transfer Hatch

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

## Current Paths

- [CS-1](./path_visualizer/index.html?%5B%7B%22position%22:%7B%22x%22:204,%22y%22:150%7D,%22theta%22:0,%22comment%22:%22%22%7D,%7B%22position%22:%7B%22x%22:180,%22y%22:181%7D,%22theta%22:1.5707499999936907,%22comment%22:%22%22%7D,%7B%22position%22:%7B%22x%22:70,%22y%22:26%7D,%22theta%22:0,%22comment%22:%22%22%7D,%7B%22position%22:%7B%22x%22:32,%22y%22:26%7D,%22theta%22:0,%22comment%22:%22%22%7D%5D)
