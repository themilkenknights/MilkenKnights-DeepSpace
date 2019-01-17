package frc.robot.lib.vision;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.command.Subsystem;

import java.util.ArrayList;
import java.util.HashMap;

public class Vision extends Subsystem {
    // These values are the default if you instantiate a PixySPI without arguments.
    // To create multiple PixySPI objects and thus use multiple Pixy cameras via SPI
    // Copy the items below, change variable names as needed and especially change
    // the SPI port used eg; Port.kOnboardCS[0-3] or Port.kMXP
    public PixySPI pixy1;
    public HashMap<Integer, ArrayList<PixyPacket>> packets = new HashMap<Integer, ArrayList<PixyPacket>>();
    Port port = Port.kOnboardCS0;
    String print;

    public Vision() {
        // Open a pipeline to a Pixy camera.
        pixy1 = new PixySPI(new SPI(port), packets, new PixyException(print));
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}

