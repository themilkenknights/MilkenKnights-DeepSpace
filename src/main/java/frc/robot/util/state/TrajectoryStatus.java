package frc.robot.util.state;

import jaci.pathfinder.Trajectory;

public class TrajectoryStatus {

    public static TrajectoryStatus NEUTRAL = new TrajectoryStatus(
            new Trajectory.Segment(0, 0, 0, 0, 0, 0, 0, 0), 0, 0, 0, 0, 0);

    private double output;
    private Trajectory.Segment seg;
    private double posError;
    private double velError;
    private double feedFoward;
    private double angError;
    private double arbFeed;

    public TrajectoryStatus(Trajectory.Segment seg, double posError, double velError, double angError,
                            double arbFeed, double output) {
        this.seg = seg;
        this.output = output;
        this.posError = posError;
        this.velError = velError;
        this.angError = angError;
        this.arbFeed = arbFeed;
    }

    public double getOutput() {
        return output;
    }

    public double getPosError() {
        return posError;
    }

    public double getVelError() {
        return velError;
    }

    public double getAngError() {
        return angError;
    }

    public double getArbFeed() {
        return arbFeed;
    }

    public Trajectory.Segment getSeg() {
        return seg;
    }

    public String toString() {
        return seg.toString() + "Output: " + output + " Position Error: " + posError
                + "Velocity Error: " + velError + "Angle Error: " + angError + " Arb Feed: " + arbFeed;
    }

}
