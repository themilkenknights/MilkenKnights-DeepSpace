package frc.robot.paths;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.geometry.Pose2d;
import frc.robot.lib.geometry.Pose2dWithCurvature;
import frc.robot.lib.geometry.Rotation2d;
import frc.robot.lib.geometry.Translation2d;
import frc.robot.lib.trajectory.Trajectory;
import frc.robot.lib.trajectory.TrajectoryUtil;
import frc.robot.lib.trajectory.timing.CentripetalAccelerationConstraint;
import frc.robot.lib.trajectory.timing.TimedState;
import frc.robot.lib.trajectory.timing.TimingConstraint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TrajectoryGenerator {
    // CRITICAL POSES
    // Origin is the center of the robot when the robot is placed against the middle of the alliance station wall.
    // +x is towards the center of the field.
    // +y is to the left.
    // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON RIGHT! (mirrored about +x axis for LEFT)
    public static final Pose2d kStartPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    private static final double kMaxCentripetalAccel = 110.0;
    private static final double kMaxVoltage = 12.0;
    private static final double kMaxAccel = 130.0;
    private static final double kMaxVel = 130.0;
    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;
    private VisionTrajectorySet mVisionTrajectorySet = null;

    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
    }

    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }

    public void generateTrajectories() {
        if (mTrajectorySet == null) {
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println("Finished trajectory generation");
        }
    }

    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(boolean reversed, final List<Pose2d> waypoints,
                                                                          final List<TimingConstraint<Pose2dWithCurvature>> constraints, double max_vel,  // inches/s
                                                                          double max_accel,  // inches/s^2
                                                                          double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_voltage);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(boolean reversed, final List<Pose2d> waypoints,
                                                                          final List<TimingConstraint<Pose2dWithCurvature>> constraints, double start_vel,  // inches/s
                                                                          double end_vel,  // inches/s
                                                                          double max_vel,  // inches/s
                                                                          double max_accel,  // inches/s^2
                                                                          double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel, max_accel, max_voltage);
    }

    public void generateVisionTrajectories(Pose2d endPose) {
        if (mVisionTrajectorySet == null) {
            System.out.println("Generating trajectories...");
            mVisionTrajectorySet = new VisionTrajectorySet(endPose);
            System.out.println("Finished trajectory generation");
        }
    }

    public VisionTrajectorySet getVisionTrajectorySet() {
        return mVisionTrajectorySet;
    }

    public class TrajectorySet {
        public final MirroredTrajectory sideStartToNearScale;

        private TrajectorySet() {
            sideStartToNearScale = new MirroredTrajectory(getSideStartToNearScale());
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSideStartToNearScale() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kStartPose);
            waypoints.add(kStartPose.transformBy(new Pose2d(new Translation2d(40.0, 0.0), Rotation2d.fromDegrees(0.0))));
            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)), kMaxVel, kMaxAccel, kMaxVoltage);
        }

        public class MirroredTrajectory {
            public final Trajectory<TimedState<Pose2dWithCurvature>> left;
            public final Trajectory<TimedState<Pose2dWithCurvature>> right;

            public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> right) {
                this.right = right;
                this.left = TrajectoryUtil.mirrorTimed(right);
            }

            public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left) {
                return left ? this.left : this.right;
            }
        }
    }


    public class VisionTrajectorySet {
        public final Trajectory<TimedState<Pose2dWithCurvature>> visionTraj;

        private VisionTrajectorySet(Pose2d endPose) {
            visionTraj = getTraj(endPose);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTraj(Pose2d endPose) {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(RobotState.getInstance().getPredictedFieldToVehicle(Timer.getFPGATimestamp() + 0.02));
            waypoints.add(kStartPose.transformBy(endPose));
            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)), kMaxVel, kMaxAccel, kMaxVoltage);
        }
    }
}
