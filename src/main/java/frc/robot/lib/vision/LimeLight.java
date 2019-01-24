package frc.robot.lib.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.vision.LimeLightControlMode.*;

/**
 * Lime Light Class was started by Corey Applegate of Team 3244 Granite City Gearheads. We Hope you Enjoy the Lime Light Camera.
 */
public class LimeLight {
    Notifier _hearBeat = new Notifier(new PeriodicRunnable());
    NetworkTableEntry tv, tx, ty, ta, ts, tl, thoriz, tvert, ledMode, camMode, pipeline, stream, snapshot;
    private NetworkTable m_table;
    private String m_tableName;
    private Boolean isConnected = false;
    private double _hearBeatPeriod = 0.1;

    /**
     * Using the Default Lime Light NT table
     */
    public LimeLight() {
        m_tableName = "limelight";
        m_table = NetworkTableInstance.getDefault().getTable(m_tableName);
        _hearBeat.startPeriodic(_hearBeatPeriod);

        tv = m_table.getEntry("tv");
        tx = m_table.getEntry("tx");
        ty = m_table.getEntry("ty");
        ta = m_table.getEntry("ta");
        ts = m_table.getEntry("ts");
        tl = m_table.getEntry("tl");
        thoriz = m_table.getEntry("thor");
        tvert = m_table.getEntry("tvert");
        ledMode = m_table.getEntry("ledMode");
        camMode = m_table.getEntry("camMode");
        pipeline = m_table.getEntry("pipeline");
        stream = m_table.getEntry("stream");
        snapshot = m_table.getEntry("snapshot");
    }

    /**
     * If you changed the name of your Lime Light tell Me the New Name
     */
    public LimeLight(String tableName) {
        m_tableName = tableName;
        m_table = NetworkTableInstance.getDefault().getTable(m_tableName);
        _hearBeat.startPeriodic(_hearBeatPeriod);
    }

    /**
     * Send an instance of the NetworkTabe
     */
    public LimeLight(NetworkTable table) {
        m_table = table;
        _hearBeat.startPeriodic(_hearBeatPeriod);
    }

    // This is a test
    public boolean isConnected() {
        return isConnected;
    }

    public LimelightTarget returnTarget() {
        return new LimelightTarget(getIsTargetFound(), getX(), getY(), getHorizLength(), getVertLength(), getCaptureTime());
    }

    /**
     * tv Whether the limelight has any valid targets (0 or 1)
     */
    public boolean getIsTargetFound() {
        double v = tv.getDouble(0);
        return v != 0.0f;
    }

    /**
     * tx Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
     */
    public double getX() {
        double x = tx.getDouble(0.0);
        return x;
    }

    /**
     * ty Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
     */
    public double getY() {
        double y = ty.getDouble(0.0);
        return y;
    }

    public double getHorizLength() {
        double h = thoriz.getDouble(0.0);
        return h;
    }

    public double getVertLength() {
        double v = tvert.getDouble(0.0);
        return v;
    }

    public double getCaptureTime() {
        double l = Timer.getFPGATimestamp() - (tl.getDouble(0.0) * 1e-3);
        return l;
    }

    /**
     * ta Target Area (0% of image to 100% of image)
     */
    public double getTargetArea() {
        double a = ta.getDouble(0.0);
        return a;
    }

    /**
     * ts Skew or rotation (-90 degrees to 0 degrees)
     */
    public double getSkew_Rotation() {
        double s = ts.getDouble(0.0);
        return s;
    }

    /**
     * tl The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
     */
    public double getPipelineLatency() {
        double l = tl.getDouble(0.0);
        return l;
    }

    private void resetPilelineLatency() {
        m_table.getEntry("tl").setValue(0.0);
    }

    /**
     * Returns current LED mode of the Lime Light
     *
     * @return LedMode
     */
    public LedMode getLEDMode() {
        double led = ledMode.getDouble(0.0);
        LedMode mode = LedMode.getByValue(led);
        return mode;
    }
    // Setters

    /**
     * LedMode Sets limelight’s LED state
     * <p>
     * kon koff kblink
     */
    public void setLEDMode(LedMode ledMode) {
        m_table.getEntry("ledMode").setValue(ledMode.getValue());
    }

    /**
     * Returns current Cam mode of the Lime Light
     *
     * @return CamMode
     */
    public CamMode getCamMode() {
        double cam = camMode.getDouble(0.0);
        CamMode mode = CamMode.getByValue(cam);
        return mode;
    }

    /**
     * camMode Sets limelight’s operation mode
     * <p>
     * kvision kdriver (Increases exposure, disables vision processing)
     */
    public void setCamMode(CamMode camMode) {
        m_table.getEntry("camMode").setValue(camMode.getValue());
    }

    /**
     * Returns current Pipeling of the Lime Light
     *
     * @return Pipelinge
     */
    public double getPipeline() {
        double pipe = pipeline.getDouble(0.0);
        return pipe;
    }
    /**
     * pipeline Sets limelight’s current pipeline
     *
     * 0 . 9 Select pipeline 0.9
     *
     * @param pipeline
     */
    /*
     * public void setPipeline(Double pipeline) { if(pipeline<0){ pipeline = 0.0;
     * throw new IllegalArgumentException("Pipeline can not be less than zero");
     * }else if(pipeline>9){ pipeline = 9.0; throw new
     * IllegalArgumentException("Pipeline can not be greater than nine"); }
     * m_table.getEntry("pipeline").setValue(pipeline); }
     */

    /**
     * pipeline Sets limelight’s current pipeline
     * <p>
     * 0 . 9 Select pipeline 0.9
     */
    public void setPipeline(Integer pipeline) {
        if (pipeline < 0) {
            pipeline = 0;
            throw new IllegalArgumentException("Pipeline can not be less than zero");
        } else if (pipeline > 9) {
            pipeline = 9;
            throw new IllegalArgumentException("Pipeline can not be greater than nine");
        }
        m_table.getEntry("pipeline").setValue(pipeline);
    }

    /**
     * Returns current Pipeling of the Lime Light
     *
     * @return Pipelinge
     */
    public Integer getPipelineInt() {
        Integer pipe = (int) pipeline.getDouble(0.0);
        return pipe;
    }

    public StreamType getStream() {
        double st = stream.getDouble(0.0);
        StreamType mode = StreamType.getByValue(st);
        return mode;
    }

    /**
     * stream Sets limelight’s streaming mode
     * <p>
     * kStandard - Side-by-side streams if a webcam is attached to Limelight kPiPMain - The secondary camera stream is placed in the
     * lower-right corner of the primary camera stream kPiPSecondary - The primary camera stream is placed in the lower-right corner of the
     * secondary camera stream
     */
    public void setStream(StreamType stream) {
        m_table.getEntry("stream").setValue(stream.getValue());
    }

    public Snapshot getSnapshot() {
        double snshot = snapshot.getDouble(0.0);
        Snapshot mode = Snapshot.getByValue(snshot);
        return mode;
    }

    /**
     * snapshot Allows users to take snapshots during a match
     * <p>
     * kon - Stop taking snapshots koff - Take two snapshots per second
     */
    public void setSnapshot(Snapshot snapshot) {
        m_table.getEntry("snapshot").setValue(snapshot.getValue());
    }

    /**
     * Limelight posts three raw contours to NetworkTables that are not influenced by your grouping mode. That is, they are filtered with your
     * pipeline parameters, but never grouped. X and Y are returned in normalized screen space (-1 to 1) rather than degrees. *
     */
    public double getAdvanced_RotationToTarget(Advanced_Target raw) {
        NetworkTableEntry txRaw = m_table.getEntry("tx" + raw.getValue());
        double x = txRaw.getDouble(0.0);
        return x;
    }
    // *************** Advanced Usage with Raw Contours *********************

    public double getAdvanced_degVerticalToTarget(Advanced_Target raw) {
        NetworkTableEntry tyRaw = m_table.getEntry("ty" + raw.getValue());
        double y = tyRaw.getDouble(0.0);
        return y;
    }

    public double getAdvanced_TargetArea(Advanced_Target raw) {
        NetworkTableEntry taRaw = m_table.getEntry("ta" + raw.getValue());
        double a = taRaw.getDouble(0.0);
        return a;
    }

    public double getAdvanced_Skew_Rotation(Advanced_Target raw) {
        NetworkTableEntry tsRaw = m_table.getEntry("ts" + raw.getValue());
        double s = tsRaw.getDouble(0.0);
        return s;
    }

    public double[] getAdvanced_RawCrosshair(Advanced_Crosshair raw) {
        double[] crosshars = new double[2];
        crosshars[0] = getAdvanced_RawCrosshair_X(raw);
        crosshars[1] = getAdvanced_RawCrosshair_Y(raw);
        return crosshars;
    }
    // Raw Crosshairs:
    // If you are using raw targeting data, you can still utilize your calibrated
    // crosshairs:

    public double getAdvanced_RawCrosshair_X(Advanced_Crosshair raw) {
        NetworkTableEntry cxRaw = m_table.getEntry("cx" + raw.getValue());
        double x = cxRaw.getDouble(0.0);
        return x;
    }

    public double getAdvanced_RawCrosshair_Y(Advanced_Crosshair raw) {
        NetworkTableEntry cyRaw = m_table.getEntry("cy" + raw.getValue());
        double y = cyRaw.getDouble(0.0);
        return y;
    }

    class PeriodicRunnable implements java.lang.Runnable {
        public void run() {
            resetPilelineLatency();
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            isConnected = getPipelineLatency() != 0.0;
        }
    }
}
