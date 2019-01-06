package frc.robot.util.state;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class MatchData {

    public static MatchData defaultMatch = new MatchData(DriverStation.MatchType.None, 0,
            Alliance.Blue);
    public DriverStation.MatchType matchType;
    public int matchNumber;
    public DriverStation.Alliance alliance;

    public MatchData(DriverStation.MatchType matchType, int matchNumber,
                     DriverStation.Alliance alliance) {
        this.matchType = matchType;
        this.matchNumber = matchNumber;
        this.alliance = alliance;
    }
}
