package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.SimpleHatch;

public class SimpleHatchPlace extends AutoModeBase {
  @Override
  protected void routine() throws AutoModeEndedException {
    runAction(new SimpleHatch());
    // runAction(new OpenLoopDrive(-0.175, -0.175, 0.75));
  }
}
