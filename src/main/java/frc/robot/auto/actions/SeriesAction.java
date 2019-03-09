package frc.robot.auto.actions;

import java.util.ArrayList;
import java.util.List;

/** Executes one action at a time. Useful as a member of {@link ParallelAction} */
public class SeriesAction implements Action {

  private final ArrayList<Action> mRemainingActions;
  private Action mCurAction;

  public SeriesAction(List<Action> actions) {
    mRemainingActions = new ArrayList<>(actions);
    mCurAction = null;
  }

  @Override
  public boolean isFinished() {
    return mRemainingActions.isEmpty() && mCurAction == null;
  }

  @Override
  public void update() {
    if (mCurAction == null) {
      if (mRemainingActions.isEmpty()) {
        return;
      }
      mCurAction = mRemainingActions.remove(0);
      mCurAction.start();
    }
    mCurAction.update();
    if (mCurAction.isFinished()) {
      mCurAction.done();
      mCurAction = null;
    }
  }

  @Override
  public void done() {}

  @Override
  public void start() {}
}
