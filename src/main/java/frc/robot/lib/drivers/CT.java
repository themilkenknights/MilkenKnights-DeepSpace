package frc.robot.lib.drivers;

import com.ctre.phoenix.ErrorCode;

import frc.robot.lib.util.Logger;

public class CT {
  public static void RE(ErrorCode errorCode) {
    if (errorCode != ErrorCode.OK) {
      Logger.logErrorWithTrace(errorCode.toString());
    }
  }
}
