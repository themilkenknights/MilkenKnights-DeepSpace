package frc.robot.lib.drivers;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class MkGyro extends AHRS {

  public MkGyro(SPI.Port spi_port_id) {
    super(spi_port_id);
  }

  public double getSwerd() {
    double a = getYaw();
    if (a < 0) {
      a = 180 + -a;
    }
    return a;
  }

  /**
   * @return Fused Heading from -180deg to 180deg
   * TODO Verify method
   */
  public double getFusedNormalized() {
    double heading = getFusedHeading();
    return heading < 180.0 ? -heading : -(heading - 360.0);
  }
}
