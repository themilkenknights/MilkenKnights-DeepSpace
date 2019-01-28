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
}
