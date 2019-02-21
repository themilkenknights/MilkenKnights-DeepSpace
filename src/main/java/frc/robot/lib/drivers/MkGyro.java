package frc.robot.lib.drivers;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants.DRIVE;

public class MkGyro extends AHRS {

    public MkGyro(SPI.Port spi_port_id) {
        super(spi_port_id, DRIVE.kNavXUpdateRate);
    }

/*	@Override
	public double getFusedHeading() {
		return (double) super.getAngle();
	} */

}
