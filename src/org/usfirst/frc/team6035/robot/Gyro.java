package org.usfirst.frc.team6035.robot;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

/**
 * Created by Gabriel on 22/12/17.
 */
public class Gyro {
    Robot robot = new Robot();
    AHRS ahrs;
    // Function used to calibrate/reset the gyro
    public void calibrate() {
        ahrs.reset();
    }
    // The autonomous code to be ran periodically
    public void getData(){
        ahrs.getDisplacementY();
        ahrs.getDisplacementX();
        ahrs.getRawGyroX();
        ahrs.getRawGyroY();
        ahrs.getRawGyroZ();
    }
    public void gyroInit(){
            try {
                ahrs = new AHRS(SPI.Port.kMXP);
            } catch (RuntimeException ex) {
                DriverStation.reportError("Error instantiating navX MXP: " + ex.getMessage(), true);
            }
        }
    }

