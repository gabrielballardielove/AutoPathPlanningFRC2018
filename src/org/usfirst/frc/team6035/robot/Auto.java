package org.usfirst.frc.team6035.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Created by Gabriel on 22/12/17.
 */
public class Auto {
    AHRS ahrs;
    Auto auto = new Auto();
    Gyro gyro = new Gyro();
    RobotInit robotInit = new RobotInit();
    Robot robot = new Robot();
    public void autonomousInit() {


        // Setting up the smart dashboard autonomous chooser

        robotInit.autoSelected = SmartDashboard.getString("Auto Selector", robotInit.chooser.getSelected());
        System.out.println("Auto selected: " + robotInit.autoSelected);
        robotInit.autoloopcounter = 0;


        // Calibrate The Gyro

        gyro.calibrate();
    }




    public void autonomousPeriodic() {

        BluePathAVelocities bluePathAVelocities = new BluePathAVelocities();

        for (int i = 0; i < bluePathAVelocities.BluePathALeftVelocities.length; i++) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            robotInit.myRobot.drive(0.4, ((bluePathAVelocities.BluePathALeftVelocities[i]/10)-(bluePathAVelocities.BluePathARightVelocities[i] / 10)));

        }


    }

}
