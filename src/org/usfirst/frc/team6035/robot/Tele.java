package org.usfirst.frc.team6035.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Joystick;

/**
 * Created by Gabriel on 22/12/17.
 */
public class Tele {
    AHRS ahrs;
    Auto auto = new Auto();
    Gyro gyro = new Gyro();
    RobotInit robotInit = new RobotInit();
    Robot robot = new Robot();
    public void teleopPeriodic() {


        // Code to be executed in a loop while in operator control
        robotInit.autoloopcounter = 0;
        while (robot.isOperatorControl() && robot.isEnabled()) {

            robotInit.stick = new Joystick(0); // Defining The Joystick
            robotInit.xbox = new Joystick(1); // Defining The Xbox Controller
            robotInit.autoloopcounter++;

            /* test code for victor setup

            if (robotInit.xbox.getRawButton(1)){
                robotInit.hasBeenPressed = true;
            }
            else if (robotInit.autoloopcounter > 2000){
                robotInit.hasBeenPressed = false;
            }
            else if (robotInit.hasBeenPressed == true){
                robotInit.victorTest.set(0.2);
            }
            */



            // If button 2 on xbox controller, run grub at 100% speed

            if (robotInit.xbox.getRawButton(2)) {
                robotInit.victor2.set(1);
            }

            // If button 3 on xbox controller, run grub at 0% speed

            else if (robotInit.xbox.getRawButton(3)) {
                robotInit.victor2.set(0);
            }

            // If button 4 on xbox controller, run grub at 50% speed

            else if (robotInit.xbox.getRawButton(4)) {
                robotInit.victor2.set(0.5);
            }

            // A button to start Ball Shooter and Agrivator

            else if (robotInit.xbox.getRawButton(6)) {
                robotInit.victor1.set(0.8); //Ball Shooter
                robotInit.victor3.set(0.15); // Agrivator
            }

            // A button to inverse controls for joystick

            else if (robotInit.stick.getRawButton(2)) {
                robotInit.reverseY = 1;
                robotInit.reverseX = -1;
            }
            // An else statement setting everything to normal in case nothing is pressed

            else {
                robotInit.reverseY = -1;
                robotInit.reverseX = 1;
                robotInit.victor1.set(0);
                robotInit.victor3.set(0);
                robotInit.isReversing = false;
                robotInit.victorTest.set(0);
            }

            // Drive Train

            robotInit.myRobot.arcadeDrive(robotInit.reverseY * robotInit.stick.getY(), robotInit.reverseX * robotInit.stick.getX());

        }

    }
}
