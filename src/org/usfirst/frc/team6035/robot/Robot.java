/*                                           			TEAM 6035 													      */

/**************************************************************************************************************************/
/**************************************************************************************************************************/
/**************************************************************************************************************************/
/**************************************************************************************************************************/

// 24/12/17

package org.usfirst.frc.team6035.robot;
import edu.wpi.first.wpilibj.*; // WPILBJ


public class Robot extends IterativeRobot {

    Auto auto = new Auto();
    Gyro gyro = new Gyro();
    RobotInit robotInit = new RobotInit();
    Tele tele = new Tele();



	public void robotInit() {
	    robotInit.robotInitiate();
	}


	public void autonomousInit() {


        auto.autonomousInit();
        gyro.calibrate();
	}


    public void autonomousPeriodic() {
	    auto.autonomousPeriodic();

    }


	public void teleopInit() {


		gyro.calibrate();
		teleopPeriodic();


	}
	public void teleopPeriodic(){
        tele.teleopPeriodic();
    }

}

