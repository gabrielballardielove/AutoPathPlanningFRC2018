package org.usfirst.frc.team6035.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Created by Gabriel on 22/12/17.
 */
public class RobotInit {
    Robot robot = new Robot();
    AHRS ahrs;
    Auto auto = new Auto();
    Gyro gyro = new Gyro();

    private static double kAngleSetpoint = 0.001; // Not quite sure...
    private Integer mode = 1; // Defualt Mode
    boolean rotateFlag = true; //Flag for when to stop roating in auto (SET BACK TO true WHEN NOT IN USE!)

    VictorSP victor1 = new VictorSP(2); // VictorSP Motor Controller (Ball Shooter)
    VictorSP victor2 = new VictorSP(3); // VictorSP Motor Controller (GRUB)
    VictorSP victor3 = new VictorSP(4); // VictorSP Motor Controller (Agitator)
    VictorSP Left = new VictorSP(0); // Left Motors for Robot
    VictorSP Right = new VictorSP(1); // Right Motors for Robot
    VictorSP victorTest = new VictorSP(5);


    //Essentials

    RobotDrive myRobot; //Drive Train
    Joystick stick; //Joystick
    Joystick xbox; // Xbox Controller
    int autoloopcounter = 0; // Auto Loop Counter for autonomous (kind of like a timer).
    private UsbCamera camera = null;


    // Controls

    double reverseX; // Inverse X Axis
    double reverseY; // Inverse Y Axis
    boolean isReversing = false; // Button Controls (Are Needed)
    boolean hasBeenPressed = false;


    //SMART DASHBOARD AUTO SELECT

    final String defaultAuto = "Default";
    final String customAuto = "My Auto";
    final String posA = "Pos A";
    final String posB = "Pos B";
    final String posC = "Pos C";
    String autoSelected;
    SendableChooser<String> chooser = new SendableChooser<>();


    public void robotInitiate() {
        //SMART DASHBOARD
        chooser.addDefault("Default Auto", defaultAuto);
        chooser.addObject("Pos A", posA); //Start pos 1
        chooser.addObject("Pos B", posB); //Start pos 2
        chooser.addObject("Pos C", posC); //Start pos 3
        SmartDashboard.putData("Auto choices", chooser);


        //DRIVE Stuff
        myRobot = new RobotDrive(0, 1);
        stick = new Joystick(0);
        xbox = new Joystick(1);
        stick.getTrigger();
        myRobot.setExpiration(0.1);
        myRobot.setExpiration(0.1);

    }
}
