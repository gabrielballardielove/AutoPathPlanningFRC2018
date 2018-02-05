package org.usfirst.frc.team6035.robot;

import org.usfirst.frc.team6035.robot.util.*;

class AutoDirectionGenerator {

    public static void main(String[] args) {
        generateDriveStraight();

        generateLeftToLeftSwitch();
//        generateMiddleToLeftSwitch();
        generateRightToLeftSwitch();
//
//        generateLeftToRightSwitch();
//        generateMiddleToRightSwitch();
//        generateRightToRightSwitch();
//
//        generateLeftToLeftScale();
//        generateMiddleToLeftScale();
//        generateRightToLeftScale();
//
//        generateLeftToRightScale();
//        generateMiddleToRightScale();
//        generateRightToRightScale();
    }

    private static void generateDriveStraight() {
        final double[][] wayPoints = {
                {0, 21}, {14, 21}
        };
        generate("DriveStraight", wayPoints);
    }

    private static void generateLeftToLeftSwitch() {
        final double[][] wayPoints = {
                {0, 21}, {8, 21}, {12, 24}, {15, 24}, {16, 23}, {16, 20}
        };
        generate("LeftToLeftSwitch", wayPoints);
    }

    private static void generateMiddleToLeftSwitch() {
//        final double[][] wayPoints = { };
//        generate("MiddleToLeftSwitch", wayPoints);
    }

    private static void generateRightToLeftSwitch() {
        final double[][] wayPoints = {
                {0, 4}, {6, 4}, {14, 4}, {22, 4}, {22, 23}, {16, 23}, {16, 20}
        };
        generate("RightToLeftSwitch", wayPoints);
    }

    private static void generateLeftToRightSwitch() {
//        final double[][] wayPoints = { };
//        generate("LeftToRightSwitch", wayPoints);
    }

    private static void generateMiddleToRightSwitch() {
//        final double[][] wayPoints = { };
//        generate("MiddleToRightSwitch", wayPoints);
    }

    private static void generateRightToRightSwitch() {
//        final double[][] wayPoints = { };
//        generate("RightToRightSwitch", wayPoints);
    }

    private static void generateLeftToLeftScale() {
//        final double[][] wayPoints = { };
//        generate("LeftToLeftScale", wayPoints);
    }

    private static void generateMiddleToLeftScale() {
//        final double[][] wayPoints = { };
//        generate("MiddleToLeftScale", wayPoints);
    }

    private static void generateRightToLeftScale() {
//        final double[][] wayPoints = { };
//        generate("RightToLeftScale", wayPoints);
    }

    private static void generateLeftToRightScale() {
//        final double[][] wayPoints = { };
//        generate("LeftToRightScale", wayPoints);
    }

    private static void generateMiddleToRightScale() {
//        final double[][] wayPoints = { };
//        generate("MiddleToRightScale", wayPoints);
    }

    private static void generateRightToRightScale() {
//        final double[][] wayPoints = { };
//        generate("RightToRightScale", wayPoints);
    }

    //------------------------------------------------------------------------------------------------------------------

    private static void generate(String className, double[][] wayPoints) {
        Track track = TrackFinder.findTrack(wayPoints);
        Plotter.plotPath(className, track);
        JavaFileCreator.createClass(className, track);
    }

    /**
     * Utility class. Hide ctor.
     */
    private AutoDirectionGenerator() { }

}
