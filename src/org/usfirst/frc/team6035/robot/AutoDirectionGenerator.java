package org.usfirst.frc.team6035.robot;

public class AutoDirectionGenerator {

	public static void main (String[] args) {
		generateDriveStraight();
		generateLeftToLeftSwitch();
		generateRightToLeftSwitch();
		generateLeftToRightSwitch();
		generateMiddleToLeftSwitch();
		generateMiddleToRightSwitch();
		generateRightToRightSwitch();
		
		generateLeftToLeftScale();
		generateRightToLeftScale();
		generateLeftToRightScale();
		generateMiddleToRightScale();
		generateMiddleToLeftScale();
		generateRightToRightScale();
		
	}
	
	private static void generateDriveStraight() {
			final double[][] wayPoints = {
					{0, 21}, {14 ,21}
			};
			
			generate("DriveStraight", wayPoints, 5);
		}
	private static void generateLeftToLeftSwitch() {
		final double[][] wayPoints = {
				{0, 21}, {7, 21}, {10, 24}, {13, 24}, {14, 23}, {14, 20}
		};
		
		generate("LeftToLeftSwitch", wayPoints, 10);
	}
	private static void generateLeftToRightSwitch() {
		final double[][] wayPoints = {	
			 {0, 21}, {8, 21}, {12, 24}, {19, 24}, {19, 22}, {19, 3}, {15, 3}, {14, 4}, {14, 7}
		};
		
		generate("LeftToRightSwitch", wayPoints, 10);
	}
	private static void generateMiddleToLeftSwitch() {
		final double[][] wayPoints = {
				{0, 14}, {5, 14}, {9 , 25}, {11, 25}, {13, 23}, {14, 22}, {14, 20}
		};
		
		generate("MiddleToLeftSwitch", wayPoints, 10);
	}
	private static void generateRightToLeftSwitch() {
		final double[][] wayPoints = {
				{0, 4},  {6, 4}, {14, 4}, {20, 4}, {20, 23}, {18, 25}, {16, 25}, {14, 23}, {14, 20}
		};
		
		generate("RightToLeftSwitch", wayPoints, 10);
	}
	private static void generateRightToRightSwitch() {
		final double[][] wayPoints = {
				 {0, 4}, {10, 3}, {12, 3}, {13.5, 4}, {14, 5}, {14, 7}
		};
		
		generate("RightToRightSwitch", wayPoints, 10);
	}
	private static void generateMiddleToRightSwitch() {

		final double[][] wayPoints = {
				{0,14}, {6, 14}, {6, 6}, {8, 3}, {12, 3}, {14, 5}, {14, 7}
		};
		
		generate("MiddleToRightSwitch", wayPoints, 10);
	}

	private static void generateLeftToLeftScale() {

		final double[][] wayPoints = {
				{0, 0}
		};
		
		generate("LeftToLeftScale", wayPoints, 10);
	}
	private static void generateMiddleToLeftScale() {
		final double[][] wayPoints = {
				{0, 0}
		};
		
		generate("MiddleToLeftScale", wayPoints, 10);
	}
	private static void generateRightToLeftScale() {
		final double[][] wayPoints = {
				{0, 0}
		};
		
		generate("RightToLeftScale", wayPoints, 10);
	}
	private static void generateRightToRightScale() {
		final double[][] wayPoints = {
				{0, 0}
		};
		
		generate("RightToRightScale", wayPoints, 10);
	}
	private static void generateMiddleToRightScale() {
	
		final double[][] wayPoints = {
				{0, 0}
		};
		
		generate("MiddleToRightScale", wayPoints, 10);
	}
	private static void generateLeftToRightScale() {
		final double[][] wayPoints = {
				{0, 0}
		};
		
		generate("LeftToRightScale", wayPoints, 10);
	}

	
	
	private static void generate(String className, double[][] wayPoints, double totalTime) {
		PathPlanner planner = new PathPlanner(wayPoints);
		planner.calculate(totalTime, 0.02, 2);
		double[] smoothLeftSpeeds = PathPlanner.getYVector(planner.smoothLeftVelocity);
		double[] smoothRightSpeeds = PathPlanner.getYVector(planner.smoothRightVelocity);
		JavaFileCreator.createClass(className, smoothLeftSpeeds, smoothRightSpeeds);
	}
}

