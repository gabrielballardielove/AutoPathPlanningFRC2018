package org.usfirst.frc.team6035.robot;

public class AutoDirectionGenerator {

	public static void main (String[] args) {
		generateDriveStraight();
		generateLeftToLeftSwitch();
		generateRightToLeftSwitch();
		generateLeftToRightSwitch();
		generateMiddleToRightSwitch();
		generateRightToRightSwitch();
		
		generateDriveStraight();
		generateLeftToLeftScale();
		generateRightToLeftScale();
		generateLeftToRightScale();
		generateMiddleToRightScale();
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
				{0, 0}
		};
		
		generate("LeftToLeftSwitch", wayPoints, 5);
	}
	
	private static void generateLeftToRightSwitch() {
		final double[][] wayPoints = {
				{0, 0}
		};
		
		generate("LeftToRightSwitch", wayPoints, 10);
	}
	
	private static void generateLeftToRightScale() {
		final double[][] wayPoints = {
				{0, 0}
		};
		
		generate("LeftToRightScale", wayPoints, 10);
	}
	private static void generateMiddleToLeftSwitch() {
		final double[][] wayPoints = {
				{0, 0}
		};
		
		generate("MiddleToLeftSwitch", wayPoints, 5);
	}
	private static void generateRightToLeftSwitch() {
		final double[][] wayPoints = {
				{0, 0}
		};
		
		generate("RightToLeftSwitch", wayPoints, 10);
	}
	private static void generateRightToRightSwitch() {
		final double[][] wayPoints = {
				{0, 0}
		};
		
		generate("RightToRightSwitch", wayPoints, 5);
	}
	private static void generateMiddleToRightSwitch() {
		final double[][] wayPoints = {
				{0, 0}
		};
		
		generate("MiddleToRightSwitch", wayPoints, 5);
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
	private static void generate(String className, double[][] wayPoints, double totalTime) {
		PathPlanner pathPlanner = new PathPlanner(wayPoints);
		pathPlanner.calculate(totalTime, 0.02, 2);
		double[] smoothLeftSpeeds = pathPlanner.getYVector(pathPlanner.smoothLeftVelocity);
		double[] smoothRightSpeeds = pathPlanner.getYVector(pathPlanner.smoothRightVelocity);
		JavaFileCreator.createClass(className, smoothLeftSpeeds, smoothRightSpeeds);
	}
}

