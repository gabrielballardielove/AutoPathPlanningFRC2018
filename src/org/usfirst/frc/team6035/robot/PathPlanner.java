package org.usfirst.frc.team6035.robot;

import java.awt.Color;

import java.awt.GraphicsEnvironment;
import java.util.LinkedList;
import java.util.List;
import java.io.PrintWriter;
import java.io.FileReader;
import java.io.FileNotFoundException;

/**
 * This Class provides many useful algorithms for Robot Path Planning. It uses optimization techniques and knowledge
 * of Robot Motion in order to calculate smooth path trajectories, if given only discrete waypoints. The Benefit of these optimization
 * algorithms are very efficient path planning that can be used to Navigate in Real-time.
 *
 * This Class uses a method of Gradient Decent, and other optimization techniques to produce smooth Velocity profiles
 * for both left and right wheels of a differential drive robot.
 *
 * This Class does not attempt to calculate quintic or cubic splines for best fitting a curve. It is for this reason, the algorithm can be ran
 * on embedded devices with very quick computation times.
 *
 * The output of this function are independent velocity profiles for the left and right wheels of a differential drive chassis. The velocity
 * profiles start and end with 0 velocity and maintain smooth transitions throughout the path.
 *
 */

public class PathPlanner  {

    //Path Variables
    public double[][] origPath;
    public double[][] nodeOnlyPath;
    public double[][] smoothPath;
    public double[][] leftPath;
    public double[][] rightPath;

    //Orig Velocity
    public double[][] origCenterVelocity;
    public double[][] origRightVelocity;
    public double[][] origLeftVelocity;

    //smooth velocity
    public double[][] smoothCenterVelocity;
    public double[][] smoothRightVelocity;
    public double[][] smoothLeftVelocity;

    //accumulated heading
    public double[][] heading;

    double totalTime;
    double totalDistance;
    double numFinalPoints;

    double pathAlpha;
    double pathBeta;
    double pathTolerance;

    double velocityAlpha;
    double velocityBeta;
    double velocityTolerance;









    /**
     * Constructor, takes a Path of Way Points defined as a double array of column vectors representing the global
     * cartesian points of the path in {x,y} coordinates. The waypoint are traveled from one point to the next in sequence.
     * <p>
     * For example: here is a properly formated waypoint array
     * <p>
     * double[][] waypointPath = new double[][]{
     * {1, 1},
     * {5, 1},
     * {9, 12},
     * {12, 9},
     * {15,6},
     * {15, 4}
     * };
     * <p>
     * This path goes from {1,1} -> {5,1} -> {9,12} -> {12, 9} -> {15,6} -> {15,4}
     * <p>
     * The units of these coordinates are position units assumed by the user (i.e inch, foot, meters)
     *
     * @param path
     */
    public PathPlanner(double[][] path) {
        this.origPath = doubleArrayCopy(path);

        //default values DO NOT MODIFY;
        pathAlpha = 0.7;
        pathBeta = 0.3;
        pathTolerance = 0.0000001;

        velocityAlpha = 0.1;
        velocityBeta = 0.3;
        velocityTolerance = 0.0000001;
    }



    public static void print(double[] path) {
        System.out.println("X: \t Y:");

        for (double u : path)
            System.out.println(u);
    }


    /**
     * Prints Cartesian Coordinates to the System Output as Column Vectors in the Form X	Y
     *
     * @param path
     */
    public static void print(double[][] path) {
        System.out.println("X: \t Y:");

        for (double[] u : path)
            System.out.println(u[0] + "\t" + u[1]);
    }

    /**
     * Performs a deep copy of a 2 Dimensional Array looping thorough each element in the 2D array
     * <p>
     * BigO: Order N x M
     *
     * @param arr
     * @return
     */
    public static double[][] doubleArrayCopy(double[][] arr) {

        //size first dimension of array
        double[][] temp = new double[arr.length][arr[0].length];

        for (int i = 0; i < arr.length; i++) {
            //Resize second dimension of array
            temp[i] = new double[arr[i].length];

            //Copy Contents
            for (int j = 0; j < arr[i].length; j++)
                temp[i][j] = arr[i][j];
        }

        return temp;

    }



    /**
     * Method upsamples the Path by linear injection. The result providing more waypoints along the path.
     * <p>
     * BigO: Order N * injection#
     *
     * @param orig
     * @param numToInject
     * @return
     */
    public double[][] inject(double[][] orig, int numToInject) {
        double morePoints[][];

        //create extended 2 Dimensional array to hold additional points
        morePoints = new double[orig.length + ((numToInject) * (orig.length - 1))][2];

        int index = 0;

        //loop through original array
        for (int i = 0; i < orig.length - 1; i++) {
            //copy first
            morePoints[index][0] = orig[i][0];
            morePoints[index][1] = orig[i][1];
            index++;

            for (int j = 1; j < numToInject + 1; j++) {
                //calculate intermediate x points between j and j+1 original points
                morePoints[index][0] = j * ((orig[i + 1][0] - orig[i][0]) / (numToInject + 1)) + orig[i][0];

                //calculate intermediate y points  between j and j+1 original points
                morePoints[index][1] = j * ((orig[i + 1][1] - orig[i][1]) / (numToInject + 1)) + orig[i][1];

                index++;
            }
        }

        //copy last
        morePoints[index][0] = orig[orig.length - 1][0];
        morePoints[index][1] = orig[orig.length - 1][1];
        index++;

        return morePoints;
    }


    /**
     * Optimization algorithm, which optimizes the data points in path to create a smooth trajectory.
     * This optimization uses gradient descent. While unlikely, it is possible for this algorithm to never
     * converge. If this happens, try increasing the tolerance level.
     * <p>
     * BigO: N^x, where X is the number of of times the while loop iterates before tolerance is met.
     *
     * @param path
     * @param weight_data
     * @param weight_smooth
     * @param tolerance
     * @return
     */
    public double[][] smoother(double[][] path, double weight_data, double weight_smooth, double tolerance) {

        //copy array
        double[][] newPath = doubleArrayCopy(path);

        double change = tolerance;
        while (change >= tolerance) {
            change = 0.0;
            for (int i = 1; i < path.length - 1; i++)
                for (int j = 0; j < path[i].length; j++) {
                    double aux = newPath[i][j];
                    newPath[i][j] += weight_data * (path[i][j] - newPath[i][j]) + weight_smooth * (newPath[i - 1][j] + newPath[i + 1][j] - (2.0 * newPath[i][j]));
                    change += Math.abs(aux - newPath[i][j]);
                }
        }

        return newPath;

    }

    /**
     * reduces the path into only nodes which change direction. This allows the algorithm to know at what points
     * the original WayPoint vector changes.
     * <p>
     * BigO: Order N + Order M, Where N is length of original Path, and M is length of Nodes found in Path
     *
     * @param path
     * @return
     */
    public static double[][] nodeOnlyWayPoints(double[][] path) {

        List<double[]> li = new LinkedList<double[]>();

        //save first value
        li.add(path[0]);

        //find intermediate nodes
        for (int i = 1; i < path.length - 1; i++) {
            //calculate direction
            double vector1 = Math.atan2((path[i][1] - path[i - 1][1]), path[i][0] - path[i - 1][0]);
            double vector2 = Math.atan2((path[i + 1][1] - path[i][1]), path[i + 1][0] - path[i][0]);

            //determine if both vectors have a change in direction
            if (Math.abs(vector2 - vector1) >= 0.01)
                li.add(path[i]);
        }

        //save last
        li.add(path[path.length - 1]);

        //re-write nodes into new 2D Array
        double[][] temp = new double[li.size()][2];

        for (int i = 0; i < li.size(); i++) {
            temp[i][0] = li.get(i)[0];
            temp[i][1] = li.get(i)[1];
        }

        return temp;
    }


    /**
     * Returns Velocity as a double array. The First Column vector is time, based on the time step, the second vector
     * is the velocity magnitude.
     * <p>
     * BigO: order N
     *
     * @param smoothPath
     * @param timeStep
     * @return
     */
    double[][] velocity(double[][] smoothPath, double timeStep) {
        double[] dxdt = new double[smoothPath.length];
        double[] dydt = new double[smoothPath.length];
        double[][] velocity = new double[smoothPath.length][2];

        //set first instance to zero
        dxdt[0] = 0;
        dydt[0] = 0;
        velocity[0][0] = 0;
        velocity[0][1] = 0;
        heading[0][1] = 0;

        for (int i = 1; i < smoothPath.length; i++) {
            dxdt[i] = (smoothPath[i][0] - smoothPath[i - 1][0]) / timeStep;
            dydt[i] = (smoothPath[i][1] - smoothPath[i - 1][1]) / timeStep;

            //create time vector
            velocity[i][0] = velocity[i - 1][0] + timeStep;
            heading[i][0] = heading[i - 1][0] + timeStep;

            //calculate velocity
            velocity[i][1] = Math.sqrt(Math.pow(dxdt[i], 2) + Math.pow(dydt[i], 2));
        }


        return velocity;

    }




    /**
     * optimize velocity by minimizing the error distance at the end of travel
     * when this function converges, the fixed velocity vector will be smooth, start
     * and end with 0 velocity, and travel the same final distance as the original
     * un-smoothed velocity profile
     * <p>
     * This Algorithm may never converge. If this happens, reduce tolerance.
     *
     * @param smoothVelocity
     * @param origVelocity
     * @param tolerance
     * @return
     */
    double[][] velocityFix(double[][] smoothVelocity, double[][] origVelocity, double tolerance) {

		/*pseudo
		 * 1. Find Error Between Original Velocity and Smooth Velocity
		 * 2. Keep increasing the velocity between the first and last node of the smooth Velocity by a small amount
		 * 3. Recalculate the difference, stop if threshold is met or repeat step 2 until the final threshold is met.
		 * 3. Return the updated smoothVelocity
		 */

        //calculate error difference
        double[] difference = errorSum(origVelocity, smoothVelocity);


        //copy smooth velocity into new Vector
        double[][] fixVel = new double[smoothVelocity.length][2];

        for (int i = 0; i < smoothVelocity.length; i++) {
            fixVel[i][0] = smoothVelocity[i][0];
            fixVel[i][1] = smoothVelocity[i][1];
        }

        //optimize velocity by minimizing the error distance at the end of travel
        //when this converges, the fixed velocity vector will be smooth, start
        //and end with 0 velocity, and travel the same final distance as the original
        //un-smoothed velocity profile
        double increase = 0.0;
        while (Math.abs(difference[difference.length - 1]) > tolerance) {
            increase = difference[difference.length - 1] / 1 / 50;

            for (int i = 1; i < fixVel.length - 1; i++)
                fixVel[i][1] = fixVel[i][1] - increase;

            difference = errorSum(origVelocity, fixVel);
        }

        //fixVel =  smoother(fixVel, 0.001, 0.001, 0.0000001);
        return fixVel;

    }


    /**
     * This method calculates the integral of the Smooth Velocity term and compares it to the Integral of the
     * original velocity term. In essence we are comparing the total distance by the original velocity path and
     * the smooth velocity path to ensure that as we modify the smooth Velocity it still covers the same distance
     * as was intended by the original velocity path.
     * <p>
     * BigO: Order N
     *
     * @param origVelocity
     * @param smoothVelocity
     * @return
     */
    private double[] errorSum(double[][] origVelocity, double[][] smoothVelocity) {
        //copy vectors
        double[] tempOrigDist = new double[origVelocity.length];
        double[] tempSmoothDist = new double[smoothVelocity.length];
        double[] difference = new double[smoothVelocity.length];


        double timeStep = origVelocity[1][0] - origVelocity[0][0];

        //copy first elements
        tempOrigDist[0] = origVelocity[0][1];
        tempSmoothDist[0] = smoothVelocity[0][1];


        //calculate difference
        for (int i = 1; i < origVelocity.length; i++) {
            tempOrigDist[i] = origVelocity[i][1] * timeStep + tempOrigDist[i - 1];
            tempSmoothDist[i] = smoothVelocity[i][1] * timeStep + tempSmoothDist[i - 1];

            difference[i] = tempSmoothDist[i] - tempOrigDist[i];

        }

        return difference;
    }

    /**
     * This method calculates the optimal parameters for determining what amount of nodes to inject into the path
     * to meet the time restraint. This approach uses an iterative process to inject and smooth, yielding more desirable
     * results for the final smooth path.
     * <p>
     * Big O: Constant Time
     *
     * @param numNodeOnlyPoints
     * @param maxTimeToComplete
     * @param timeStep
     */
    public int[] injectionCounter2Steps(double numNodeOnlyPoints, double maxTimeToComplete, double timeStep) {
        int first = 0;
        int second = 0;
        int third = 0;

        double oldPointsTotal = 0;

        numFinalPoints = 0;

        int[] ret = null;

        double totalPoints = maxTimeToComplete / timeStep;

        if (totalPoints < 100) {
            double pointsFirst = 0;
            double pointsTotal = 0;


            for (int i = 4; i <= 6; i++)
                for (int j = 1; j <= 8; j++) {
                    pointsFirst = i * (numNodeOnlyPoints - 1) + numNodeOnlyPoints;
                    pointsTotal = (j * (pointsFirst - 1) + pointsFirst);

                    if (pointsTotal <= totalPoints && pointsTotal > oldPointsTotal) {
                        first = i;
                        second = j;
                        numFinalPoints = pointsTotal;
                        oldPointsTotal = pointsTotal;
                    }
                }

            ret = new int[]{first, second, third};
        } else {

            double pointsFirst = 0;
            double pointsSecond = 0;
            double pointsTotal = 0;

            for (int i = 1; i <= 5; i++)
                for (int j = 1; j <= 8; j++)
                    for (int k = 1; k < 8; k++) {
                        pointsFirst = i * (numNodeOnlyPoints - 1) + numNodeOnlyPoints;
                        pointsSecond = (j * (pointsFirst - 1) + pointsFirst);
                        pointsTotal = (k * (pointsSecond - 1) + pointsSecond);

                        if (pointsTotal <= totalPoints) {
                            first = i;
                            second = j;
                            third = k;
                            numFinalPoints = pointsTotal;
                        }
                    }

            ret = new int[]{first, second, third};
        }


        return ret;
    }

    /**
     * Calculates the left and right wheel paths based on robot track width
     * <p>
     * Big O: 2N
     *
     * @param smoothPath      - center smooth path of robot
     * @param robotTrackWidth - width between left and right wheels of robot of skid steer chassis.
     */
    public void leftRight(double[][] smoothPath, double robotTrackWidth) {

        double[][] leftPath = new double[smoothPath.length][2];
        double[][] rightPath = new double[smoothPath.length][2];

        double[][] gradient = new double[smoothPath.length][2];

        for (int i = 0; i < smoothPath.length - 1; i++)
            gradient[i][1] = Math.atan2(smoothPath[i + 1][1] - smoothPath[i][1], smoothPath[i + 1][0] - smoothPath[i][0]);

        gradient[gradient.length - 1][1] = gradient[gradient.length - 2][1];


        for (int i = 0; i < gradient.length; i++) {
            leftPath[i][0] = (robotTrackWidth / 2 * Math.cos(gradient[i][1] + Math.PI / 2)) + smoothPath[i][0];
            leftPath[i][1] = (robotTrackWidth / 2 * Math.sin(gradient[i][1] + Math.PI / 2)) + smoothPath[i][1];

            rightPath[i][0] = robotTrackWidth / 2 * Math.cos(gradient[i][1] - Math.PI / 2) + smoothPath[i][0];
            rightPath[i][1] = robotTrackWidth / 2 * Math.sin(gradient[i][1] - Math.PI / 2) + smoothPath[i][1];

            //convert to degrees 0 to 360 where 0 degrees is +X - axis, accumulated to aline with WPI sensor
            double deg = Math.toDegrees(gradient[i][1]);

            gradient[i][1] = deg;

            if (i > 0) {
                if ((deg - gradient[i - 1][1]) > 180)
                    gradient[i][1] = -360 + deg;

                if ((deg - gradient[i - 1][1]) < -180)
                    gradient[i][1] = 360 + deg;
            }


        }

        this.heading = gradient;
        this.leftPath = leftPath;
        this.rightPath = rightPath;
    }

    /**
     * Returns the first column of a 2D array of doubles
     *
     * @param arr 2D array of doubles
     * @return array of doubles representing the 1st column of the initial parameter
     */

    public static double[] getXVector(double[][] arr) {
        double[] temp = new double[arr.length];

        for (int i = 0; i < temp.length; i++)
            temp[i] = arr[i][0];

        return temp;
    }

    /**
     * Returns the second column of a 2D array of doubles
     *
     * @param arr 2D array of doubles
     * @return array of doubles representing the 1st column of the initial parameter
     */
    public static double[] getYVector(double[][] arr) {
        double[] temp = new double[arr.length];

        for (int i = 0; i < temp.length; i++)
            temp[i] = arr[i][1];

        return temp;
    }



    public static double[][] transposeVector(double[][] arr) {
        double[][] temp = new double[arr[0].length][arr.length];

        for (int i = 0; i < temp.length; i++)
            for (int j = 0; j < temp[i].length; j++)
                temp[i][j] = arr[j][i];

        return temp;
    }

    public void setPathAlpha(double alpha) {
        pathAlpha = alpha;
    }

    public void setPathBeta(double beta) {
        pathAlpha = beta;
    }

    public void setPathTolerance(double tolerance) {
        pathAlpha = tolerance;
    }

    /**
     * This code will calculate a smooth path based on the program parameters. If the user doesn't set any parameters, the will use the defaults optimized for most cases. The results will be saved into the corresponding
     * class members. The user can then access .smoothPath, .leftPath, .rightPath, .smoothCenterVelocity, .smoothRightVelocity, .smoothLeftVelocity as needed.
     * <p>
     * After calling this method, the user only needs to pass .smoothRightVelocity[1], .smoothLeftVelocity[1] to the corresponding speed controllers on the Robot, and step through each setPoint.
     *
     * @param totalTime       - time the user wishes to complete the path in seconds. (this is the maximum amount of time the robot is allowed to take to traverse the path.)
     * @param timeStep        - the frequency at which the robot controller is running on the robot.
     * @param robotTrackWidth - distance between left and right side wheels of a skid steer chassis. Known as the track width.
     */
    public void calculate(double totalTime, double timeStep, double robotTrackWidth) {
        /**
         * pseudo code
         *
         * 1. Reduce input waypoints to only essential (direction changing) node points
         * 2. Calculate how many total datapoints we need to satisfy the controller for "playback"
         * 3. Simultaneously inject and smooth the path until we end up with a smooth path with required number
         *    of datapoints, and which follows the waypoint path.
         * 4. Calculate left and right wheel paths by calculating parallel points at each datapoint
         */



        //first find only direction changing nodes
        nodeOnlyPath = nodeOnlyWayPoints(origPath);

        //Figure out how many nodes to inject
        int[] inject = injectionCounter2Steps(nodeOnlyPath.length, totalTime, timeStep);

        //iteratively inject and smooth the path
        for (int i = 0; i < inject.length; i++) {
            if (i == 0) {
                smoothPath = inject(nodeOnlyPath, inject[0]);
                smoothPath = smoother(smoothPath, pathAlpha, pathBeta, pathTolerance);
            } else {
                smoothPath = inject(smoothPath, inject[i]);
                smoothPath = smoother(smoothPath, 0.1, 0.3, 0.0000001);
            }
        }

        //calculate left and right path based on center path
        leftRight(smoothPath, robotTrackWidth);

        origCenterVelocity = velocity(smoothPath, timeStep);
        origLeftVelocity = velocity(leftPath, timeStep);
        origRightVelocity = velocity(rightPath, timeStep);

        //copy smooth velocities into fix Velocities
        smoothCenterVelocity = doubleArrayCopy(origCenterVelocity);
        smoothLeftVelocity = doubleArrayCopy(origLeftVelocity);
        smoothRightVelocity = doubleArrayCopy(origRightVelocity);

        //set final vel to zero
        smoothCenterVelocity[smoothCenterVelocity.length - 1][1] = 0.0;
        smoothLeftVelocity[smoothLeftVelocity.length - 1][1] = 0.0;
        smoothRightVelocity[smoothRightVelocity.length - 1][1] = 0.0;

        //Smooth velocity with zero final V
        smoothCenterVelocity = smoother(smoothCenterVelocity, velocityAlpha, velocityBeta, velocityTolerance);
        smoothLeftVelocity = smoother(smoothLeftVelocity, velocityAlpha, velocityBeta, velocityTolerance);
        smoothRightVelocity = smoother(smoothRightVelocity, velocityAlpha, velocityBeta, velocityTolerance);

        //fix velocity distance error
        smoothCenterVelocity = velocityFix(smoothCenterVelocity, origCenterVelocity, 0.0000001);
        smoothLeftVelocity = velocityFix(smoothLeftVelocity, origLeftVelocity, 0.0000001);
        smoothRightVelocity = velocityFix(smoothRightVelocity, origRightVelocity, 0.0000001);
    }




    //main program
    public static void main(String[] args) {
        DriveStraight();

    }


    public static void BluePathLeftToLeft() {


            //Lets create a bank image
            LinePlot fig3 = new LinePlot(new double[][]{{0.0, 0.0}});
            fig3.yGridOn();
            fig3.xGridOn();
            fig3.setYLabel("Y (feet)");
            fig3.setXLabel("X (feet)");
            fig3.setTitle("Top Down View of FRC Field (27ft x 54ft) \n shows global position of robot path, along with left and right wheel trajectories");


            double fieldWidth = 27.0;
            fig3.setXTic(0, 54, 1);
            fig3.setYTic(0, fieldWidth, 1);

            double[][] BluePortalLine1 = new double[][]{{0, 24.5}, {2.298, 27}};
            fig3.addData(BluePortalLine1, Color.blue);

            double[][] BluePortalLine2 = new double[][]{{2.298, 0}, {0, 2.298}};
            fig3.addData(BluePortalLine2, Color.blue);

            double[][] RedPortalLine1 = new double[][]{{54 - 2.298, 27}, {54, 24.5}};
            fig3.addData(RedPortalLine1, Color.red);

            double[][] RedPortalLine2 = new double[][]{{54 - 2.298, 0}, {54, 2.5}};
            fig3.addData(RedPortalLine2, Color.red);

            double[][] RedSwitch = new double[][]{{11.6666, 7.1042}, {11.6666, 27 - 7.1042}, {16.3332, 27 - 7.1042}, {16.3332, 7.1042}, {11.6666, 7.1042}};
            fig3.addData(RedSwitch, Color.red);

            double[][] BlueSwitch = new double[][]{{54 - 11.6666, 7.1042}, {54 - 11.6666, 27 - 7.1042}, {54 - 16.3332, 27 - 7.1042}, {54 - 16.3332, 7.1042}, {54 - 11.6666, 7.1042}};
            fig3.addData(BlueSwitch, Color.blue);

            double[][] RedCube = new double[][]{{8.1666, 11.625}, {8.1666, 11.625 + 3.75}, {11.6666, 11.625 + 3.75}, {11.6666, 11.625}, {8.1666, 11.625}};
            fig3.addData(RedCube, Color.red);

            double[][] BlueCube = new double[][]{{54 - 8.1666, 11.625}, {54 - 8.1666, 11.625 + 3.75}, {42.3334, 11.625 + 3.75}, {42.3334, 11.625}, {54 - 8.1666, 11.625}};
            fig3.addData(BlueCube, Color.blue);

            double[][] RedPlatform = new double[][] {{21.458396, 7.9375}, {21.458396, 7.9375 + 11.125}, {26.3124, 7.9375 + 11.125}, {26.3124, 7.9375}, {21.458396, 7.9375}};
            fig3.addData(RedPlatform, Color.red);
            
            double[][] BluePlatform = new double[][] {{32.541604, 7.9375}, {32.541604, 7.9375 + 11.125}, {27.6876, 7.9375 + 11.125}, {27.6876, 7.9375}, {32.541604, 7.9375}};
            fig3.addData(BluePlatform, Color.blue);
            
            double[][] Scale1 = new double[][] {{24.992, 18}, {24.992, 21}, {24.992 + 4.016, 21}, {24.992 + 4.016, 18}, {24.992, 18}};
            fig3.addData(Scale1,  Color.black);
            
            double[][] Scale2 = new double[][] {{24.992, 9}, {24.992, 6}, {24.992 + 4.016, 6}, {24.992 + 4.016, 9}, {24.992, 9}};
            fig3.addData(Scale2, Color.black);
            
            double[][] Path = new double[][]{
                    {0, 21}, {7, 21}, {10, 24}, {13, 24}, {14, 23}, {14, 20}
            };

            long start = System.currentTimeMillis();

            double totalTime = 10; //seconds
            double timeStep = 0.02; //period of control loop on Rio, seconds
            double robotTrackWidth = 2; //distance between left and right wheels, feet

            final PathPlanner path = new PathPlanner(Path);
            path.calculate(totalTime, timeStep, robotTrackWidth);

            System.out.println("Time in ms: " + (System.currentTimeMillis() - start));

            //waypoint path
            fig3.addData(path.nodeOnlyPath, Color.blue, Color.green);

            //add all other paths
            fig3.addData(path.smoothPath, Color.red, Color.blue);
            fig3.addData(path.leftPath, Color.magenta);
            fig3.addData(path.rightPath, Color.magenta);


            //Velocity
            LinePlot fig4 = new LinePlot(path.smoothCenterVelocity, null, Color.blue);
            fig4.yGridOn();
            fig4.xGridOn();
            fig4.setYLabel("Velocity (ft/sec)");
            fig4.setXLabel("time (seconds)");
            fig4.setTitle("Velocity Profile for Left and Right Wheels \n Left = Cyan, Right = Magenta");
            fig4.addData(path.smoothRightVelocity, Color.magenta);
            fig4.addData(path.smoothLeftVelocity, Color.cyan);


            double[] LeftWheelsLeftToLeftBlue = getYVector(path.smoothLeftVelocity);
            double[] RightWheelsLeftToLeftBlue = getYVector(path.smoothRightVelocity);

            try {
                PrintWriter pr = new PrintWriter("LeftWheels-LeftToLeft-Blue");

                for (int i = 0; i < LeftWheelsLeftToLeftBlue.length; i++) {
                    pr.println(LeftWheelsLeftToLeftBlue[i] + ", ");
                }
                pr.close();
            } catch (Exception e) {
                e.printStackTrace();
                System.out.println("No such file exists.");
            }

            try {
                PrintWriter pr = new PrintWriter("RightWheels-LeftToLeft-Blue");

                for (int i = 0; i < RightWheelsLeftToLeftBlue.length; i++) {
                    pr.println(RightWheelsLeftToLeftBlue[i] + ", ");
                }
                pr.close();
            } catch (Exception e) {
                e.printStackTrace();
                System.out.println("No such file exists.");
            }

        }


        public static void BluePathLeftToRight() {

            //Lets create a bank image
            LinePlot fig3 = new LinePlot(new double[][]{{0.0, 0.0}});
            fig3.yGridOn();
            fig3.xGridOn();
            fig3.setYLabel("Y (feet)");
            fig3.setXLabel("X (feet)");
            fig3.setTitle("Top Down View of FRC Field (27ft x 54ft) \n shows global position of robot path, along with left and right wheel trajectories");


            double fieldWidth = 27.0;
            fig3.setXTic(0, 54, 1);
            fig3.setYTic(0, fieldWidth, 1);

            double[][] BluePortalLine1 = new double[][]{{0, 24.5}, {2.298, 27}};
            fig3.addData(BluePortalLine1, Color.blue);

            double[][] BluePortalLine2 = new double[][]{{2.298, 0}, {0, 2.298}};
            fig3.addData(BluePortalLine2, Color.blue);

            double[][] RedPortalLine1 = new double[][]{{54 - 2.298, 27}, {54, 24.5}};
            fig3.addData(RedPortalLine1, Color.red);

            double[][] RedPortalLine2 = new double[][]{{54 - 2.298, 0}, {54, 2.5}};
            fig3.addData(RedPortalLine2, Color.red);

            double[][] RedSwitch = new double[][]{{11.6666, 7.1042}, {11.6666, 27 - 7.1042}, {16.3332, 27 - 7.1042}, {16.3332, 7.1042}, {11.6666, 7.1042}};
            fig3.addData(RedSwitch, Color.red);

            double[][] BlueSwitch = new double[][]{{54 - 11.6666, 7.1042}, {54 - 11.6666, 27 - 7.1042}, {54 - 16.3332, 27 - 7.1042}, {54 - 16.3332, 7.1042}, {54 - 11.6666, 7.10}};
            fig3.addData(BlueSwitch, Color.blue);

            double[][] RedCube = new double[][]{{8.1666, 11.625}, {8.1666, 11.625 + 3.75}, {11.6666, 11.625 + 3.75}, {11.6666, 11.625}, {8.1666, 11.625}};
            fig3.addData(RedCube, Color.red);

            double[][] BlueCube = new double[][]{{54 - 8.1666, 11.625}, {54 - 8.1666, 11.625 + 3.75}, {42.3334, 11.625 + 3.75}, {42.3334, 11.625}, {54 - 8.1666, 11.625}};
            fig3.addData(BlueCube, Color.blue);
            
            double[][] RedPlatform = new double[][] {{21.458396, 7.9375}, {21.458396, 7.9375 + 11.125}, {26.3124, 7.9375 + 11.125}, {26.3124, 7.9375}, {21.458396, 7.9375}};
            fig3.addData(RedPlatform, Color.red);
            
            double[][] BluePlatform = new double[][] {{32.541604, 7.9375}, {32.541604, 7.9375 + 11.125}, {27.6876, 7.9375 + 11.125}, {27.6876, 7.9375}, {32.541604, 7.9375}};
            fig3.addData(BluePlatform, Color.blue);
            
            double[][] Scale1 = new double[][] {{24.992, 18}, {24.992, 21}, {24.992 + 4.016, 21}, {24.992 + 4.016, 18}, {24.992, 18}};
            fig3.addData(Scale1,  Color.black);
            
            double[][] Scale2 = new double[][] {{24.992, 9}, {24.992, 6}, {24.992 + 4.016, 6}, {24.992 + 4.016, 9}, {24.992, 9}};
            fig3.addData(Scale2, Color.black);


            double[][] Path = new double[][]{
                    {0, 21}, {8, 21}, {12, 24}, {19, 24}, {19, 22}, {19, 3}, {15, 3}, {14, 4}, {14, 7}
            };

            long start = System.currentTimeMillis();

            double totalTime = 10; //seconds
            double timeStep = 0.02; //period of control loop on Rio, seconds
            double robotTrackWidth = 2; //distance between left and right wheels, feet

            final PathPlanner path = new PathPlanner(Path);
            path.calculate(totalTime, timeStep, robotTrackWidth);

            System.out.println("Time in ms: " + (System.currentTimeMillis() - start));

            //waypoint path
            fig3.addData(path.nodeOnlyPath, Color.blue, Color.green);

            //add all other paths
            fig3.addData(path.smoothPath, Color.red, Color.blue);
            fig3.addData(path.leftPath, Color.magenta);
            fig3.addData(path.rightPath, Color.magenta);


            //Velocity
            LinePlot fig4 = new LinePlot(path.smoothCenterVelocity, null, Color.blue);
            fig4.yGridOn();
            fig4.xGridOn();
            fig4.setYLabel("Velocity (ft/sec)");
            fig4.setXLabel("time (seconds)");
            fig4.setTitle("Velocity Profile for Left and Right Wheels \n Left = Cyan, Right = Magenta");
            fig4.addData(path.smoothRightVelocity, Color.magenta);
            fig4.addData(path.smoothLeftVelocity, Color.cyan);


            double[] LeftWheelsLeftToRightBlue = getYVector(path.smoothLeftVelocity);
            double[] RightWheelsLeftToRightBlue = getYVector(path.smoothRightVelocity);

            try {
                PrintWriter pr = new PrintWriter("LeftWheels-LeftToRight-Blue");

                for (int i = 0; i < LeftWheelsLeftToRightBlue.length; i++) {
                    pr.println(LeftWheelsLeftToRightBlue[i] + ", ");
                }
                pr.close();
            } catch (Exception e) {
                e.printStackTrace();
                System.out.println("No such file exists.");
            }

            try {
                PrintWriter pr = new PrintWriter("RightWheels-LeftToRight-Blue");

                for (int i = 0; i < RightWheelsLeftToRightBlue.length; i++) {
                    pr.println(RightWheelsLeftToRightBlue[i] + ", ");
                }
                pr.close();
            } catch (Exception e) {
                e.printStackTrace();
                System.out.println("No such file exists.");
            }
        }
        public static void BluePathMiddleToLeft(){
         //Lets create a bank image
         LinePlot fig3 = new LinePlot(new double[][]{{0.0, 0.0}});
         fig3.yGridOn();
         fig3.xGridOn();
         fig3.setYLabel("Y (feet)");
         fig3.setXLabel("X (feet)");
         fig3.setTitle("Top Down View of FRC Field (27ft x 54ft) \n shows global position of robot path, along with left and right wheel trajectories");


         double fieldWidth = 27.0;
         fig3.setXTic(0, 54, 1);
         fig3.setYTic(0, fieldWidth, 1);

         double[][] BluePortalLine1 = new double[][]{{0, 24.5}, {2.298, 27}};
         fig3.addData(BluePortalLine1, Color.blue);

         double[][] BluePortalLine2 = new double[][]{{2.298, 0}, {0, 2.298}};
         fig3.addData(BluePortalLine2, Color.blue);

         double[][] RedPortalLine1 = new double[][]{{54 - 2.298, 27}, {54, 24.5}};
         fig3.addData(RedPortalLine1, Color.red);

         double[][] RedPortalLine2 = new double[][]{{54 - 2.298, 0}, {54, 2.5}};
         fig3.addData(RedPortalLine2, Color.red);

         double[][] RedSwitch = new double[][]{{11.6666, 7.1042}, {11.6666, 27 - 7.1042}, {16.3332, 27 - 7.1042}, {16.3332, 7.1042}, {11.6666, 7.1042}};
         fig3.addData(RedSwitch, Color.red);

         double[][] BlueSwitch = new double[][]{{54 - 11.6666, 7.1042}, {54 - 11.6666, 27 - 7.1042}, {54 - 16.3332, 27 - 7.1042}, {54 - 16.3332, 7.1042}, {54 - 11.6666, 7.10}};
         fig3.addData(BlueSwitch, Color.blue);

         double[][] RedCube = new double[][]{{8.1666, 11.625}, {8.1666, 11.625 + 3.75}, {11.6666, 11.625 + 3.75}, {11.6666, 11.625}, {8.1666, 11.625}};
         fig3.addData(RedCube, Color.red);

         double[][] BlueCube = new double[][]{{54 - 8.1666, 11.625}, {54 - 8.1666, 11.625 + 3.75}, {42.3334, 11.625 + 3.75}, {42.3334, 11.625}, {54 - 8.1666, 11.625}};
         fig3.addData(BlueCube, Color.blue);
         
         double[][] RedPlatform = new double[][] {{21.458396, 7.9375}, {21.458396, 7.9375 + 11.125}, {26.3124, 7.9375 + 11.125}, {26.3124, 7.9375}, {21.458396, 7.9375}};
         fig3.addData(RedPlatform, Color.red);
         
         double[][] BluePlatform = new double[][] {{32.541604, 7.9375}, {32.541604, 7.9375 + 11.125}, {27.6876, 7.9375 + 11.125}, {27.6876, 7.9375}, {32.541604, 7.9375}};
         fig3.addData(BluePlatform, Color.blue);
         
         double[][] Scale1 = new double[][] {{24.992, 18}, {24.992, 21}, {24.992 + 4.016, 21}, {24.992 + 4.016, 18}, {24.992, 18}};
         fig3.addData(Scale1,  Color.black);
         
         double[][] Scale2 = new double[][] {{24.992, 9}, {24.992, 6}, {24.992 + 4.016, 6}, {24.992 + 4.016, 9}, {24.992, 9}};
         fig3.addData(Scale2, Color.black);


         double[][] Path = new double[][]{
                 {0, 14}, {5, 14}, {9 , 25}, {11, 25}, {13, 23}, {14, 22}, {14, 20}
         };

         long start = System.currentTimeMillis();

         double totalTime = 10; //seconds
         double timeStep = 0.02; //period of control loop on Rio, seconds                                                                                                                }
         double robotTrackWidth = 2; //distance between left and right wheels, feet

          final PathPlanner path = new PathPlanner(Path);
          path.calculate(totalTime, timeStep, robotTrackWidth);

          System.out.println("Time in ms: " + (System.currentTimeMillis() - start));

          //waypoint path
          fig3.addData(path.nodeOnlyPath, Color.blue, Color.green);

          //add all other paths
          fig3.addData(path.smoothPath, Color.red, Color.blue);
          fig3.addData(path.leftPath, Color.magenta);
          fig3.addData(path.rightPath, Color.magenta);


          //Velocity
          LinePlot fig4 = new LinePlot(path.smoothCenterVelocity, null, Color.blue);
          fig4.yGridOn();
          fig4.xGridOn();
          fig4.setYLabel("Velocity (ft/sec)");
          fig4.setXLabel("time (seconds)");
          fig4.setTitle("Velocity Profile for Left and Right Wheels \n Left = Cyan, Right = Magenta");
          fig4.addData(path.smoothRightVelocity, Color.magenta);
          fig4.addData(path.smoothLeftVelocity, Color.cyan);


          double[] LeftWheelsMiddleToRightBlue = getYVector(path.smoothLeftVelocity);
          double[] RightWheelsMiddleToRightBlue = getYVector(path.smoothRightVelocity);

          try {
              PrintWriter pr = new PrintWriter("LeftWheels-MiddleToLeft-Blue");

              for (int i = 0; i < LeftWheelsMiddleToRightBlue.length; i++) {
                  pr.println(LeftWheelsMiddleToRightBlue[i] + ", ");
              }
              pr.close();
          } catch (Exception e) {
              e.printStackTrace();
              System.out.println("No such file exists.");
          }

          try {
              PrintWriter pr = new PrintWriter("RightWheels-MiddleToLeft-Blue");

              for (int i = 0; i < RightWheelsMiddleToRightBlue.length; i++) {
                  pr.println(RightWheelsMiddleToRightBlue[i] + ", ");
              }
              pr.close();
          } catch (Exception e) {
              e.printStackTrace();
              System.out.println("No such file exists.");
          }
    }
    public static void BluePathMiddleToRight(){
     //Lets create a bank image
     LinePlot fig3 = new LinePlot(new double[][]{{0.0, 0.0}});
     fig3.yGridOn();
     fig3.xGridOn();
     fig3.setYLabel("Y (feet)");
     fig3.setXLabel("X (feet)");
     fig3.setTitle("Top Down View of FRC Field (27ft x 54ft) \n shows global position of robot path, along with left and right wheel trajectories");


     double fieldWidth = 27.0;
     fig3.setXTic(0, 54, 1);
     fig3.setYTic(0, fieldWidth, 1);

     double[][] BluePortalLine1 = new double[][]{{0, 24.5}, {2.298, 27}};
     fig3.addData(BluePortalLine1, Color.blue);

     double[][] BluePortalLine2 = new double[][]{{2.298, 0}, {0, 2.298}};
     fig3.addData(BluePortalLine2, Color.blue);

     double[][] RedPortalLine1 = new double[][]{{54 - 2.298, 27}, {54, 24.5}};
     fig3.addData(RedPortalLine1, Color.red);

     double[][] RedPortalLine2 = new double[][]{{54 - 2.298, 0}, {54, 2.5}};
     fig3.addData(RedPortalLine2, Color.red);

     double[][] RedSwitch = new double[][]{{11.6666, 7.1042}, {11.6666, 27 - 7.1042}, {16.3332, 27 - 7.1042}, {16.3332, 7.1042}, {11.6666, 7.1042}};
     fig3.addData(RedSwitch, Color.red);

     double[][] BlueSwitch = new double[][]{{54 - 11.6666, 7.1042}, {54 - 11.6666, 27 - 7.1042}, {54 - 16.3332, 27 - 7.1042}, {54 - 16.3332, 7.1042}, {54 - 11.6666, 7.10}};
     fig3.addData(BlueSwitch, Color.blue);

     double[][] RedCube = new double[][]{{8.1666, 11.625}, {8.1666, 11.625 + 3.75}, {11.6666, 11.625 + 3.75}, {11.6666, 11.625}, {8.1666, 11.625}};
     fig3.addData(RedCube, Color.red);

     double[][] BlueCube = new double[][]{{54 - 8.1666, 11.625}, {54 - 8.1666, 11.625 + 3.75}, {42.3334, 11.625 + 3.75}, {42.3334, 11.625}, {54 - 8.1666, 11.625}};
     fig3.addData(BlueCube, Color.blue);
     
     double[][] RedPlatform = new double[][] {{21.458396, 7.9375}, {21.458396, 7.9375 + 11.125}, {26.3124, 7.9375 + 11.125}, {26.3124, 7.9375}, {21.458396, 7.9375}};
     fig3.addData(RedPlatform, Color.red);
     
     double[][] BluePlatform = new double[][] {{32.541604, 7.9375}, {32.541604, 7.9375 + 11.125}, {27.6876, 7.9375 + 11.125}, {27.6876, 7.9375}, {32.541604, 7.9375}};
     fig3.addData(BluePlatform, Color.blue);
     
     double[][] Scale1 = new double[][] {{24.992, 18}, {24.992, 21}, {24.992 + 4.016, 21}, {24.992 + 4.016, 18}, {24.992, 18}};
     fig3.addData(Scale1,  Color.black);
     
     double[][] Scale2 = new double[][] {{24.992, 9}, {24.992, 6}, {24.992 + 4.016, 6}, {24.992 + 4.016, 9}, {24.992, 9}};
     fig3.addData(Scale2, Color.black);


     double[][] Path = new double[][]{
             {0, 14}, {6, 14}, {6, 6}, {8, 3}, {12, 3}, {14, 5}, {14, 7}
     };

     long start = System.currentTimeMillis();

     double totalTime = 10; //seconds
     double timeStep = 0.02; //period of control loop on Rio, seconds                                                                                                                }
     double robotTrackWidth = 2; //distance between left and right wheels, feet

      final PathPlanner path = new PathPlanner(Path);
      path.calculate(totalTime, timeStep, robotTrackWidth);

      System.out.println("Time in ms: " + (System.currentTimeMillis() - start));

      //waypoint path
      fig3.addData(path.nodeOnlyPath, Color.blue, Color.green);

      //add all other paths
      fig3.addData(path.smoothPath, Color.red, Color.blue);
      fig3.addData(path.leftPath, Color.magenta);
      fig3.addData(path.rightPath, Color.magenta);


      //Velocity
      LinePlot fig4 = new LinePlot(path.smoothCenterVelocity, null, Color.blue);
      fig4.yGridOn();
      fig4.xGridOn();
      fig4.setYLabel("Velocity (ft/sec)");
      fig4.setXLabel("time (seconds)");
      fig4.setTitle("Velocity Profile for Left and Right Wheels \n Left = Cyan, Right = Magenta");
      fig4.addData(path.smoothRightVelocity, Color.magenta);
      fig4.addData(path.smoothLeftVelocity, Color.cyan);


      double[] LeftWheelsMiddleToRightBlue = getYVector(path.smoothLeftVelocity);
      double[] RightWheelsMiddleToRightBlue = getYVector(path.smoothRightVelocity);

      try {
          PrintWriter pr = new PrintWriter("LeftWheels-MiddleToRight-Blue");

          for (int i = 0; i < LeftWheelsMiddleToRightBlue.length; i++) {
              pr.println(LeftWheelsMiddleToRightBlue[i] + ", ");
          }
          pr.close();
      } catch (Exception e) {
          e.printStackTrace();
          System.out.println("No such file exists.");
      }

      try {
          PrintWriter pr = new PrintWriter("RightWheels-MiddleToRight-Blue");

          for (int i = 0; i < RightWheelsMiddleToRightBlue.length; i++) {
              pr.println(RightWheelsMiddleToRightBlue[i] + ", ");
          }
          pr.close();
      } catch (Exception e) {
          e.printStackTrace();
          System.out.println("No such file exists.");
      }
    }
    public static void BluePathRightToRight(){
        //Lets create a bank image
        LinePlot fig3 = new LinePlot(new double[][]{{0.0, 0.0}});
        fig3.yGridOn();
        fig3.xGridOn();
        fig3.setYLabel("Y (feet)");
        fig3.setXLabel("X (feet)");
        fig3.setTitle("Top Down View of FRC Field (27ft x 54ft) \n shows global position of robot path, along with left and right wheel trajectories");


        double fieldWidth = 27.0;
        fig3.setXTic(0, 54, 1);
        fig3.setYTic(0, fieldWidth, 1);

        double[][] BluePortalLine1 = new double[][]{{0, 24.5}, {2.298, 27}};
        fig3.addData(BluePortalLine1, Color.blue);

        double[][] BluePortalLine2 = new double[][]{{2.298, 0}, {0, 2.298}};
        fig3.addData(BluePortalLine2, Color.blue);

        double[][] RedPortalLine1 = new double[][]{{54 - 2.298, 27}, {54, 24.5}};
        fig3.addData(RedPortalLine1, Color.red);

        double[][] RedPortalLine2 = new double[][]{{54 - 2.298, 0}, {54, 2.5}};
        fig3.addData(RedPortalLine2, Color.red);

        double[][] RedSwitch = new double[][]{{11.6666, 7.1042}, {11.6666, 27 - 7.1042}, {16.3332, 27 - 7.1042}, {16.3332, 7.1042}, {11.6666, 7.1042}};
        fig3.addData(RedSwitch, Color.red);

        double[][] BlueSwitch = new double[][]{{54 - 11.6666, 7.1042}, {54 - 11.6666, 27 - 7.1042}, {54 - 16.3332, 27 - 7.1042}, {54 - 16.3332, 7.1042}, {54 - 11.6666, 7.10}};
        fig3.addData(BlueSwitch, Color.blue);

        double[][] RedCube = new double[][]{{8.1666, 11.625}, {8.1666, 11.625 + 3.75}, {11.6666, 11.625 + 3.75}, {11.6666, 11.625}, {8.1666, 11.625}};
        fig3.addData(RedCube, Color.red);

        double[][] BlueCube = new double[][]{{54 - 8.1666, 11.625}, {54 - 8.1666, 11.625 + 3.75}, {42.3334, 11.625 + 3.75}, {42.3334, 11.625}, {54 - 8.1666, 11.625}};
        fig3.addData(BlueCube, Color.blue);
        
        double[][] RedPlatform = new double[][] {{21.458396, 7.9375}, {21.458396, 7.9375 + 11.125}, {26.3124, 7.9375 + 11.125}, {26.3124, 7.9375}, {21.458396, 7.9375}};
        fig3.addData(RedPlatform, Color.red);
        
        double[][] BluePlatform = new double[][] {{32.541604, 7.9375}, {32.541604, 7.9375 + 11.125}, {27.6876, 7.9375 + 11.125}, {27.6876, 7.9375}, {32.541604, 7.9375}};
        fig3.addData(BluePlatform, Color.blue);
        
        double[][] Scale1 = new double[][] {{24.992, 18}, {24.992, 21}, {24.992 + 4.016, 21}, {24.992 + 4.016, 18}, {24.992, 18}};
        fig3.addData(Scale1,  Color.black);
        
        double[][] Scale2 = new double[][] {{24.992, 9}, {24.992, 6}, {24.992 + 4.016, 6}, {24.992 + 4.016, 9}, {24.992, 9}};
        fig3.addData(Scale2, Color.black);


        double[][] Path = new double[][]{
                {0, 4}, {10, 3}, {12, 3}, {13.5, 4}, {14, 5}, {14, 7}
        };

        long start = System.currentTimeMillis();

        double totalTime = 10; //seconds
        double timeStep = 0.02; //period of control loop on Rio, seconds                                                                                                                }
        double robotTrackWidth = 2; //distance between left and right wheels, feet

        final PathPlanner path = new PathPlanner(Path);
        path.calculate(totalTime, timeStep, robotTrackWidth);

        System.out.println("Time in ms: " + (System.currentTimeMillis() - start));

        //waypoint path
        fig3.addData(path.nodeOnlyPath, Color.blue, Color.green);

        //add all other paths
        fig3.addData(path.smoothPath, Color.red, Color.blue);
        fig3.addData(path.leftPath, Color.magenta);
        fig3.addData(path.rightPath, Color.magenta);


        //Velocity
        LinePlot fig4 = new LinePlot(path.smoothCenterVelocity, null, Color.blue);
        fig4.yGridOn();
        fig4.xGridOn();
        fig4.setYLabel("Velocity (ft/sec)");
        fig4.setXLabel("time (seconds)");
        fig4.setTitle("Velocity Profile for Left and Right Wheels \n Left = Cyan, Right = Magenta");
        fig4.addData(path.smoothRightVelocity, Color.magenta);
        fig4.addData(path.smoothLeftVelocity, Color.cyan);


        double[] LeftWheelsRightToRightBlue = getYVector(path.smoothLeftVelocity);
        double[] RightWheelsRightToRightBlue = getYVector(path.smoothRightVelocity);

        try {
            PrintWriter pr = new PrintWriter("LeftWheels-RightToRight-Blue");

            for (int i = 0; i < LeftWheelsRightToRightBlue.length; i++) {
                pr.println(LeftWheelsRightToRightBlue[i] + ", ");
            }
            pr.close();
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("No such file exists.");
        }

        try {
            PrintWriter pr = new PrintWriter("RightWheels-RightToRight-Blue");

            for (int i = 0; i < RightWheelsRightToRightBlue.length; i++) {
                pr.println(RightWheelsRightToRightBlue[i] + ", ");
            }
            pr.close();
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("No such file exists.");
        }
    }
    public static void BluePathRightToLeft(){
        //Lets create a bank image
        LinePlot fig3 = new LinePlot(new double[][]{{0.0, 0.0}});
        fig3.yGridOn();
        fig3.xGridOn();
        fig3.setYLabel("Y (feet)");
        fig3.setXLabel("X (feet)");
        fig3.setTitle("Top Down View of FRC Field (27ft x 54ft) \n shows global position of robot path, along with left and right wheel trajectories");


        double fieldWidth = 27.0;
        fig3.setXTic(0, 54, 1);
        fig3.setYTic(0, fieldWidth, 1);

        double[][] BluePortalLine1 = new double[][]{{0, 24.5}, {2.298, 27}};
        fig3.addData(BluePortalLine1, Color.blue);

        double[][] BluePortalLine2 = new double[][]{{2.298, 0}, {0, 2.298}};
        fig3.addData(BluePortalLine2, Color.blue);

        double[][] RedPortalLine1 = new double[][]{{54 - 2.298, 27}, {54, 24.5}};
        fig3.addData(RedPortalLine1, Color.red);

        double[][] RedPortalLine2 = new double[][]{{54 - 2.298, 0}, {54, 2.5}};
        fig3.addData(RedPortalLine2, Color.red);

        double[][] RedSwitch = new double[][]{{11.6666, 7.1042}, {11.6666, 27 - 7.1042}, {16.3332, 27 - 7.1042}, {16.3332, 7.1042}, {11.6666, 7.1042}};
        fig3.addData(RedSwitch, Color.red);

        double[][] BlueSwitch = new double[][]{{54 - 11.6666, 7.1042}, {54 - 11.6666, 27 - 7.1042}, {54 - 16.3332, 27 - 7.1042}, {54 - 16.3332, 7.1042}, {54 - 11.6666, 7.10}};
        fig3.addData(BlueSwitch, Color.blue);

        double[][] RedCube = new double[][]{{8.1666, 11.625}, {8.1666, 11.625 + 3.75}, {11.6666, 11.625 + 3.75}, {11.6666, 11.625}, {8.1666, 11.625}};
        fig3.addData(RedCube, Color.red);

        double[][] BlueCube = new double[][]{{54 - 8.1666, 11.625}, {54 - 8.1666, 11.625 + 3.75}, {42.3334, 11.625 + 3.75}, {42.3334, 11.625}, {54 - 8.1666, 11.625}};
        fig3.addData(BlueCube, Color.blue);
        
        double[][] RedPlatform = new double[][] {{21.458396, 7.9375}, {21.458396, 7.9375 + 11.125}, {26.3124, 7.9375 + 11.125}, {26.3124, 7.9375}, {21.458396, 7.9375}};
        fig3.addData(RedPlatform, Color.red);
        
        double[][] BluePlatform = new double[][] {{32.541604, 7.9375}, {32.541604, 7.9375 + 11.125}, {27.6876, 7.9375 + 11.125}, {27.6876, 7.9375}, {32.541604, 7.9375}};
        fig3.addData(BluePlatform, Color.blue);
        
        double[][] Scale1 = new double[][] {{24.992, 18}, {24.992, 21}, {24.992 + 4.016, 21}, {24.992 + 4.016, 18}, {24.992, 18}};
        fig3.addData(Scale1,  Color.black);
        
        double[][] Scale2 = new double[][] {{24.992, 9}, {24.992, 6}, {24.992 + 4.016, 6}, {24.992 + 4.016, 9}, {24.992, 9}};
        fig3.addData(Scale2, Color.black);


        double[][] Path = new double[][]{
                {0, 4},  {6, 4}, {14, 4}, {20, 4}, {20, 23}, {18, 25}, {16, 25}, {14, 23}, {14, 20}
        };

        long start = System.currentTimeMillis();

        double totalTime = 10; //seconds
        double timeStep = 0.02; //period of control loop on Rio, seconds                                                                                                                }
        double robotTrackWidth = 2; //distance between left and right wheels, feet

        final PathPlanner path = new PathPlanner(Path);
        path.calculate(totalTime, timeStep, robotTrackWidth);

        System.out.println("Time in ms: " + (System.currentTimeMillis() - start));

        //waypoint path
        fig3.addData(path.nodeOnlyPath, Color.blue, Color.green);

        //add all other paths
        fig3.addData(path.smoothPath, Color.red, Color.blue);
        fig3.addData(path.leftPath, Color.magenta);
        fig3.addData(path.rightPath, Color.magenta);


        //Velocity
        LinePlot fig4 = new LinePlot(path.smoothCenterVelocity, null, Color.blue);
        fig4.yGridOn();
        fig4.xGridOn();
        fig4.setYLabel("Velocity (ft/sec)");
        fig4.setXLabel("time (seconds)");
        fig4.setTitle("Velocity Profile for Left and Right Wheels \n Left = Cyan, Right = Magenta");
        fig4.addData(path.smoothRightVelocity, Color.magenta);
        fig4.addData(path.smoothLeftVelocity, Color.cyan);


        double[] LeftWheelsRightToLeftBlue = getYVector(path.smoothLeftVelocity);
        double[] RightWheelsRightToLeftBlue = getYVector(path.smoothRightVelocity);

        try {
            PrintWriter pr = new PrintWriter("LeftWheels-RightToLeft-Blue");

            for (int i = 0; i < LeftWheelsRightToLeftBlue.length; i++) {
                pr.println(LeftWheelsRightToLeftBlue[i] + ", ");
            }
            pr.close();
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("No such file exists.");
        }

        try {
            PrintWriter pr = new PrintWriter("RightWheels-RightToLeft-Blue");

            for (int i = 0; i < RightWheelsRightToLeftBlue.length; i++) {
                pr.println(RightWheelsRightToLeftBlue[i] + ", ");
            }
            pr.close();
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("No such file exists.");
        }
    }

    public static void DriveStraight() {



        //Lets create a bank image
        LinePlot fig3 = new LinePlot(new double[][]{{0.0, 0.0}});
        fig3.yGridOn();
        fig3.xGridOn();
        fig3.setYLabel("Y (feet)");
        fig3.setXLabel("X (feet)");
        fig3.setTitle("Top Down View of FRC Field (27ft x 54ft) \n shows global position of robot path, along with left and right wheel trajectories");


        double fieldWidth = 27.0;
        fig3.setXTic(0, 54, 1);
        fig3.setYTic(0, fieldWidth, 1);

        double[][] BluePortalLine1 = new double[][]{{0, 24.5}, {2.298, 27}};
        fig3.addData(BluePortalLine1, Color.blue);

        double[][] BluePortalLine2 = new double[][]{{2.298, 0}, {0, 2.298}};
        fig3.addData(BluePortalLine2, Color.blue);

        double[][] RedPortalLine1 = new double[][]{{54 - 2.298, 27}, {54, 24.5}};
        fig3.addData(RedPortalLine1, Color.red);

        double[][] RedPortalLine2 = new double[][]{{54 - 2.298, 0}, {54, 2.5}};
        fig3.addData(RedPortalLine2, Color.red);

        double[][] RedSwitch = new double[][]{{11.6666, 7.1042}, {11.6666, 27 - 7.1042}, {16.3332, 27 - 7.1042}, {16.3332, 7.1042}, {11.6666, 7.1042}};
        fig3.addData(RedSwitch, Color.red);

        double[][] BlueSwitch = new double[][]{{54 - 11.6666, 7.1042}, {54 - 11.6666, 27 - 7.1042}, {54 - 16.3332, 27 - 7.1042}, {54 - 16.3332, 7.1042}, {54 - 11.6666, 7.1042}};
        fig3.addData(BlueSwitch, Color.blue);

        double[][] RedCube = new double[][]{{8.1666, 11.625}, {8.1666, 11.625 + 3.75}, {11.6666, 11.625 + 3.75}, {11.6666, 11.625}, {8.1666, 11.625}};
        fig3.addData(RedCube, Color.red);

        double[][] BlueCube = new double[][]{{54 - 8.1666, 11.625}, {54 - 8.1666, 11.625 + 3.75}, {42.3334, 11.625 + 3.75}, {42.3334, 11.625}, {54 - 8.1666, 11.625}};
        fig3.addData(BlueCube, Color.blue);

        double[][] RedPlatform = new double[][] {{21.458396, 7.9375}, {21.458396, 7.9375 + 11.125}, {26.3124, 7.9375 + 11.125}, {26.3124, 7.9375}, {21.458396, 7.9375}};
        fig3.addData(RedPlatform, Color.red);
        
        double[][] BluePlatform = new double[][] {{32.541604, 7.9375}, {32.541604, 7.9375 + 11.125}, {27.6876, 7.9375 + 11.125}, {27.6876, 7.9375}, {32.541604, 7.9375}};
        fig3.addData(BluePlatform, Color.blue);
        
        double[][] Scale1 = new double[][] {{24.992, 18}, {24.992, 21}, {24.992 + 4.016, 21}, {24.992 + 4.016, 18}, {24.992, 18}};
        fig3.addData(Scale1,  Color.black);
        
        double[][] Scale2 = new double[][] {{24.992, 9}, {24.992, 6}, {24.992 + 4.016, 6}, {24.992 + 4.016, 9}, {24.992, 9}};
        fig3.addData(Scale2, Color.black);

        double[][] Path = new double[][]{
                {0, 21}, {14, 21}
        };

        long start = System.currentTimeMillis();

        double totalTime = 5; //seconds
        double timeStep = 0.02; //period of control loop on Rio, seconds
        double robotTrackWidth = 2; //distance between left and right wheels, feet

        final PathPlanner path = new PathPlanner(Path);
        path.calculate(totalTime, timeStep, robotTrackWidth);

        System.out.println("Time in ms: " + (System.currentTimeMillis() - start));

        //waypoint path
        fig3.addData(path.nodeOnlyPath, Color.blue, Color.green);

        //add all other paths
        fig3.addData(path.smoothPath, Color.red, Color.blue);
        fig3.addData(path.leftPath, Color.magenta);
        fig3.addData(path.rightPath, Color.magenta);


        //Velocity
        LinePlot fig4 = new LinePlot(path.smoothCenterVelocity, null, Color.blue);
        fig4.yGridOn();
        fig4.xGridOn();
        fig4.setYLabel("Velocity (ft/sec)");
        fig4.setXLabel("time (seconds)");
        fig4.setTitle("Velocity Profile for Left and Right Wheels \n Left = Cyan, Right = Magenta");
        fig4.addData(path.smoothRightVelocity, Color.magenta);
        fig4.addData(path.smoothLeftVelocity, Color.cyan);


        double[] LeftWheelsDriveStraight = getYVector(path.smoothLeftVelocity);
        double[] RightWheelsDriveStraight = getYVector(path.smoothRightVelocity);

        try {
            PrintWriter pr = new PrintWriter("LeftWheels-Straight-Blue");

            for (int i = 0; i < LeftWheelsDriveStraight.length; i++) {
                pr.println(LeftWheelsDriveStraight[i] + ", ");
            }
            pr.close();
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("No such file exists.");
        }

        try {
            PrintWriter pr = new PrintWriter("RightWheels-Straight-Blue");

            for (int i = 0; i < RightWheelsDriveStraight.length; i++) {
                pr.println(RightWheelsDriveStraight[i] + ", ");
            }
            pr.close();
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("No such file exists.");
        }

    }
}