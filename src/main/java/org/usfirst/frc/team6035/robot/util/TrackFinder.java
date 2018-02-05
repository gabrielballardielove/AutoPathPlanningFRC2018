package org.usfirst.frc.team6035.robot.util;

import org.usfirst.frc.team2168.robot.FalconPathPlanner;

import java.util.Arrays;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class TrackFinder {

    public static Track findTrack(double[][] wayPoints) {
        final FalconPathPlanner plan = getOptimisedPath(wayPoints);

        final Stream<Double> leftSpeeds = Arrays.stream(plan.smoothLeftVelocity).map(ts -> ts[1]);
        final Stream<Double> rightSpeeds = Arrays.stream(plan.smoothRightVelocity).map(ts -> ts[1]);

        final Stream<Point<Double>> path = Arrays.stream(plan.origPath).map(xy -> new Point<>(xy[0], xy[1]));

        return new Track(Config.TIME_STEP_S,
                leftSpeeds.collect(Collectors.toList()),
                rightSpeeds.collect(Collectors.toList()),
                path.collect(Collectors.toList()));
    }

    private static FalconPathPlanner getOptimisedPath(double[][] wayPoints) {
        final int initialTime = 10;

        final FalconPathPlanner path = new FalconPathPlanner(wayPoints);
        path.calculate(initialTime, Config.TIME_STEP_S, Config.ROBOT_TRACK_WIDTH_FT);

        final double maxSpeed = getMaxSpeed(path);
        final double factor = Config.MAX_SPEED_FT_PER_S / maxSpeed;

        final FalconPathPlanner path2 = new FalconPathPlanner(wayPoints);
        path2.calculate(initialTime / factor, Config.TIME_STEP_S, Config.ROBOT_TRACK_WIDTH_FT);

        return path2;
    }

    private static double getMaxSpeed(final FalconPathPlanner path) {
        double maxSpeed = -Double.MAX_VALUE;
        int nSteps = path.smoothLeftVelocity.length;

        for (int i = 0; i != nSteps; i++) {
            maxSpeed = Math.max(maxSpeed, path.smoothLeftVelocity[i][1]);
            maxSpeed = Math.max(maxSpeed, path.smoothRightVelocity[i][1]);
        }

        return maxSpeed;
    }

}
