package org.usfirst.frc.team6035.robot.util;

import java.util.ArrayList;
import java.util.List;

public class Track {

    private double tick;

    private ArrayList<Double> leftSpeeds;
    private ArrayList<Double> rightSpeeds;

    private ArrayList<Point<Double>> path;

    public Track(double tick,
                 final List<Double> leftSpeeds,
                 final List<Double> rightSpeeds,
                 final List<Point<Double>> path) {
        assert(leftSpeeds.size() == rightSpeeds.size());

        this.tick = tick;
        this.leftSpeeds = new ArrayList<>(leftSpeeds);
        this.rightSpeeds = new ArrayList<>(rightSpeeds);

        this.path = new ArrayList<>(path);
    }

    public double[][] leftTrackDataset() {
        return trackSpeedDataset(leftSpeeds);
    }

    public double[][] rightTrackDataset() {
        return trackSpeedDataset(rightSpeeds);
    }

    private double[][] trackSpeedDataset(ArrayList<Double> speeds) {
        final int dataSize = speeds.size();
        final double[][] data = new double[2][dataSize];

        double t = 0.0;
        for (int i = 0; i != dataSize; i++) {
            data[0][i] = t;
            data[1][i] = speeds.get(i);
            t = t + tick;
        }

        return data;
    }

    public double[][] pathDataset() {
        final int dataSize = path.size();
        final double[][] data = new double[2][dataSize];

        for (int i = 0; i != dataSize; i++) {
            final Point<Double> point = path.get(i);
            data[0][i] = point.x;
            data[1][i] = point.y;
        }

        return data;
    }

}
