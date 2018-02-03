package org.usfirst.frc.team6035.robot.util;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.JFreeChart;
import org.jfree.data.xy.DefaultXYDataset;
import org.jfree.data.xy.XYDataset;

import javax.swing.*;
import java.awt.*;

public class Plotter {

    public static void plotPath(String title, Track track) {
        plotTrackSpeeds(title, track);
        plotPaths(title, track);
    }

    private static void plotTrackSpeeds(final String title, final Track speeds) {
        final XYDataset dataset = getTrackSpeedsDataset(speeds);
        final JFreeChart chart = ChartFactory.createXYLineChart("Track speed over time - " + title, "Time (ms)", "Speed (ft/s)", dataset);
        showChartInWindow(chart);
    }

    private static void showChartInWindow(final JFreeChart chart) {
        final Image image = chart.createBufferedImage(640, 480);
        final ImageIcon icon = new ImageIcon(image);
        final JLabel label = new JLabel(icon);
        final JFrame frame = new JFrame(chart.getTitle().toString());
        frame.getContentPane().add(label);
        frame.pack();
        frame.setVisible(true);
    }

    private static XYDataset getTrackSpeedsDataset(Track track) {
        final DefaultXYDataset dataset = new DefaultXYDataset();
        dataset.addSeries("Left Track", track.leftTrackDataset());
        dataset.addSeries("Right Track", track.rightTrackDataset());
        return dataset;
    }

    private static void plotPaths(final String title, final Track speeds) {
        final XYDataset dataset = getPathsDataset(speeds);
        final JFreeChart chart = ChartFactory.createXYLineChart("Track paths - " + title, "x (ft)", "y (ft)", dataset);
        showChartInWindow(chart);
    }

    private static XYDataset getPathsDataset(Track track) {
        final DefaultXYDataset dataset = new DefaultXYDataset();
        dataset.addSeries("Path", track.pathDataset());
        return dataset;
    }

}
