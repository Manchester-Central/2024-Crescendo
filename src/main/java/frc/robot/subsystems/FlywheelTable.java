package frc.robot.subsystems;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class FlywheelTable {

    String row;
    BufferedReader csvReader;
    public static final String PATH = "FlywheelTable.csv";
    String[] data;
    double[] doubleData;

    // holds an ArrayList with a key (distance) as reference to [key]
    ArrayList<TableData> flyTable = new ArrayList<TableData>();

    public FlywheelTable() {
        readCSV();
    }

    public boolean readCSV() {
        return readCSV(Filesystem.getDeployDirectory().toPath().resolve(PATH));
    }

    // parses data from .csv into doubles for addData()
    public boolean readCSV(Path path) { // change return type
        flyTable.clear();
        
        // Attempt to read FlywheelTable file
        try {
            // System.out.println(realPath.toString());
            csvReader = new BufferedReader(new FileReader(path.toString()));
        } catch (FileNotFoundException ie) {
            ie.printStackTrace();
            return false;
        }

        // If found, read the FlywheelTable file
        try {
            csvReader.readLine(); // skips 1st line
            while ((row = csvReader.readLine()) != null) {
                // System.out.println(row);
                data = row.split(",");
                addData(TableData.FromCSV(data));
            }

            flyTable.sort(TableData.getComparator()); // Sort flyTable by each TableData's distanceMeters attribute
            for (TableData data : flyTable) {
                System.out.println(data.toString());
            }
            return true;

        } catch (Exception ie) {
            ie.printStackTrace();
            return false;
        }
    }

    // takes raw data, adds to data structure
    private void addData(TableData data) {
        // System.out.println(data.getDistance() + " " + data.getSpeed() + " " + data.getAngle());
        flyTable.add(data);
    }

    private TableData getTableData(int index) {
        return flyTable.get(index);
    }

    private double getDistance(int index) {
        return getTableData(index).getDistanceMeters();
    }

    private double getSpeed(int index) {
        return getTableData(index).getLauncherSpeedRPM();
    }

 private double getTilt(int index) {
        return getTableData(index).getTiltAngle().getDegrees();
    }

    /**
     * Finds the index of the first element whose value is less than distance.
     * @param distance - The distance between the robot and the target.
     * @return The index of the first element in flyTable with a value less than distance.
     */
    private int findIndex(double distance) {
        for (int i = 0; i < flyTable.size(); i++) {
            if (distance < getDistance(i)) {
                return i;
            }
        }
        return flyTable.size() - 1;
    }

    private double getInterpolatedValue(double x1, double x2, double y1, double y2, double distance) {
        double slope = (y2 - y1) / (x2 - x1);
        double intercept = y1 - (slope * x1);

        return (slope * distance) + intercept;
    }

    /**
     * This is a step function courstey of Lord Gartrand.
     * @param distance - The distance between the robot and the target.
     * @return - The greatest distance value smaller than distance in FlywheelTable.csv.
     */
    private double stepHeight(double distance) {
        for(int i = flyTable.size() - 1; i >= 0; i--) {
            if(distance > getDistance(i)) {
                return getTableData(i).getHeightMeters();
            }
        }
        return Double.MAX_VALUE; // Gartrand the Destoryer has found you.
    }

    /**
     * Given the distance between the robot and the target, calculate the ideal launcher angle, speed, and lift height to score
     * @param distance - The distance between the robot and the target.
     * @return - A TableData object representing the optimal flywheel speed, tilt, and launcher height for the given angle.
     */
    public TableData getIdealTarget(double distance) {
        int initialIndex = findIndex(distance);
        int topIndex = (initialIndex == 0) ? 1 : findIndex(distance); // Change this to (initialIndex == 0) ? 1 : initialIndex
        int botIndex = topIndex - 1;

        double idealSpeed = getInterpolatedValue(getDistance(topIndex), getDistance(botIndex), getSpeed(topIndex), getSpeed(botIndex), distance);
        double idealTilt = getInterpolatedValue(getDistance(topIndex), getDistance(botIndex), getTilt(topIndex), getTilt(botIndex), distance);
        return new TableData(
        distance,
        idealSpeed,
        idealTilt, 
        stepHeight(distance)
        );
    }
}