package frc.robot.subsystems;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.wpilibj.Filesystem;

/**
 * Helps us figure out our target speed, tilt, height for launching (in a very naive way)
 */
public class FlywheelTable {

    public static final String PATH = "FlywheelTable.csv";
    double minDistance = 0;
    double maxDistance = 0;

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
        
        BufferedReader csvReader;
        String row;
        String[] data;
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

            flyTable.sort(TableData.getComparator()); // Sort flyTable by each TableData's distanceMeters attribute (lowest to highest)
            for (TableData dataRow : flyTable) {
                System.out.println(dataRow.toString());
            }

            // set the min and max distances
            if (flyTable.size() > 0) {
                minDistance = getDistance(0);
                maxDistance = getDistance(flyTable.size() - 1);
            }
            return true;

        } catch (Exception ie) {
            ie.printStackTrace();
            return false;
        }
    }

    // takes raw data, adds to data structure
    private void addData(TableData data) {
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


    /**
     * Calculates the slope between two distance values and a dependent variable, and finds the point along the line for the target. (y = mx + b)
     * @param distance1 the first distance value
     * @param distance2 the second distance value
     * @param dependent1 the first dependent value (speed, tilt, etc)
     * @param dependent2 the second dependent value (speed, tilt, etc)
     * @param targetDistance the point to interpolate a value at
     * @return the interpolated dependent value
     */
    private double getInterpolatedValue(double distance1, double distance2, double dependent1, double dependent2, double targetDistance) {
        double slope = (dependent2 - dependent1) / (distance2 - distance1);
        double intercept = dependent1 - (slope * distance1);

        return (slope * targetDistance) + intercept;
    }

    /**
     * Given the distance between the robot and the target, calculate the ideal launcher angle, speed, and lift height to score
     * @param distance - The distance between the robot and the target.
     * @return - A TableData object representing the optimal flywheel speed, tilt, and launcher height for the given distance. An Optional.Empty() if the distance is outside the table.
     */
    public Optional<TableData> getIdealTarget(double distance) {
        if (distance < minDistance || distance > maxDistance) {
            return Optional.empty();
        }

        int initialIndex = findIndex(distance);
        int topIndex = (initialIndex == 0) ? 1 : initialIndex;
        int botIndex = topIndex - 1;

        // For discrete values, default to the higher distance value
        var topTableData = getTableData(topIndex);

        // Interpolate speed and tilt
        double idealSpeed = getInterpolatedValue(getDistance(topIndex), getDistance(botIndex), getSpeed(topIndex), getSpeed(botIndex), distance);
        double idealTilt = getInterpolatedValue(getDistance(topIndex), getDistance(botIndex), getTilt(topIndex), getTilt(botIndex), distance);

        // Returns a new TableData value with the interpolated values and discrete values
        return Optional.of(new TableData(
            distance,
            idealSpeed,
            topTableData.getSpeedOffsetRPM(),
            idealTilt, 
            topTableData.getHeightMeters()
        ));
    }
}