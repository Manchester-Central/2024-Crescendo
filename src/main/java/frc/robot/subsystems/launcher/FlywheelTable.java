package frc.robot.subsystems.launcher;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

import edu.wpi.first.wpilibj.Filesystem;

/**
 * Helps us figure out our target speed, tilt, height for launching (in a very naive way)
 */
public class FlywheelTable {

    public static final String FlywheelTableLowerHeight = "FlywheelTableLowerHeight.csv";
    public static final String FlywheelTableUpperHeight = "FlywheelTableUpperHeight.csv";
    private double m_minTY = 0;
    private double m_maxTY = 0;
    private double m_minDistanceMeters = 0;
    private double m_maxDistanceMeters = 0;

    // holds an ArrayList with a key (distance) as reference to [key]
    ArrayList<LauncherTargetTableData> flyTableByTY = new ArrayList<LauncherTargetTableData>();
    ArrayList<LauncherTargetTableData> flyTableByDistance = new ArrayList<LauncherTargetTableData>();

    /**
     * Creates a FlyWheel table for the given fileName in the deploy directory
     * @param fileNamethe file in the deploy directory to read
     */
    public FlywheelTable(String fileName) {
        var lines = readCsvFile(fileName);
        parseLines(lines);
    }

    /**
     * Creates a FlyWheel table with the given lines (used by unit tests)
     * @param lines the csv lines to convert to TableDatas
     */
    public FlywheelTable(List<String> lines) {
        parseLines(lines);
    }

    /**
     * Reads a CSV file's lines
     * @param fileName the file in the deploy directory to read
     * @return the lines of the file as a string array (with the header row removed)
     */
    public ArrayList<String> readCsvFile(String fileName) {
        var path = Filesystem.getDeployDirectory().toPath().resolve(fileName);
        var lines = new ArrayList<String>();
        String row;
        BufferedReader csvReader;
        try {
            csvReader = new BufferedReader(new FileReader(path.toString()));
        } catch (FileNotFoundException ie) {
            ie.printStackTrace();
            return lines;
        }

        // If found, read the FlywheelTable file
        try {
            csvReader.readLine(); // skips 1st line
            while ((row = csvReader.readLine()) != null) {
                lines.add(row);
            }
            csvReader.close();
            return lines;

        } catch (Exception ie) {
            ie.printStackTrace();
            return lines;
        }
    }

    /**
     * Parses the lines into TableData
     * @param lines the string array of the CSV file (with no header row) 
     */
    public void parseLines(List<String> lines) {
        flyTableByTY.clear();
        flyTableByDistance.clear();

        String[] data;

        // If found, read the FlywheelTable file
        try {
            for (String row: lines) {
                data = row.split(",");
                addData(LauncherTargetTableData.FromCSV(data));
            }

            flyTableByTY.sort(LauncherTargetTableData.getComparatorTY()); // Sort flyTable by each TableData's distanceMeters attribute (lowest to highest)
            flyTableByDistance.sort(LauncherTargetTableData.getComparatorDistanceM());
            for (LauncherTarget dataRow : flyTableByTY) {
                System.out.println(dataRow.toString());
            }

            // set the min and max distances
            if (flyTableByTY.size() > 0) {
                var lastRowIndex = flyTableByTY.size() - 1;
                m_minTY = getTY(0);
                m_maxTY = getTY(lastRowIndex);
                m_minDistanceMeters = getDistanceMeters(0);
                m_maxDistanceMeters = getDistanceMeters(lastRowIndex);
            }
        } catch (Exception ie) {
            ie.printStackTrace();
            return;
        }
    }

    // takes raw data, adds to data structure
    private void addData(LauncherTargetTableData data) {
        flyTableByTY.add(data);
        flyTableByDistance.add(data);
    }

    private LauncherTargetTableData getTYTableData(int index) {
        return flyTableByTY.get(index);
    }

    private LauncherTargetTableData getDistanceTableData(int index) {
        return flyTableByDistance.get(index);
    }

    private double getTY(int index) {
        return getTYTableData(index).getTY();
    }
    private double getDistanceMeters(int index) {
        return getDistanceTableData(index).getDistanceMeters();
    }

    /**
     * Finds the index of the first element whose value is less than distance.
     * @param ty - The distance between the robot and the target.
     * @return The index of the first element in flyTable with a value less than distance.
     */
    private int findIndexByTY(double ty) {
        for (int i = 0; i < flyTableByTY.size(); i++) {
            if (ty < getTY(i)) {
                return i;
            }
        }
        return flyTableByTY.size() - 1;
    }

    /**
     * Finds the index of the first element whose value is less than distance.
     * @param distance - The distance between the robot and the target.
     * @return The index of the first element in flyTable with a value less than distance.
     */
    private int findIndexByDistance(double distance) {
        for (int i = 0; i < flyTableByDistance.size(); i++) {
            if (distance < getDistanceMeters(i)) {
                return i;
            }
        }
        return flyTableByDistance.size() - 1;
    }

    /**
     * Calculates the slope between two distance values and a dependent variable, and finds the point along the line for the target. (y = mx + b)
     * @param distance1 the first distance value
     * @param distance2 the second distance value
     * @param dependent1 the first dependent value (speed, tilt, etc)
     * @param dependent2 the second dependent value (speed, tilt, etc)
     * @param target the point to interpolate a value at
     * @return the interpolated dependent value
     */
    private double getInterpolatedValue(double distance1, double distance2, double dependent1, double dependent2, double target) {
        double slope = (dependent2 - dependent1) / (distance2 - distance1);
        double intercept = dependent1 - (slope * distance1);

        return (slope * target) + intercept;
    }


    /**
     * Gets the interpolated value between a TableData's dependent variable
     * @param bottomTableData the row with the lower distance
     * @param topTableData the row with the higher distance
     * @param target the point to interpolate the value at
     * @param dependentGetter a lambda function to get the dependent variable
     * @return the interpolated dependent value
     */
    private double getInterpolatedValue(LauncherTargetTableData bottomTableData, LauncherTargetTableData topTableData, double target, Function<LauncherTarget, Double> dependentGetter) {
        return getInterpolatedValue(
            bottomTableData.getTY(),
            topTableData.getTY(),
            dependentGetter.apply(bottomTableData),
            dependentGetter.apply(topTableData),
            target
        );
    }

    /**
     * Creates a new TableData with all dependent variables interpolated
     * @param bottomTableData the row with the lower distance
     * @param topTableData the row with the higher distance
     * @param targetTY the point to interpolate the values at
     * @return the new TableData with interpolated values
     */
    private LauncherTarget getInterpolatedDataByTY(LauncherTargetTableData bottomTableData, LauncherTargetTableData topTableData, double targetTY) {
        return new LauncherTarget(
            getInterpolatedValue(bottomTableData, topTableData, targetTY, td -> td.getLauncherSpeedRPM()),
            getInterpolatedValue(bottomTableData, topTableData, targetTY, td -> td.getSpeedOffsetRPM()),
            getInterpolatedValue(bottomTableData, topTableData, targetTY, td -> td.getTiltAngle().getDegrees()),
            getInterpolatedValue(bottomTableData, topTableData, targetTY, td -> td.getHeightMeters())
        );
    }

     /**
     * Creates a new TableData with all dependent variables interpolated
     * @param bottomTableData the row with the lower distance
     * @param topTableData the row with the higher distance
     * @param targetDistance the point to interpolate the values at
     * @return the new TableData with interpolated values
     */
    private LauncherTarget getInterpolatedDataByDistance(LauncherTargetTableData bottomTableData, LauncherTargetTableData topTableData, double targetDistance) {
        return new LauncherTarget(
            getInterpolatedValue(bottomTableData, topTableData, targetDistance, td -> td.getLauncherSpeedRPM()),
            getInterpolatedValue(bottomTableData, topTableData, targetDistance, td -> td.getSpeedOffsetRPM()),
            getInterpolatedValue(bottomTableData, topTableData, targetDistance, td -> td.getTiltAngle().getDegrees()),
            getInterpolatedValue(bottomTableData, topTableData, targetDistance, td -> td.getHeightMeters())
        );
    }

    /**
     * Given the distance between the robot and the target, calculate the ideal launcher angle, speed, and lift height to score
     * @param ty - The distance between the robot and the target.
     * @return - A TableData object representing the optimal flywheel speed, tilt, and launcher height for the given distance. An Optional.Empty() if the distance is outside the table.
     */
    public Optional<LauncherTarget> getIdealTargetByTY(double ty) {
        if (flyTableByTY.isEmpty() || ty < m_minTY || ty > m_maxTY) {
            return Optional.empty();
        }

        int initialIndex = findIndexByTY(ty);
        int topIndex = (initialIndex == 0) ? 1 : initialIndex;
        int botIndex = topIndex - 1;

        var bottomTableData = getTYTableData(botIndex);
        var topTableData = getTYTableData(topIndex);

        // Get interpolated values
        var interpolatedValues = getInterpolatedDataByTY(bottomTableData, topTableData, ty);

        // Returns a new TableData value with the interpolated values and discrete values
        return Optional.of(new LauncherTarget(
            interpolatedValues.getLauncherSpeedRPM(),
            bottomTableData.getSpeedOffsetRPM(), // Default to further row's offset
            interpolatedValues.getTiltAngle().getDegrees(), 
            bottomTableData.getHeightMeters() // Default to further row's lift height
        ));
    }

    /**
     * Given the distance between the robot and the target, calculate the ideal launcher angle, speed, and lift height to score
     * @param distance - The distance between the robot and the target.
     * @return - A TableData object representing the optimal flywheel speed, tilt, and launcher height for the given distance. An Optional.Empty() if the distance is outside the table.
     */
    public Optional<LauncherTarget> getIdealTargetByDistance(double distance) {
        if (flyTableByDistance.isEmpty() || distance < m_minDistanceMeters || distance > m_maxDistanceMeters) {
            return Optional.empty();
        }

        int initialIndex = findIndexByDistance(distance);
        int topIndex = (initialIndex == 0) ? 1 : initialIndex;
        int botIndex = topIndex - 1;

        var bottomTableData = getDistanceTableData(botIndex);
        var topTableData = getDistanceTableData(topIndex);

        // Get interpolated values
        var interpolatedValues = getInterpolatedDataByDistance(bottomTableData, topTableData, distance);

        // Returns a new TableData value with the interpolated values and discrete values
        return Optional.of(new LauncherTarget(
            interpolatedValues.getLauncherSpeedRPM(),
            bottomTableData.getSpeedOffsetRPM(), // Default to further row's offset
            interpolatedValues.getTiltAngle().getDegrees(), 
            bottomTableData.getHeightMeters() // Default to further row's lift height
        ));
    }

    public double getMinTY() {
        return m_minTY;
    }

    public double getMaxTY() {
        return m_maxTY;
    }

    public double getMinDistanceMeters() {
        return m_minDistanceMeters;
    }

     public double getMaxDistanceMeters() {
        return m_maxDistanceMeters;
    }
}