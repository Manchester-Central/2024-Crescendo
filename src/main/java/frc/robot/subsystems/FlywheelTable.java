package frc.robot.subsystems;

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
    private double m_minDistance = 0;
    private double m_maxDistance = 0;

    // holds an ArrayList with a key (distance) as reference to [key]
    ArrayList<TableData> flyTable = new ArrayList<TableData>();


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
        flyTable.clear();
        
        String[] data;

        // If found, read the FlywheelTable file
        try {
            for (String row: lines) {
                data = row.split(",");
                addData(TableData.FromCSV(data));
            }

            flyTable.sort(TableData.getComparator()); // Sort flyTable by each TableData's distanceMeters attribute (lowest to highest)
            for (TableData dataRow : flyTable) {
                System.out.println(dataRow.toString());
            }

            // set the min and max distances
            if (flyTable.size() > 0) {
                m_minDistance = getDistance(0);
                m_maxDistance = getDistance(flyTable.size() - 1);
            }
        } catch (Exception ie) {
            ie.printStackTrace();
            return;
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
     * Gets the interpolated value between a TableData's dependent variable
     * @param bottomTableData the row with the lower distance
     * @param topTableData the row with the higher distance
     * @param targetDistance the point to interpolate the value at
     * @param dependentGetter a lambda function to get the dependent variable
     * @return the interpolated dependent value
     */
    private double getInterpolatedValue(TableData bottomTableData, TableData topTableData, double targetDistance, Function<TableData, Double> dependentGetter) {
        return getInterpolatedValue(
            bottomTableData.getDistanceMeters(),
            topTableData.getDistanceMeters(),
            dependentGetter.apply(bottomTableData),
            dependentGetter.apply(topTableData),
            targetDistance
        );
    }

    /**
     * Creates a new TableData with all dependent variables interpolated
     * @param bottomTableData the row with the lower distance
     * @param topTableData the row with the higher distance
     * @param targetDistance the point to interpolate the values at
     * @return the new TableData with interpolated values
     */
    private TableData getInterpolatedData(TableData bottomTableData, TableData topTableData, double targetDistance) {
        return new TableData(
            targetDistance,
            getInterpolatedValue(bottomTableData, topTableData, targetDistance, td -> td.getLauncherSpeedRPM()),
            getInterpolatedValue(bottomTableData, topTableData, targetDistance, td -> td.getSpeedOffsetRPM()),
            getInterpolatedValue(bottomTableData, topTableData, targetDistance, td -> td.getTiltAngle().getDegrees()),
            getInterpolatedValue(bottomTableData, topTableData, targetDistance, td -> td.getHeightMeters())
        );
    }

    /**
     * Given the distance between the robot and the target, calculate the ideal launcher angle, speed, and lift height to score
     * @param distance - The distance between the robot and the target.
     * @return - A TableData object representing the optimal flywheel speed, tilt, and launcher height for the given distance. An Optional.Empty() if the distance is outside the table.
     */
    public Optional<TableData> getIdealTarget(double distance) {
        if (flyTable.isEmpty() || distance < m_minDistance || distance > m_maxDistance) {
            return Optional.empty();
        }

        int initialIndex = findIndex(distance);
        int topIndex = (initialIndex == 0) ? 1 : initialIndex;
        int botIndex = topIndex - 1;

        var bottomTableData = getTableData(botIndex);
        var topTableData = getTableData(topIndex);

        // Get interpolated values
        var interpolatedValues = getInterpolatedData(bottomTableData, topTableData, distance);

        // Returns a new TableData value with the interpolated values and discrete values
        return Optional.of(new TableData(
            distance,
            interpolatedValues.getLauncherSpeedRPM(),
            topTableData.getSpeedOffsetRPM(), // Default to further row's offset
            interpolatedValues.getTiltAngle().getDegrees(), 
            topTableData.getHeightMeters() // Default to further row's lift height
        ));
    }
}