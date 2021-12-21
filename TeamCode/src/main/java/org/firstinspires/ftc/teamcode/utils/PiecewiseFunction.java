package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;

// function that provides a piecewise math function connecting the dots between an arbitrary number of points.
public class PiecewiseFunction {
    private ArrayList<Double> elementsX, elementsY;
    private double defaultValue = Double.MAX_VALUE;
    private int coordSize = 0, SelectedIndex = 0;
    private boolean Clamped = false, DefaultHigh = false, ClampLimits = true;

    public boolean debug = false;

    public PiecewiseFunction() {
        elementsX = new ArrayList<>();
        elementsY = new ArrayList<>();
    }

    // return the average value of Y weighted by the length of X
    // for example, given the following points (0, 0), (1, 1), (2, 1), (3, 0)
    // This would return 2/3
    public double getAverage() {
        if (!isValid()) return defaultValue;
        double total = 0.0;
        for (int i = 1; i < coordSize; i++) {
            total += (getElementY(i) - getElementY(i - 1)) * (getElementX(i) - getElementX(i - 1));
        }
        return total / (getLastX() - getFirstX());
    }

    public ArrayList<Double> getElementsX() {
        return elementsX;
    }

    public ArrayList<Double> getElementsY() {
        return elementsY;
    }

    public void setCoordSize(int newN) {
        while (elementsX.size() < newN) elementsX.add(defaultValue);
        while (elementsY.size() < newN) elementsY.add(defaultValue);
        while (elementsX.size() > newN) elementsX.remove(elementsX.size() - 1);
        while (elementsY.size() > newN) elementsY.remove(elementsY.size() - 1);
        coordSize = newN;
    }

    public void setElement(int elementNumber, double newX, double newY) {
        if (elementNumber < elementsX.size()) {
            elementsX.set(elementNumber, newX);
            elementsY.set(elementNumber, newY);
        }
    }

    public void addElement(double newX, double newY) {
        elementsX.add(newX);
        elementsY.add(newY);
        coordSize = elementsX.size();
    }

    public double getElementX(int elementNumber) {
        return elementsX.get(elementNumber);
    }

    public double getElementY(int elementNumber) {
        return elementsY.get(elementNumber);
    }

    public double getLastX() {
        if (isValid()) return elementsX.get(elementsX.size() - 1);
        return defaultValue;
    }

    public double getLastY() {
        if (isValid()) return elementsY.get(elementsY.size() - 1);
        return defaultValue;
    }

    public double getFirstX() {
        if (isValid()) return elementsX.get(0);
        return defaultValue;
    }

    public double getFirstY() {
        if (isValid()) return elementsY.get(0);
        return defaultValue;
    }

    public void removeElement(int removeIndex) {
        if (getCoordSize() > 2) {
            elementsX.remove(removeIndex);
            elementsY.remove(removeIndex);
            coordSize = coordSize - 1;
        }
    }

    /** Reset deletes all elements and returns things to default values.
     * defaultValue is set to max double value.
     * all elements are deleted.
     * Clamped and DefaultHigh are set to false.
     * ClampLimits and debug are set to true.
     */
    public void reset() {
        while (!elementsX.isEmpty()) elementsX.remove(0);
        while (!elementsY.isEmpty()) elementsY.remove(0);
        defaultValue = Double.MAX_VALUE;
        coordSize = 0; SelectedIndex = 0;
        Clamped = DefaultHigh = false; ClampLimits = true;
        debug = false;
    }

    // Default value that will be included in the arrays
    public void setDefaultValue(double newDefault) {defaultValue = newDefault;}

    // In the case of two X points on top of each other, use the high value or the low value
    public void setDefaultHigh(boolean newDefaultHigh) {DefaultHigh = newDefaultHigh;}

    // If TRUE, the provided X is clamped to [firstX, lastX]
    public boolean getClampLimits() {return ClampLimits;}

    // Set whether or not recieved X values are clamped to the first and last X
    public void setClampLimits(boolean newLimits) {ClampLimits = newLimits;}

    // get default value. Mostly so you can compare outputted Ys against it.
    public double getDefaultValue() {return defaultValue;}

    // If TRUE, getY(X) will return the higher of two Y values for a single X value;
    public boolean getDefaultHigh() {return DefaultHigh;}

    // Returns the number of points in the elements; Must be larger than 1
    public int getCoordSize() {return coordSize;}

    // Returns the currently active index; Could be used for "stages" defined by the curve
    public int getSelectedIndex() {return SelectedIndex;}

    // Returns TRUE if clamping the output is enabled
    public boolean clampsEnabled() {return ClampLimits;}

    // Returns TRUE if the output is currently being clamped to a limit
    public boolean isClamped() {return Clamped;}

    // returns TRUE if all settings are valid and providing an X will return a valid Y
    public boolean isValid() {
        // If the number of UsePoints doesn't fit within the Size, that is an error.
        boolean error = (coordSize <= 1);

        // Make sure that the points given make sense
        if (!error) {
            // Check that all X values are in increasing order (IE, list of points is sorted by X value, with duplicates allowed)
            for (int i = 1; i < coordSize; i++) {
                error = error || (elementsX.get(i) < elementsX.get(i - 1));
            }
        }
        return !error;
    }

    // given an X value, calculate the appropriate Y value
    public double getY(double X) {
        // if there is an error, return the default value;
        if (!isValid()) {
            if (debug) RobotLog.d(",debug,settings not valid! Returning DefaultValue");
            return defaultValue;
        }

        double Y = defaultValue;
        double M = defaultValue;
        double B = defaultValue;
        boolean Calc_Y = true;
        // Set several values to good defaults.
        SelectedIndex = -1;
        Clamped = false;

        if (debug) RobotLog.d(",debug,starting calculation");
        if (debug) RobotLog.d(",debug,x = " + X);
        // Check to see if X is less than the lowest X
        if (X < getFirstX()) {
            if (debug) RobotLog.d(",debug,X < first X");
            // If ClampLimits is TRUE, or the resulting line would be vertical, apply the clamp
            if (ClampLimits || elementsX.get(1).equals(elementsX.get(0))) {
                if (debug) RobotLog.d(",debug,clamping at the start");
                Y = elementsY.get(0);
                Calc_Y = false;
                Clamped = true;
            } else {// ClampLimits is not TRUE, calculate M and B from the first pair of points
                if (debug) RobotLog.d(",debug,pre-range linear projection");
                SelectedIndex = 1;
                Calc_Y = true;
            }
        }

        // Check to see if X falls exactly on a point
        // If X falls exactly on two points, apply DefaultHigh to determine which to use
        for (int i = 0; i < coordSize; i++) {
            if (X == elementsX.get(i)) {
                if (debug) RobotLog.d(",debug,X ("+ X +") matches element number " + i + " (" + elementsX.get(i) + ")");

                // Check to see if X falls exactly on the next point as well
                if (i != coordSize - 1) {
                    if (X == elementsX.get(i + 1)) {
                        if (DefaultHigh) {
                            Y = Math.max(elementsY.get(i), elementsY.get(i + 1));
                            if (debug) RobotLog.d(",debug,X ("+ X +") matches element number " + i + " AND element number " + (i + 1) + " default high");
                        } else {
                            Y = Math.min(elementsY.get(i), elementsY.get(i + 1));
                            if (debug) RobotLog.d(",debug,X ("+ X +") matches element number " + i + " AND element number " + (i + 1) + " default low");
                        }
                        Calc_Y = false;
                        break;
                    }
                }
                Y = elementsY.get(i);
                Calc_Y = false;
                break;
            }
        }

        // Check to see if X is greater than the highest X
        if (X > getLastX()) {
            if (debug) RobotLog.d(",debug,X > last X");
            // If ClampLimits is TRUE, or if the resulting line would be vertical, apply the clamp
            if (ClampLimits || (elementsX.get(coordSize - 1).equals(elementsX.get(coordSize - 2)))) {
                if (debug) RobotLog.d(",debug,clamping at the end");
                Y = elementsY.get(coordSize - 1);
                Calc_Y = false;
                Clamped = true;
            } else { // ClampLimits is not TRUE and last line is not vertical, so calculate M and B from the last pair of points
                if (debug) RobotLog.d(",debug,post-range linear projection");
                SelectedIndex = coordSize - 1;
                Calc_Y = true;
            }
        }

        // Cycle through every used point, starting at 1
        for (int i = 1; i < coordSize; i++) {
            // Check to see which pair of points X falls between
            if (X < elementsX.get(i) && X > elementsX.get(i - 1)) {
                SelectedIndex = i;
                if (debug) RobotLog.d(",debug,selectedIndex = " + SelectedIndex);
                break;
            }
        }

        // If the flag hasn't been reset and SelectedIndex is valid, calculate Y
        if (Calc_Y) {
            if (debug) RobotLog.d(",debug,calculating Y,Y is currently," + Y);
            if (SelectedIndex < coordSize && SelectedIndex > 0) {
                if (debug) RobotLog.d(",debug,valid selectedIndex");
                M = (elementsY.get(SelectedIndex) - elementsY.get(SelectedIndex - 1)) / (elementsX.get(SelectedIndex) - elementsX.get(SelectedIndex - 1));
                B = elementsY.get(SelectedIndex) - M * elementsX.get(SelectedIndex);
                Y = M * X + B;
            } else  // the Calc_Y flag is set, but an invalid index is selected
                Y = defaultValue;
        }
        if (debug) RobotLog.d(",debug,x = " + X + ",y = " + Y + ",M = " + M + ",B = " + B + ",selectedIndex = " + SelectedIndex + ",Calc_Y = " + Calc_Y);
        return Y;
    }
}
