package org.firstinspires.ftc.teamcode.utils;

import java.util.ArrayList;

// function that provides a piecewise math function connecting the dots between an arbitrary number of points.
public class PiecewiseFunction {
    private ArrayList<Double> elementsX, elementsY;
    private double defaultValue = Double.MAX_VALUE;
    private int CoordSize = 0, SelectedIndex = 0;
    private boolean Calc_Y = false, Clamped = false, DefaultHigh = false, ClampLimits = true;

    public void setCoordSize(int newN) {
        while (elementsX.size() < newN) elementsX.add(defaultValue);
        while (elementsY.size() < newN) elementsY.add(defaultValue);
        while (elementsX.size() > newN) elementsX.remove(elementsX.size() - 1);
        while (elementsY.size() > newN) elementsY.remove(elementsY.size() - 1);
        CoordSize = newN;
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
        CoordSize = elementsX.size();
    }

    // Default value that will be included in the arrays
    public void setDefaultValue(double newDefault) {defaultValue = newDefault;}

    // In the case of two X points on top of each other, use the high value or the low value
    public void setDefaultHigh(boolean newDefaultHigh) {DefaultHigh = newDefaultHigh;}

    // If TRUE, the provided X is clamped to [firstX, lastX]
    public void setClampLimits(boolean newLimits) {ClampLimits = newLimits;}

    // get default value. Mostly so you can compare outputted Ys against it.
    public double getDefaultValue() {return defaultValue;}

    // If TRUE, getY(X) will return the higher of two Y values for a single X value;
    public boolean getDefaultHigh() {return DefaultHigh;}

    // Returns the number of points in the elements; Must be larger than 1
    public int getCoordSize() {return CoordSize;}

    // Returns the currently active index; Could be used for "stages" defined by the curve
    public int getSelectedIndex() {return SelectedIndex;}

    // Returns TRUE if clamping the output is enabled
    public boolean clampsEnabled() {return ClampLimits;}

    // Returns TRUE if the output is currently being clamped to a limit
    public boolean isClamped() {return Clamped;}

    // returns TRUE if all settings are valid and providing an X will return a valid Y
    public boolean isValid() {
        // If the number of UsePoints doesn't fit within the Size, that is an error.
        boolean error = (CoordSize <= 1);

        // Make sure that the points given make sense
        if (!error) {
            // Check that all X values are in increasing order (IE, list of points is sorted by X value, with duplicates allowed)
            for (int i = 1; i < CoordSize; i++) {
                error = error || (elementsX.get(i) < elementsX.get(i - 1));
            }
        }
        return error;
    }

    // given an X value, calculate the appropriate Y value
    public double getY(double X) {
        // if there is an error, return the default value;
        if (!isValid()) return defaultValue;

        double Y = 0.0;
        double M;
        double B;
        // Set several values to good defaults.
        Calc_Y = true;
        SelectedIndex = -1;
        Clamped = false;

        // Check to see if X is less than the lowest X
        if (X < elementsX.get(0)) {
            // If ClampLimits is TRUE, or the resulting line would be vertical, apply the clamp
            if (ClampLimits || elementsX.get(1) == elementsX.get(0)) {
                Y = elementsY.get(0);
                Calc_Y = false;
                Clamped = true;
            } else {// ClampLimits is not TRUE, calculate M and B from the first pair of points
                SelectedIndex = 1;
                Calc_Y = true;
            }
        }

        // Check to see if X falls exactly on a point
        // If DefaultHigh is true, search through the points in reverse order
        if (DefaultHigh) {
            for (int i = CoordSize - 1; i == 0; i--) {
                if (X == elementsX.get(i)) {
                    Y = elementsY.get(i);
                    Calc_Y = false;
                }
            }
        } else { // If DefaultHigh is false, search through the points in forward order
            for (int i = 0; i < CoordSize - 1; i++) {
                if (X == elementsX.get(i)) {
                    Y = elementsY.get(i);
                    Calc_Y = false;
                }
            }
        }

        // Check to see if X is greater than the highest X
        if (X > elementsX.get(CoordSize - 1)) {
            if (X > elementsX.get(CoordSize - 1)) {
                // If ClampLimits is TRUE, or if the resulting line would be vertical, apply the clamp
                if (ClampLimits || (elementsX.get(CoordSize - 1) == elementsX.get(CoordSize - 2))) {
                    Y = elementsY.get(CoordSize - 1);
                    Calc_Y = false;
                    Clamped = true;
                } else { // ClampLimits is not TRUE, calculate M and B from the last pair of points
                    SelectedIndex = CoordSize - 1;
                    Calc_Y = true;
                }
            }
        }

        // Cycle through every used point, starting at 1
        for (int i = 1; i < CoordSize - 1; i++) {
            // Check to see which pair of points X falls between
            if (X < elementsX.get(i) && X > elementsX.get(i - 1)) {
                SelectedIndex = i;
                break;
            }
        }

        // If the flag hasn't been reset and SelectedIndex is valid, calculate Y
        if (Calc_Y && SelectedIndex < CoordSize && SelectedIndex > 0) {
            M = (elementsY.get(SelectedIndex) - elementsY.get(SelectedIndex - 1))/( elementsX.get(SelectedIndex) - elementsX.get(SelectedIndex - 1));
            B = elementsY.get(SelectedIndex) - M * elementsX.get(SelectedIndex);
            Y = M*X + B;
        }
        return Y;
    }
}
