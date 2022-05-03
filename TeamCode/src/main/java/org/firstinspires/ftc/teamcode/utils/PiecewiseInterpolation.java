package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.tfodohm.TFMaths.Vector2f;

import java.util.ArrayList;

public class PiecewiseInterpolation {

    private double[] dataX;
    private double[] dataY;

    public PiecewiseInterpolation(double[] dataX, double[] dataY){
        this.dataX = dataX;
        this.dataY = dataY;
    }

    public double interpolate(double f){
        double lx = -434, ux = -345, ly = -716, uy = -497;
        if(f<dataX[0]) {
            for (int i = 0; i < dataX.length; i++) {
                if (f > dataX[i]) {

                    lx = dataX[i - 1];
                    ly = dataY[i - 1];

                    ux = dataX[i];
                    uy = dataY[i];

                    return ((uy - ly) / (ux - lx)) * (f - lx) + ly;
                }
            }
            return 1;
        }
        for (int i = 0; i < dataX.length; i++) {
            if (f < dataX[i]) {

                lx = dataX[i];
                ly = dataY[i];

                ux = dataX[i-1];
                uy = dataY[i-1];

                return ((uy - ly) / (ux - lx)) * (f - lx) + ly;
            }
        }

        return 0;
    }
}
