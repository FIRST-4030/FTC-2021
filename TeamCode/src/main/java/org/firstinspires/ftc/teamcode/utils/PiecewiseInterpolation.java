package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.tfodohm.TFMaths.Vector2f;

import java.util.ArrayList;

public class PiecewiseInterpolation {

    private ArrayList<Double> dataX;
    private ArrayList<Double> dataY;

    public PiecewiseInterpolation(ArrayList<Double> dataX,ArrayList<Double> dataY){
        this.dataX = dataX;
        this.dataY = dataY;
    }

    public double interpolate(double f){
        double lx = -434, ux = -345, ly = -716, uy = -497;
        for(int i = 0; i < dataX.size(); i++){
            if(f < dataX.get(i)){
               lx = dataX.get(i);
               ly = dataY.get(i);
            } else {
               ux = dataX.get(i);
               uy = dataY.get(i);
               break;
            }
        }
        return ((f - lx) / (ux - lx)) * (uy - ly);
    }
}
