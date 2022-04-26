package org.firstinspires.ftc.teamcode.sensors.pot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class DoublePotentiometer implements Potentiometer {

    //hardware
    private BasicPotentiometer pot1, pot2;

    private final double scaleRad = Math.PI / 180;

    //potentiometer tuning for linearization - for later
    public final double OFFSET;
    private static double CLIP_TOP = 30;
    private static double CLIP_BOTTOM = 30;

    //initalize hardware
    public DoublePotentiometer(HardwareMap map, Telemetry telemetry, String n1, String n2, double offset){
        if(n1 == null || n2 == null || n1.isEmpty() || n2.isEmpty()){
            throw new IllegalArgumentException(this.getClass().getSimpleName() + ": invalid name");
        }
        ArrayList normalized = new ArrayList<Double>();
        normalized.add(0);
        normalized.add(1);
        pot1 = new BasicPotentiometer(map, telemetry, n1, normalized);
        pot2 = new BasicPotentiometer(map, telemetry, n2, normalized);
        this.OFFSET = offset;
    }

    // see overridden methods
    // some (marked with numbers) allow acces to the individual potentiometers inside the dual unit

    @Override
    public double getMV() {
        return 0;
    }

    public double getMV1(){
        return pot1.getMV();
    }

    public double getMV2(){
        return pot2.getMV();
    }

    @Override
    public double getAngleD() {
        double angle1 = pot1.getAngleD();
        double angle2 = pot2.getAngleD();

        if(angle2 < CLIP_BOTTOM || angle2 > 360-CLIP_TOP || (angle2 > 180-CLIP_TOP && angle2 < 180 + CLIP_BOTTOM)) return pot1.getAngleD();
        else if((angle1 < CLIP_BOTTOM + OFFSET && angle1 > OFFSET - CLIP_TOP) || (angle1 < 180 + CLIP_BOTTOM +OFFSET && angle1 > 180 - CLIP_TOP + OFFSET)) return pot2.getAngleD();
        return (pot1.getAngleD() + pot2.getAngleD())/2;
    }

    @Override
    public double getAngleR() {
      return getAngleD()*scaleRad;
    }

    public boolean isZeroOne(){
        return pot1.isZero();
    }

    public boolean isZeroTwo(){
        return pot2.isZero();
    }

    @Override
    public boolean isZero(){
        return getAngleD() == 0;
    }

    @Override
    public boolean isAvailable() {
        return false;
    }
}
