package org.firstinspires.ftc.teamcode.wheels;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.lang.Math.*;

public class Odometry {
    public static final String L_POD = "BL";
    public static final String R_POD = "FR";
    public static final double TICKS_PER_INCH = 1739.0 + 2.0/3.0;
    public static final double LR_POD_DISTANCE = 11.52;
    public static final float M_POD_DISTANCE = 4;
    // The three motors of whose encoders shall be used
    private DcMotor leftEncoder;
    private DcMotor rightEncoder;
    private DcMotor midEncoder;

    //Delta encoder values
    public double s1;
    public double s2;
    private double s3;

    //Values from previous loop
    private double ll = 0;
    private double lr = 0;
    private double lm = 0;

    private HardwareMap hardwareMap;

    private Pose2d coords;

    // Constructors
    public Odometry (HardwareMap map, int startX, int startY, String left, String right){
        leftEncoder = map.dcMotor.get(left);
        leftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        rightEncoder = map.dcMotor.get(right);
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        coords = new Pose2d (startX, startY, 0);
    }
    public Odometry (HardwareMap map, int startX, int startY, String left, String right, String mid){
        this(map, startX, startY, left, right);
        midEncoder = map.dcMotor.get(mid);

        midEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        midEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Loop
    public void update (){
        // DO MATHY STUFF
        //Temp vars for easy reference
        double pi = Math.PI;
        double l = leftEncoder.getCurrentPosition();
        double r = rightEncoder.getCurrentPosition();
        double m = midEncoder.getCurrentPosition();
        double dX;
        double dY;
        Pose2d newPosition;
        //Get change in encoder values and store them
        s1 = (l - ll) / TICKS_PER_INCH;
        s2 = (r - lr) / TICKS_PER_INCH;
        s3 = (m - lm) / TICKS_PER_INCH;
        double theta = (s1-s2)*360/(2*3.14159*LR_POD_DISTANCE);
        double gamma = coords.getHeading() + theta;
        if(theta != 0){
            double r1 = s1*57.2958/theta;
            double r2 = s2*57.2958/theta;
            double r3 = s1*57.2958/theta;

            double turnRadius = r1-LR_POD_DISTANCE/2;
            dX = turnRadius*Math.sin(theta*pi/180);
            dY = turnRadius*(1-Math.cos(theta*pi/180));

        } else {
            dY = 0;
            dX = s1;
        }
        double X = coords.getX();
        double Y = coords.getY();
        double gX = X+dY*Math.sin(gamma*pi/180)+dX*Math.cos(gamma*pi/180);
        double gY = Y-dY*Math.cos(gamma*pi/180)+dX*Math.sin(gamma*pi/180);
        newPosition = new Pose2d(gX,gY, gamma);
        coords = newPosition;
        /*
        X = (d1cosθ1h1-d2cosθ2h2)/(cosθ1h1-cosθ2h2)
R (radians) = (d2-x)cosθ2h2
Y = (d3 - (R/cosθ3h3)
         */

        //Store current values for next loop
        ll = l;
        lr = r;
        lm = m;
    }

    public Pose2d getPosition (){
        return coords;
    }
    public double getLeftEncoder(){
        return leftEncoder.getCurrentPosition()/TICKS_PER_INCH;
    }
    public double getRightEncoder(){
        return rightEncoder.getCurrentPosition()/TICKS_PER_INCH;
    }
    public double getS1(){
        return s1;
    }
    public double getS2(){
        return s2;
    }

}
