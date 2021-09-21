package org.firstinspires.ftc.teamcode.robot.common;

import org.firstinspires.ftc.teamcode.actuators.ServoFTC;
import org.firstinspires.ftc.teamcode.config.BOT;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class Arm implements CommonTask {
    private static boolean DEBUG = false;

    private final Robot robot;
    private final ServoFTC lower;
    private final ServoFTC upper;

    // Servo positions when the servo is at 90 degrees
    private static final float LOWER_MIDPOINT = 0.5f;
    private static final float UPPER_MIDPOINT = 0.3f;

    // The arm has two segments, measured from joint to joint
    // The lower segment is from the base to the middle joint
    // The upper segment is from the base to the claw joint
    // lengths are measured in inches
    private static final double LOWER_LENGTH = 12.625;
    private static final double UPPER_LENGTH = 12.625;

    // Circle for arm limit checking
    private static final double CENTER_X = 4.77;
    private static final double CENTER_Y = 2.16;
    private static final double RADIUS = 7.22;

    // current arm position
    private float armX = 0.0f;
    private float armY = 0.0f;

    public Arm(Robot robot) {
        this.robot = robot;

        // Set which servos control the arm
        this.lower = robot.lower;
        this.upper = robot.upper;
    }

    /**
     * Moves arm to specified position
     * x and y are when looking at the arm from the side on a 2D plane
     * @param x x position in inches (extension)
     * @param y y position in inches (elevation)
     */
    public void setPosition(float x, float y) {
        if (robot.bot != BOT.ARM) {
            return;
        }

        // check position// check new arm position
        if (!checkPosition(x, y)) {
            return;
        }

        // update arm pos
        armX = x;
        armY = y;


        // trig OwO

        // first, find the hypotenuse (from the base to the desired pos)
        double hypot = Math.sqrt(x * x + y * y);

        // use the law of cosines to find the internal angles of the triangle made by the arm
        // segments and the hypotenuse
        double B = lawOfCosines(LOWER_LENGTH, hypot, UPPER_LENGTH);
        double A = lawOfCosines(UPPER_LENGTH, LOWER_LENGTH, hypot);

        // calculate the angles the servos need to be at
        // the lower servo is the angle between the hypotenuse and the x-axis plus B
        float S1 = (float) (Math.atan(y/x) + B);
        // the upper servo is the difference between A and the third angle of the right
        // triangle made by the x-axis and the lower segment
        double M = Math.PI - (S1 + Math.PI/2);
        float S2 = (float) (A - M);

        // Make the angles usable servo positions
        S1 = (float) (S1 / Math.PI) + (LOWER_MIDPOINT - 0.5f);
        S2 = (float) (S2 / Math.PI) + (UPPER_MIDPOINT - 0.5f);

        if (DEBUG) {
            robot.telemetry.addData("lower raw", ((Math.asin(y / hypot) + B)));
            robot.telemetry.addData("upper raw", (A - (Math.PI - (Math.PI/2 + S1))));
            robot.telemetry.addData("lower pos", S1);
            robot.telemetry.addData("upper pos", S2);
        }

        // set servos to those positions
        lower.setPosition(S1);
        upper.setPosition(S2);
    }

    /**
     * Changes the current position of the arm by x and y
     * @param x change in the x position (inches)
     * @param y change in the y position (inches)
     */
    public void setPositionDelta(float x, float y) {
        // change arm position
        float newX = armX + x;
        float newY = armY + y;

        // move arm
        setPosition(newX, newY);
    }

    /**
     * Checks if the given arm position (x, y) will fall into the range of motion
     * of the arm. This will need to be updated if the arm changes
     * @param x proposed X position of the arm (in inches)
     * @param y proposed Y position of the arm (in inches)
     * @return true if the arm position is valid, false otherwise
     */
    public boolean checkPosition(float x, float y) {
        // Check if the triangle that's gonna be made is possible
        double hypot = Math.sqrt(x*x + y*y);
        // no side should be longer than the other two added together
        if (hypot >= UPPER_LENGTH + LOWER_LENGTH ||
                UPPER_LENGTH >= hypot + LOWER_LENGTH ||
                LOWER_LENGTH >= UPPER_LENGTH + hypot)
            return false;

        // minimum angle A between the two arms
        double min_A_angle = 15 * Math.PI / 180;
        double requestedRange = Math.sqrt(x*x + y*y);
        double minRange = Math.sqrt(UPPER_LENGTH*UPPER_LENGTH+LOWER_LENGTH*LOWER_LENGTH-2*UPPER_LENGTH*LOWER_LENGTH*Math.cos(min_A_angle));
        double maxRange = LOWER_LENGTH + UPPER_LENGTH;

        if ((requestedRange > maxRange) || (requestedRange < minRange)) return false;

        /*
        // Big circle limit (magic internet math)
        if(((x-CENTER_X)*(x-CENTER_X) + (y-CENTER_Y)*(y-CENTER_Y) > RADIUS*RADIUS))
            return false;

        // Check the little slice of the circle
        if (y < ((-5.0/3.0) * x + 6.0))
            return false;
       */

        // keep x positive
        if (x <= 0)
            return false;

        return true;
    }

    private double lawOfCosines(double a, double b, double c) {
        return Math.acos((a*a + b*b - c*c) / (2*a*b));
    }

    // Getters for current arm pos
    public float getArmX() {return armX;}
    public float getArmY() {return armY;}
}