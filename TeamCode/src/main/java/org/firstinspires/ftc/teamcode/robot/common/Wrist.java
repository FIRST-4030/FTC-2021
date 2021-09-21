package org.firstinspires.ftc.teamcode.robot.common;

import org.firstinspires.ftc.teamcode.actuators.ServoFTC;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class Wrist implements CommonTask {

    private final Robot robot;
    public final ServoFTC wristServo;
    private final float offset;

    // Servo positions when the claw is perpendicular/parallel to the arm
    private float perpendicular;
    private float parallel;
    private float incPerDeg;

    private float wristAngle = 0.0f;
    private float wristServoPos = 0.0f;

    /**
     * Main constructor
     *
     * @param robot Robot
     * @param wristServo Servo that acts as the arm's "wrist"
     * @param offset
     * @param perpendicular Servo position when the claw is perpendicular to the arm
     * @param parallel Servo position when the claw is parallel to the arm
     */
    public Wrist(Robot robot, ServoFTC wristServo, float offset, float perpendicular, float parallel){
        this.robot = robot;
        this.wristServo = wristServo;
        this.offset = offset;
        this.perpendicular = perpendicular;
        this.parallel = parallel;

        // Calculate servo increment per degree
        this.incPerDeg = Math.abs(perpendicular - parallel) / 90.0f;
    }

    /**
     * Assumes that the claw is perpendicular to the arm at the wrist servo's min, and parallel
     * at the max
     *
     * @param robot Robot
     * @param wristServo Servo that acts as the arm's "wrist"
     */
    public Wrist(Robot robot, ServoFTC wristServo, float offset) {
        this(robot, wristServo, offset, wristServo.getMin(), wristServo.getMax());
    }

    /**
     * Sets the wrist's angle
     *
     * 0 degrees is when the claw is perpendicular to the arm
     * @param degrees Angle (0-90)
     */
    public void setAngle(float degrees) {
        // Limit angle
        if (degrees > 90) degrees = 90;
        if (degrees < 0) degrees = 0;
        wristAngle = degrees;

        // Check which way the wrist rotates
        if (perpendicular > parallel) {
            wristServoPos = (degrees * incPerDeg) - perpendicular;
        } else {
            wristServoPos = (degrees * incPerDeg) + perpendicular;
        }

        // Move servo
        wristServo.setPosition(wristServoPos + offset);
    }

    public void setAngleDelta(float degrees) {
        this.setAngle(wristAngle + degrees);
    }

    public float getWristAngle() {
        return wristAngle;
    }

    public float getWristServoPos() {
        return wristServoPos;
    }
}
