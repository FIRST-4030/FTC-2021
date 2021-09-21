package org.firstinspires.ftc.teamcode.robot.common;

import org.firstinspires.ftc.teamcode.actuators.ServoFTC;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class Claw implements CommonTask {

    private final Robot robot;
    public final ServoFTC clawServo;

    private float closedPos;
    private float openPos;

    /**
     * This constructor lets you manually set the open and closed positions of the claw
     *
     * @param robot Robot
     * @param clawServo Servo that controls the claw
     * @param closedPos Servo position when the claw is closed
     * @param openPos Servo position when the claw is opened
     */
    public Claw(Robot robot, ServoFTC clawServo, float closedPos, float openPos) {
        this.robot = robot;
        this.clawServo = clawServo;
        this.closedPos = closedPos;
        this.openPos = openPos;
    }

    /**
     * This constructor assumes the min and max of the claw servo are the closed and open
     * positions respectively
     *
     * @param robot Robot
     * @param clawServo Servo that controls the claw
     */
    public Claw(Robot robot, ServoFTC clawServo) {
        this(robot, clawServo, clawServo.getMin(), clawServo.getMax());
    }

    /**
     * Closes the claw
     */
    public void close() {
        clawServo.setPosition(closedPos);
    }

    /**
     * Opens the claw
     */
    public void open() {
        clawServo.setPosition(openPos);
    }

}
