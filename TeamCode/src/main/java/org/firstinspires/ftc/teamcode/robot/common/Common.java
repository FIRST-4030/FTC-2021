package org.firstinspires.ftc.teamcode.robot.common;

import org.firstinspires.ftc.teamcode.robot.Robot;

/*
 * These are robot-specific helper methods
 * They exist to encourage code re-use across classes
 *
 * They are a reasonable template for future robots, but are unlikely to work as-is
 */
public class Common {

    // Runtime
    private final Robot robot;
    public final Drive drive;
    public Arm arm;
    public Claw claw;

    public Common(Robot r) {
        if (r == null) {
            throw new IllegalArgumentException(this.getClass().getSimpleName() + ": Null robot");
        }
        this.robot = r;

        switch (robot.bot) {
            case ARM:
                arm = new Arm(robot);
                claw = new Claw(robot, robot.claw);
                break;

            case PRODUCTION:

        }

        this.drive = new Drive(robot);
    }
}
