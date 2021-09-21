package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.buttons.BUTTON_TYPE;
import org.firstinspires.ftc.teamcode.buttons.ButtonHandler;
import org.firstinspires.ftc.teamcode.buttons.PAD_BUTTON;
import org.firstinspires.ftc.teamcode.config.BOT;
import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.drive.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.robot.auto.NewAuto;
import org.firstinspires.ftc.teamcode.robot.auto.RingStackTF;
import org.firstinspires.ftc.teamcode.wheels.MOTOR_END;
import org.firstinspires.ftc.teamcode.wheels.MOTOR_SIDE;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Prod")
public class TeleOpMode extends LinearOpMode implements RobotConstants {

    // Devices and subsystems
    private Robot robot = null;
    private ButtonHandler buttons;

    private boolean controlLocked = false;
    private double shooterSpeed = -1835;


    // other constants
    private static final float NORMAL_SPEED = 1.0f;
    private static final float SLOW_MODE = 0.35f;

    private NewAuto auto;


    @Override
    public void runOpMode() {

        controlLocked = false;
        // Placate drivers
        telemetry.addData(">", "NOT READY");
        telemetry.update();

        // Init the common tasks elements
        robot = new Robot(hardwareMap, telemetry);
        robot.wheels.setTeleop(true);
        auto = new NewAuto(robot.wheels.getMotor(MOTOR_SIDE.LEFT, MOTOR_END.FRONT),robot.wheels.getMotor(MOTOR_SIDE.RIGHT, MOTOR_END.FRONT), hardwareMap, robot.odometry);

        // Check robot
        if (robot.bot != BOT.PRODUCTION) {
            telemetry.log().add("Opmode not compatible with bot " + robot.bot);
            requestOpModeStop();
            return;
        }

        // Register buttons
        buttons = new ButtonHandler(robot);
        buttons.register("UPDATE_ODOMETRY", gamepad1, PAD_BUTTON.x);

        buttons.register("SLOW_MODE", gamepad1, PAD_BUTTON.b, BUTTON_TYPE.TOGGLE);
        buttons.register("REVERSE_COLLECTOR", gamepad1, PAD_BUTTON.y);
        buttons.register("FRONT_MID_COLLECT", gamepad1, PAD_BUTTON.left_bumper);
        buttons.register("BACK_MID_COLLECT", gamepad1, PAD_BUTTON.right_bumper);
        buttons.register("FRONT_FULL_COLLECT", gamepad1, PAD_BUTTON.left_trigger);
        buttons.register("BACK_FULL_COLLECT", gamepad1, PAD_BUTTON.right_trigger);
        buttons.register("TOGGLE_ARM", gamepad2, PAD_BUTTON.a, BUTTON_TYPE.TOGGLE);
        buttons.register("TOGGLE_GRIP", gamepad2, PAD_BUTTON.b, BUTTON_TYPE.TOGGLE);
        buttons.register("SHOOT", gamepad2, PAD_BUTTON.right_bumper);
        buttons.register("MANUAL_SHOOT", gamepad2, PAD_BUTTON.right_trigger);
        buttons.register("TOGGLE_MAGAZINE_POS", gamepad2, PAD_BUTTON.left_trigger, BUTTON_TYPE.TOGGLE);

        buttons.register("SHOOTER_SPEED_UP", gamepad2, PAD_BUTTON.dpad_up);
        buttons.register("SHOOTER_SPEED_DOWN", gamepad2, PAD_BUTTON.dpad_down);

        buttons.register("POWER_SHOT_MOVE", gamepad1, PAD_BUTTON.dpad_left);

        // Wait for the game to begin
        telemetry.addData(">", "Ready for game start");
        telemetry.update();

        waitForStart();

        //@Override
        while (opModeIsActive()) {
            // Update buttons
            buttons.update();

            // Move the robot


            if (!controlLocked) {
                driveBase();
                auxiliary();
            }

            telemetry.update();
            idle();
        }
    }

    /*@Override
    public void init_loop() {

    }

    @Override
    public void start() {
    }*/


    //Slow mode
    private void driveBase() {
        robot.odometry.update();
        if (buttons.get("SLOW_MODE")) {
            robot.wheels.setSpeedScale(SLOW_MODE);
        } else {
            robot.wheels.setSpeedScale(NORMAL_SPEED);
        }
        robot.wheels.loop(gamepad1);
    }

    //telemetry data showed on phone during OpMode
    private void auxiliary() {
        telemetry.addData("H:", robot.odometry.getPoseEstimate().getHeading());
        //telemetry.addData("S1:", robot.odometry.getS1());
        //telemetry.addData("S2:", robot.odometry.getS2());
        //telemetry.addData("L:", robot.odometry.getLeftEncoder());
        //telemetry.addData("R:", robot.odometry.getRightEncoder());
        telemetry.addData("X:", robot.odometry.getPoseEstimate().getX());
        telemetry.addData("Y:", robot.odometry.getPoseEstimate().getY());
        telemetry.addData("S:", shooterSpeed);


        //CLAW
        if (buttons.get("TOGGLE_GRIP")) {
            robot.wobbleGoalGrip.setPosition(CLAW_OPEN);
        } else {
            robot.wobbleGoalGrip.setPosition(CLAW_CLOSED);
        }
        //ARM
        if(buttons.get("TOGGLE_ARM")){
            robot.wobbleGoalArm.setPosition(ARM_OUT);
        } else {
            robot.wobbleGoalArm.setPosition(ARM_IN);
        }

        //robot.wobbleGoalArm.setPower(gamepad2.right_stick_y);

        //COLLECT SAFEGUARD
        if(!buttons.get("TOGGLE_MAGAZINE_POS")) {

            // ==[FRONT COLLECTOR]==
            if (buttons.held("FRONT_FULL_COLLECT")) {
               robot.frontRaiseLower.setPosition(COLLECT_FULL_FRONT);
                robot.collectorFront.setPower(1.0f);
            } else if (buttons.held("FRONT_MID_COLLECT")) {
                robot.frontRaiseLower.setPosition(COLLECT_MID);
                robot.collectorFront.setPower(1.0f);
            } else if (buttons.held("REVERSE_COLLECTOR")) {
                robot.frontRaiseLower.setPosition(COLLECT_MID);
            } else {
                robot.frontRaiseLower.setPosition(COLLECT_NO);
                robot.collectorFront.setPower(0.0f);
            }

            // ==[BACK COLLECTOR]==
            if (buttons.held("BACK_FULL_COLLECT")) {
                robot.backRaiseLower.setPosition(COLLECT_FULL_BACK);
                robot.collectorBack.setPower(1.0f);
            } else if (buttons.held("BACK_MID_COLLECT")) {
                robot.backRaiseLower.setPosition(COLLECT_MID);
                robot.collectorBack.setPower(1.0f);

            } else if (buttons.held("REVERSE_COLLECTOR")) {
                robot.backRaiseLower.setPosition(COLLECT_MID);
            } else {
                robot.backRaiseLower.setPosition(COLLECT_NO);
                robot.collectorBack.setPower(0.0f);
            }
        } else {
            robot.backRaiseLower.setPosition(COLLECT_NO);
            robot.collectorBack.setPower(0.0f);
            robot.frontRaiseLower.setPosition(COLLECT_NO);
            robot.collectorFront.setPower(0.0f);
        }

        if (buttons.held("REVERSE_COLLECTOR")) {
            robot.collectorFront.setPower(-1.0f);
            robot.collectorBack.setPower(-1.0f);
        }

        if (buttons.get("TOGGLE_MAGAZINE_POS")) {
            robot.queue.setPosition(MAGAZINE_UP);
            robot.shooter.setVelocity(shooterSpeed);
        } else {
            robot.queue.setPosition(MAGAZINE_DOWN);
            robot.shooter.setVelocity(0);
        }

        if (buttons.get("SHOOTER_SPEED_UP")) {
            shooterSpeed = HIGH_SHOOTER_SPEED;
        } else if (buttons.get("SHOOTER_SPEED_DOWN")) {
            shooterSpeed = POWERSHOT_SHOOTER_SPEED;
        }
        if (buttons.held("MANUAL_SHOOT") && buttons.get("TOGGLE_MAGAZINE_POS")) {
            robot.queueFlipper.setPosition(FLIPPER_SHOOT);
            //controlLocked = true;
        } /*else {
            if(buttons.held("SHOOT") && robot.shooter.getVelocity() >= 2580 && robot.shooter.getVelocity() <= 2600) robot.queueFlipper.setPosition(FLIPPER_SHOOT);
            */
            else robot.queueFlipper.setPosition(FLIPPER_IDLE);


        if (buttons.get("POWER_SHOT_MOVE")) {
            robot.wheels.setTeleop(false);
            auto.drive(-8, 1);
            robot.wheels.setTeleop(true);
        }
    }



    //public void stop() {
    //}
}