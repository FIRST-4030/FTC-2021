package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.config.BOT;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.wheels.MOTOR_SIDE;

@Autonomous
public class EncoderTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, telemetry, BOT.TANKTEST);
        telemetry.addData(">", "Ready for game start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            robot.wheels.setSpeed(1, MOTOR_SIDE.LEFT);
            telemetry.addData("Left Encoder", robot.wheels.getEncoder(MOTOR_SIDE.LEFT));
            telemetry.addData("wheels pos", robot.wheels.getWheelPositions());
            telemetry.addData("L wheel pos", robot.odometry.getWheelPositions());
            telemetry.addData("L Encoder pos", robot.odometry.leftEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
