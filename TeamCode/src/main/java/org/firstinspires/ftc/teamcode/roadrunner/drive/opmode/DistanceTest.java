package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.buttons.BUTTON_TYPE;
import org.firstinspires.ftc.teamcode.buttons.ButtonHandler;
import org.firstinspires.ftc.teamcode.buttons.PAD_BUTTON;

@Config
@Autonomous(name = "DistanceTest", group = "Test")
public class DistanceTest extends OpMode {
    // Hardware
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor duckSpinner = null;
    private DcMotor depBelt = null;
    private Servo depLow = null;
    private Servo depMid = null;
    private Servo depTilt = null;
    private DcMotor collector = null;
    private Servo collectorArm = null;
    private Servo capstoneArm = null;
    private DistanceSensor distanceLeft = null;
    private DistanceSensor distanceRight = null;

    // Consts
    private static float DRIVE_POWER = 0.375f;
    private static double TICKS_PER_INCH = 43.24;
    private static double TURN_RATIO = 8;
    private static double ANGLE_CONST = 1.23;
    private static double SENSOR_TIME = 1;
    private static double leftSensorAccum = 0;
    private static double rightSensorAccum = 0;
    private static int objectPos = 0;

    // Members
    private ElapsedTime sensorTimer = new ElapsedTime();

    // Members
    private boolean done = false;
    private boolean driveCmdRunning = false;
    private int autoStep = 0;
    private boolean leftinRange = false;
    private boolean rightinRange = false;

    @Override
    public void init() {
        boolean error = false;
        telemetry.addData("Status", "Initializing...");

        // Drive Motors
        try {
            leftDrive = hardwareMap.get(DcMotor.class, "BL");
            rightDrive = hardwareMap.get(DcMotor.class, "BR");
            leftDrive.setDirection(DcMotor.Direction.FORWARD);
            rightDrive.setDirection(DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            telemetry.log().add("Could not find drive");
            error = true;
        }

        // Distance Sensors
        try {
            distanceLeft = hardwareMap.get(DistanceSensor.class, "DL");
            distanceRight = hardwareMap.get(DistanceSensor.class, "DR");
        } catch (Exception e) {
            telemetry.log().add("Could not find distance sensors");
            error = true;
        }

        // Initialization status
        String status = "Ready";
        if (error) {
            status = "Hardware Error";
        }
        telemetry.addData("Status", status);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        driveStop();
        done = false;
        autoStep = 0;
    }

    @Override
    public void loop() {
        // Stop when the autoSteps are complete
        if (done) {
            requestOpModeStop();
            return;
        }

        // Feedback
        telemetry.addData("Drive", "L %.2f/%d, R %.2f/%d",
                leftDrive.getPower(), leftDrive.getCurrentPosition(),
                rightDrive.getPower(), rightDrive.getCurrentPosition());

        // Don't start new commands until the last one is complete
        if (driveCmdRunning) {
            // When the motors are done
            if (!isBusy()) {
                // Clear the running flag
                driveCmdRunning = false;
                // Advance to the next autoStep
                autoStep++;
            }
            // Continue from the top of loop()
            return;
        }

        //
        // Code past here is not run when driveCmdRunning is true
        //

        // Step through the auto commands
        switch (autoStep) {
            case 0:
                sensorTimer.reset();
                leftSensorAccum = 0;
                leftSensorAccum = (leftSensorAccum * 0.9) + (0.1 * distanceLeft.getDistance(DistanceUnit.INCH));
                if (sensorTimer.seconds() > SENSOR_TIME) {
                    autoStep++;
                }
                break;
            case 1:
                if (leftSensorAccum <= 19) {
                    leftinRange = true;
                }
                autoStep++;
                break;
            case 2:
                sensorTimer.reset();
                rightSensorAccum = 0;
                rightSensorAccum = (rightSensorAccum * 0.9) + (0.1 * distanceRight.getDistance(DistanceUnit.INCH));
                if (sensorTimer.seconds() > SENSOR_TIME) {
                    autoStep++;
                }
                break;
            case 3:
                if (rightSensorAccum <= 19) {
                    rightinRange = true;
                }
                autoStep++;
                break;
            case 4:
                if (leftinRange && !rightinRange) {
                    objectPos = 1;
                    telemetry.addData("Position: ", objectPos);
                } else if (!leftinRange && rightinRange){
                    objectPos = 3;
                    telemetry.addData("Position: ", objectPos);
                } else if (!leftinRange && !rightinRange) {
                    objectPos = 2;
                    telemetry.addData("Position: ", objectPos);
                } else {
                    objectPos = 0;
                    telemetry.addData("Position: ", objectPos);
                }
                break;
            // If we get past the end stop and end the loop
            default:
                driveStop();
                done = true;
                break;
        }
    }

    @Override
    public void stop() {
        // Reset to the standard drive mode
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveStop() {
        // Stop, zero the drive encoders, and enable RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setTargetPosition(leftDrive.getTargetPosition());
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setPower(0);

        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setTargetPosition(rightDrive.getTargetPosition());
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setPower(0);
    }

    public boolean isBusy() {
        return leftDrive.isBusy() || rightDrive.isBusy();
    }

    public void driveTo(float speed, float distance) {
        // Don't allow new moves if we're still busy
        if (isBusy()) {
            telemetry.log().add("driveTo(): Motors in use");
            return;
        }

        // Set a target, translated from inches to encoder ticks
        int leftTarget = leftDrive.getCurrentPosition();
        int rightTarget = rightDrive.getCurrentPosition();
        leftTarget += distance * TICKS_PER_INCH;
        rightTarget += distance * TICKS_PER_INCH;
        leftDrive.setTargetPosition(leftTarget);
        rightDrive.setTargetPosition(rightTarget);

        // Start the motors
        driveCmdRunning = true;
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
    }

    public void turnTo(float speed, int angle) {
        // Don't allow new moves if we're still busy
        if (isBusy()) {
            telemetry.log().add("driveTo(): Motors in use");
            return;
        }

        // Fake turns using a distance translation
        // We have a gyro but let's start with just one control mode
        int leftTarget = leftDrive.getCurrentPosition();
        int rightTarget = rightDrive.getCurrentPosition();
        leftTarget += angle * TURN_RATIO;
        rightTarget -= angle * TURN_RATIO;
        leftDrive.setTargetPosition(leftTarget);
        rightDrive.setTargetPosition(rightTarget);

        // Start the motors
        driveCmdRunning = true;
        leftDrive.setPower(speed);
        rightDrive.setPower(-speed);
    }
}