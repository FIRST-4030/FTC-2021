/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.momm.MultiOpModeManager;

@Config
@TeleOp(name = "NewTeleOp", group = "Test")
public class NewTeleOp extends MultiOpModeManager {
    // Hardware
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor duckSpinner = null;
    private DcMotor duckSpinner2 = null;
    private DcMotor depBelt = null;
    private Servo depLow = null;
    private Servo depMid = null;
    private Servo depHigh = null;
    private Servo depTilt = null;
    private DcMotor collector = null;
    private Servo collectorArm = null;
    private Servo capstoneArm = null;
    private DistanceSensor distanceLeft = null;
    private DistanceSensor distanceRight = null;
    private TouchSensor sensorCollector = null;
    private Depositor depositor;

    // Constants used for hardware
    private static double DUCK_POWER = 0.0;
    private static double DEP_BELT_POWER = 0.88;
    private static double DEP_UP = 0.47;
    private static double DEP_DOWN = 0.18;
    private static double LOW_OPEN = 0.98;
    private static double LOW_CLOSE = 0.56;
    private static double MID_OPEN = 0.9;
    private static double MID_CLOSE = 0.47;
    private static double HIGH_OPEN = 0.13;
    private static double HIGH_INIT = 0.55;
    public static double COLLECTOR_UP = 0.65;
    public static double COLLECTOR_DOWN = 0.90;
    private static double SPEED = 1;
    public static int DISTANCE = 30;
    public static double EJECT_TIME = 3;
    private static double timerRatio = 0.0;
    public static double duckPowerMin = 0.63;  // min duck spinner speed (0 - 1.0)
    public static double duckPowerMax = 0.88;  // max duck spinner speed (0 - 1.0)
    public static double duckRampTime = 1.4;  // duck spinner ramp time (seconds, >0)
    public static double CAP_IN = 0;
    //private static double CAP_UP = 0.35;
    public static double CAP_MID = 0.5;
    public static double CAP_DOWN = 0.87;
    private static double CAPSTONE_DELTA = 0.01;
    private static double delayTime = 0.1;
    private static double capstoneTarget = 0;
    private double LEFT_DRIVE_POW = 0;
    private double RIGHT_DRIVE_POW = 0;
    private static double ACCEL_CONSTANT = 0.4;
    private static double fastFactor = 0;
    private static double duckPowerAutoMin = 0.46;
    private static double duckPowerAutoMax = 0.66;
    private static double autoDuckRampTime = 1.65;

    private static boolean collectorActive = false;

    // Servo position test constants
    private float servoPos = 0.4f;
    private static final float INCREMENT = 0.01f;

    // Members
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime duckTimer = new ElapsedTime();
    private ElapsedTime capTimer = new ElapsedTime();
    private ElapsedTime collectorTimer = new ElapsedTime(0);

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

        // Duck Spinner
        try {
            duckSpinner = hardwareMap.get(DcMotor.class, "duck");
            duckSpinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //duckSpinner2 = hardwareMap.get(DcMotor.class, "duck2");
            //duckSpinner2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            telemetry.log().add("Could not find duck spinner");
            error = true;
        }

        // Depositor
        try {
            /* depBelt = hardwareMap.get(DcMotor.class, "Depbelt");
            depBelt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            depLow = hardwareMap.get(Servo.class, "Deplow");
            depMid = hardwareMap.get(Servo.class, "Depmid");
            depHigh = hardwareMap.get(Servo.class, "Dephigh");
            depTilt = hardwareMap.get(Servo.class, "Deptilt");*/
            super.register(new Depositor());
            depositor = new Depositor();
            super.register(depositor);
            depositor.init();
        } catch (Exception e) {
            telemetry.log().add("Could not find depositor");
            error = true;
        }


        // Collector
        try {
            /* super.register(new Collector());
            collector = new Collector();
            super.register(collector);

            super.init(); */
            collector = hardwareMap.get(DcMotor.class, "Collector");
            collectorArm = hardwareMap.get(Servo.class, "CollectorArm");
            sensorCollector = hardwareMap.get(TouchSensor.class, "DC");
        } catch (Exception e) {
            telemetry.log().add("Could not find collector");
            error = true;
        }

        // Capstone Grabber
        try {
            //capstoneHook = hardwareMap.get(Servo.class, "Caphook");
            capstoneArm = hardwareMap.get(Servo.class, "Caparm");
        } catch (Exception e) {
            telemetry.log().add("Could not find capstone dep");
            error = true;
        }

        // Distance Sensors
        try {
            distanceLeft = hardwareMap.get(DistanceSensor.class, "DL");
            distanceRight = hardwareMap.get(DistanceSensor.class, "DR");
        } catch (Exception e) {
            telemetry.log().add("Could not find range sensors");
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
        runtime.reset();

        // Zero the drive encoders and enable RUN_USING_ENCODER for velocity PID
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collectorArm.setPosition(COLLECTOR_UP);
        capstoneArm.setPosition(CAP_MID);
        super.start();
    }

    @Override
    public void loop() {
        // PoV drive
        if (gamepad1.right_trigger > ACCEL_CONSTANT) {
            fastFactor = gamepad1.right_trigger;
        } else {
            fastFactor = ACCEL_CONSTANT;
        }
        LEFT_DRIVE_POW = Math.pow(-gamepad1.left_stick_y, 1);
        RIGHT_DRIVE_POW = Math.pow(-gamepad1.right_stick_y, 1);
        leftDrive.setPower(LEFT_DRIVE_POW * fastFactor);
        rightDrive.setPower(RIGHT_DRIVE_POW * fastFactor);

        // Duck spinner
        if (gamepad1.a) {
            DUCK_POWER = duckPowerMin;
            duckTimer.reset();
        } else if (gamepad1.b) {
            DUCK_POWER = -duckPowerMin;
            duckTimer.reset();
        }

        if (DUCK_POWER != 0 && duckTimer.seconds() < duckRampTime) {
            DUCK_POWER = (duckPowerMin + (duckTimer.seconds() / duckRampTime) *
                    (duckPowerMax - duckPowerMin)) * Math.signum(DUCK_POWER);
        } else {
            DUCK_POWER = 0.0;
        }
        duckSpinner.setPower(DUCK_POWER);
        //duckSpinner2.setPower(DUCK_POWER);

        // Depositor
        depositor.loop();

        // Collector
        // Sensor
        boolean collected = sensorCollector.isPressed();
        telemetry.addData("Collected? ", collected);
        /* if (gamepad2.left_bumper) {
            collectorActive = true;
        }

        if (collectorActive) {
            if ((collected || !gamepad2.left_bumper) && collectorArm.getPosition() == COLLECTOR_DOWN) {
                collectorTimer.reset();
            }
            if (collectorTimer.seconds() > EJECT_TIME) {
                collectorActive = false;
                collector.setPower(-1);
            } else {
                collector.setPower(1);
            }
        } else {
            collector.setPower(0);
        }
        collectorArm.setPosition(collectorActive ? COLLECTOR_DOWN : COLLECTOR_UP);*/
        if ((collected || gamepad2.left_bumper) && collectorArm.getPosition() == COLLECTOR_DOWN) {
            collectorTimer.reset();
        }
        if (gamepad2.left_bumper && collectorArm.getPosition() == COLLECTOR_UP) {
            collectorArm.setPosition(COLLECTOR_DOWN);
            collector.setPower(1);
        } else if (collectorTimer.seconds() < 0.24) {
            collectorArm.setPosition(COLLECTOR_DOWN);
            collector.setPower(1);
        } else if (collectorTimer.seconds() < EJECT_TIME) {
            collectorArm.setPosition(COLLECTOR_UP);
            collector.setPower(-1);
        } else {
            collectorArm.setPosition(COLLECTOR_UP);
            collector.setPower(0);
        }
        RobotLog.d("," + "Collector" + "," + getRuntime() + "," +
                gamepad2.left_bumper + "," + collected + "," +
                collectorArm.getPosition() + "," + collector.getPower() + "," +
                collectorTimer.seconds());

        // Capstone
        if (gamepad2.dpad_down) {
            capstoneTarget = CAP_DOWN;
        } else if (gamepad2.dpad_up) {
            capstoneTarget = CAP_MID;
        }
 /*       if (capstoneTarget >= 0 && capstoneTarget <= 1) {
            capstoneTarget += (gamepad2.right_stick_y * 0.02);
        } */

        double capError = capstoneTarget - capstoneArm.getPosition();
        if (capError != 0 && capTimer.seconds() > delayTime) {
            double delta = Math.max(CAPSTONE_DELTA, Math.abs(capError));
            delta *= Math.signum(capError);
            capstoneArm.setPosition(capstoneArm.getPosition() + delta);
            capTimer.reset();
        }

        // Distance Sensor
        if (distanceLeft.getDistance(DistanceUnit.INCH) <= 19) {
            telemetry.addData("In Range (Left): ", "Yes");
        } else {
            telemetry.addData("In Range (Left): ", "No");
        }
        if (distanceRight.getDistance(DistanceUnit.INCH) <= 19) {
            telemetry.addData("In Range (Right): ", "Yes");
        } else {
            telemetry.addData("In Range (Right): ", "No");
        }
        telemetry.addData("Left Distance", String.format("%.01f in", distanceLeft.getDistance(DistanceUnit.INCH)));
        telemetry.addData("Right Distance", String.format("%.01f in", distanceRight.getDistance(DistanceUnit.INCH)));

        // Feedback
        telemetry.addData("Drive", "L %.2f/%d, R %.2f/%d",
                leftDrive.getPower(), leftDrive.getCurrentPosition(),
                rightDrive.getPower(), rightDrive.getCurrentPosition());
        telemetry.addData("Duck/Collector", "D %.2f, C (%.2f)",
                duckSpinner.getPower(), collector.getPower());
        /* telemetry.addData("Depositor", "B %.2f, L %.2f, M %.2f",
                depBelt.getPower(), depLow.getPosition(), depMid.getPosition());
        telemetry.addData("Spin", spin);
        telemetry.addData("Dep Belt Pos: ", depBelt.getCurrentPosition()); */

        // Shows number of servoPos
        telemetry.addData("Pos:", servoPos);
        // Moving the servo position and number should increase
        if (gamepad1.dpad_up) {
            servoPos += INCREMENT;
            servoPos = Math.min(1.0f, servoPos);
            // Moving the servo position and number should decrease
        } else if (gamepad1.dpad_down) {
            servoPos -= INCREMENT;
            servoPos = Math.max(0.0f, servoPos);
        }
        // Set position of desired servo
        //collectorArm.setPosition(servoPos);
    }

    @Override
    public void stop() {
    }
}