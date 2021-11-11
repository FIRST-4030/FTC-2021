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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp(name = "NewTeleOp", group = "Test")
public class NewTeleOp extends OpMode {
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
    //private Servo capstoneHook = null;
    private DistanceSensor distanceLeft;
    private DistanceSensor distanceRight;

    // Constants used for hardware
    private static double DUCK_POWER = 0.0;
    private static double DEP_BELT_POWER = 1;
    private static double DEP_DOWN = 0.44;
    private static double LOW_OPEN = 1;
    private static double LOW_CLOSE = 0.56;
    private static double MID_OPEN = 0.9;
    private static double MID_CLOSE = 0.49;
    private static double COLLECTOR_UP = 0.75;
    private static double COLLECTOR_DOWN = 0.0;
    private static double COLLECTOR_POWER = -1;
    private static double timerRatio = 0.0;
    private static double duckPowerMin = 0.65;  // min duck spinner speed (0 - 1.0)
    private static double duckPowerMax = 0.9;  // max duck spinner speed (0 - 1.0)
    private static double duckRampTime = 1.25;  // duck spinner ramp time (seconds, >0)
    private static double CAP_UP = 0.25;
    private static double CAP_MID = 0.5;
    private static double CAP_DOWN = 0.75;
    //private static double CAP_HOOK_DOWN = 0.75;
    //private static double CAP_HOOK_UP = 0.25;

    // Servo position test constants
    private float servoPos = 0.5f;
    private static final float INCREMENT = 0.01f;

    // Members
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime duckTimer = new ElapsedTime();

    @Override
    public void init() {
        boolean error = false;
        telemetry.addData("Status", "Initializing...");

        // Drive Motors
        try {
            leftDrive = hardwareMap.get(DcMotor.class, "BL");
            rightDrive = hardwareMap.get(DcMotor.class, "BR");
            leftDrive.setDirection(DcMotor.Direction.FORWARD);
            rightDrive.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception e) {
            telemetry.log().add("Could not find drive");
            error = true;
        }

        // Duck Spinner
        try {
            duckSpinner = hardwareMap.get(DcMotor.class, "duck");
        } catch (Exception e) {
            telemetry.log().add("Could not find duck spinner");
            error = true;
        }

        // Depositor
        try {
            depBelt = hardwareMap.get(DcMotor.class, "Depbelt");
            depLow = hardwareMap.get(Servo.class, "Deplow");
            depMid = hardwareMap.get(Servo.class, "Depmid");
            depTilt = hardwareMap.get(Servo.class, "Deptilt");
        } catch (Exception e) {
            telemetry.log().add("Could not find depositor");
            error = true;
        }


        // Collector
        try {
            collector = hardwareMap.get(DcMotor.class, "Collector");
            collectorArm = hardwareMap.get(Servo.class, "CollectorArm");
        } catch (Exception e) {
            telemetry.log().add("Could not find collector");
            error = true;
        }

        // Capstone Grabber
        try {
            //capstoneHook = hardwareMap.get(Servo.class, "Caphook");
            capstoneArm = hardwareMap.get(Servo.class, "Caparm");
        } catch (Exception e) {
            telemetry.log().add("Could not find collector");
            error = true;
        }

        // Distance Sensors
        try {
            distanceLeft = hardwareMap.get(DistanceSensor.class, "DL");
            distanceRight = hardwareMap.get(DistanceSensor.class, "DR");
        } catch (Exception e) {
            telemetry.log().add("Could not find collector");
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
        depTilt.setPosition(DEP_DOWN);
        depLow.setPosition(LOW_CLOSE);
        depMid.setPosition(MID_CLOSE);
        capstoneArm.setPosition(0.5);
        collectorArm.setPosition(COLLECTOR_DOWN);
    }

    @Override
    public void loop() {
        // PoV drive
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        leftDrive.setPower(Range.clip(drive + turn, -1.0, 1.0));
        rightDrive.setPower(Range.clip(drive - turn, -1.0, 1.0));

        // Duck spinner
        if (gamepad1.a) duckTimer.reset();
        timerRatio = Math.max(Math.min(duckTimer.seconds() / duckRampTime, 1.0), 0);
        if (timerRatio != 0.0 && timerRatio != 1.0) {
            DUCK_POWER = duckPowerMin + timerRatio * (duckPowerMax - duckPowerMin);
        } else {
            DUCK_POWER = 0.0;
        }
        duckSpinner.setPower(DUCK_POWER);

        // Depositor
        if (gamepad2.a || gamepad2.b || gamepad2.x) {
            depBelt.setPower(DEP_BELT_POWER);
        } else {
            depBelt.setPower(0);
        }
        if (gamepad2.a) {
            depLow.setPosition(LOW_OPEN);
        } else {
            depLow.setPosition(LOW_CLOSE);
        }
        if (gamepad2.b) {
            depMid.setPosition(MID_OPEN);
        } else {
            depMid.setPosition(MID_CLOSE);
        }

        // Collector
        if (gamepad1.right_bumper) {
            collector.setPower(COLLECTOR_POWER);
        } else {
            collector.setPower(0);
        }
        if (gamepad1.left_bumper) {
            collectorArm.setPosition(COLLECTOR_UP);
        } else {
            collectorArm.setPosition(COLLECTOR_DOWN);
        }

        // Capstone
        if (gamepad2.dpad_up) {
            capstoneArm.setPosition(CAP_UP);
        }
        if (gamepad2.left_bumper) {
            capstoneArm.setPosition(CAP_MID);
        }
        if (gamepad2.right_bumper) {
            capstoneArm.setPosition(CAP_DOWN);
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
        telemetry.addData("Depositor", "B %.2f, L %.2f, M %.2f",
                depBelt.getPower(), depLow.getPosition(), depMid.getPosition());
        /*
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
        depMid.setPosition(servoPos);*/
    }

    @Override
    public void stop() {
    }
}