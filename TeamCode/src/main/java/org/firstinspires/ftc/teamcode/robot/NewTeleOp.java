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
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.momm.MultiOpModeManager;

@Config
@TeleOp(name = "NewTeleOp", group = "Test")
public class NewTeleOp extends MultiOpModeManager {
    // Hardware
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor duckSpinner = null;
    private DcMotor duckSpinner2 = null;
    private DcMotor collector = null;
    private Servo collectorArm = null;
    private Servo capstoneArm = null;
    private DistanceSensor distanceLeft = null;
    private DistanceSensor distanceRight = null;
    private TouchSensor sensorCollector = null;
    private Depositor depositor;
    private DuckSpin duckSpin;

    // Constants used for hardware
    private static double DUCK_POWER = 0.0;
    public static double COLLECTOR_UP = 0.53;
    public static double COLLECTOR_DOWN = 0.90;
    private static double SPEED = 1;
    public static int DISTANCE = 30;
    public static double DELAY_TIME = 1.75;
    public static double EJECT_TIME = 2;
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
    private static double fastFactor2 = 0;
    collectCmd collectCmdState = collectCmd.IDLE;

    private static boolean collectorActive = false;
    private static boolean collected = false;

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
            /*duckSpinner = hardwareMap.get(DcMotor.class, "duck");
            duckSpinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            duckSpinner2 = hardwareMap.get(DcMotor.class, "duck2");
            duckSpinner2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
            super.register(new DuckSpin());
            duckSpin = new DuckSpin();
            super.register(duckSpin);
            duckSpin.init();
        } catch (Exception e) {
            telemetry.log().add("Could not find duck spinner");
            error = true;
        }

        // Depositor
        try {
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
        if (gamepad1.left_trigger > ACCEL_CONSTANT) {
            fastFactor = gamepad1.left_trigger;
        } else {
            fastFactor = ACCEL_CONSTANT;
        }
        if (gamepad1.right_trigger > ACCEL_CONSTANT) {
            fastFactor2 = gamepad1.right_trigger;
        } else {
            fastFactor2 = ACCEL_CONSTANT;
        }
        LEFT_DRIVE_POW = Math.pow(-gamepad1.left_stick_y, 1);
        RIGHT_DRIVE_POW = Math.pow(-gamepad1.right_stick_y, 1);
        leftDrive.setPower(LEFT_DRIVE_POW * fastFactor);
        rightDrive.setPower(RIGHT_DRIVE_POW * fastFactor2);

        // Duck spinner
        /* if (gamepad1.a) {
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
        duckSpinner2.setPower(DUCK_POWER); */
        duckSpin.loop();

        // Depositor
        depositor.loop();
        if (!depositor.isDone() && depositor.state == Depositor.AUTO_STATE.DOOR_PREP && collectCmdState == collectCmd.IDLE) {
            collectCmdState = collectCmd.BEFORE_EJECT;
        }

        // Collector state
        if (!collected) {
            collected = sensorCollector.isPressed();
        } else if (collectCmdState != collectCmd.COLLECT || collectorTimer.seconds() <= (Math.PI / 10)) {
            collected = sensorCollector.isPressed();
        }
        telemetry.addData("Collected? ", collected);
        switch (collectCmdState) {
            case IDLE:
                if (gamepad2.left_bumper) {
                    collectCmdState = collectCmd.BEFORE_COLLECT;
                }
                break;
            case BEFORE_COLLECT:
                collectorTimer.reset();
                collectCmdState = collectCmd.COLLECT;
                break;
            case COLLECT:
                if (!gamepad2.left_bumper) {
                    collectCmdState = collectCmd.BEFORE_EJECT;
                }
                if (collected && collectorTimer.seconds() > (Math.PI / 10)) {
                    collectorTimer.reset();
                    collectCmdState = collectCmd.SENSOR_DELAY;
                }
                break;
            case SENSOR_DELAY:
                collected = false;
                if (collectorTimer.seconds() > DELAY_TIME) {
                    collectCmdState = collectCmd.BEFORE_EJECT;
                }
                break;
            case BEFORE_EJECT:
                collectorTimer.reset();
                collectCmdState = collectCmd.EJECT;
                break;
            case EJECT:
                if (collectorTimer.seconds() > EJECT_TIME) {
                    collectCmdState = collectCmd.IDLE;
                }
                if (gamepad2.right_bumper) {
                    collectCmdState = collectCmd.IDLE;
                }
                break;
        }

        // Collector commands
        switch (collectCmdState) {
            case IDLE:
                collectorArm.setPosition(COLLECTOR_UP);
                collector.setPower(0);
                break;
            case BEFORE_COLLECT:
            case COLLECT:
            case SENSOR_DELAY:
                collectorArm.setPosition(COLLECTOR_DOWN);
                collector.setPower(1);
                break;
            case BEFORE_EJECT:
                collectorArm.setPosition(COLLECTOR_UP);
                collector.setPower(1);
                break;
            case EJECT:
                collectorArm.setPosition(COLLECTOR_UP);
                collector.setPower(-1);
                break;
        }
    }

    @Override
    public void stop() {
    }

    enum collectCmd {
        IDLE,
        BEFORE_COLLECT,
        COLLECT,
        SENSOR_DELAY,
        BEFORE_EJECT,
        EJECT
    }
}