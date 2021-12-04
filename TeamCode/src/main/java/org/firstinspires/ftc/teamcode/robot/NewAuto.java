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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.gamepad.GAMEPAD;
import org.firstinspires.ftc.teamcode.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.gamepad.PAD_KEY;
import org.firstinspires.ftc.teamcode.momm.MultiOpModeManager;
import org.firstinspires.ftc.teamcode.utils.OrderedEnum;
import org.firstinspires.ftc.teamcode.utils.OrderedEnumHelper;

@Config
@Autonomous(name = "NewAuto", group = "Test")
public class NewAuto extends MultiOpModeManager {
    // Hardware
    /* private DcMotor leftDrive = null;
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
    private DistanceSensor distanceRight = null; */
    private Drive drive;
    private DuckSpin duck;
    private Distance distance;
    private Depositor depositor;
    private Capstone capstone;

    // Consts
    private static float DRIVE_POWER = 0.4f;
    private static double TICKS_PER_INCH = 43.24;
    private static double TURN_RATIO = 6.375;
    /*
    private static double ANGLE_CONST = 1.23;
    private static double DUCK_POWER = 0.0;
    private static double DEP_BELT_POWER = 0.9;
    private static double DEP_UP = 0.6;
    private static double DEP_DOWN = 0.44;
    private static double LOW_OPEN = 0.98;
    private static double LOW_CLOSE = 0.56;
    private static double MID_OPEN = 0.9;
    private static double MID_CLOSE = 0.48;
    private static double COLLECTOR_UP = 0.96;
    private static double COLLECTOR_DOWN = 0.23;
    private static double COLLECTOR_POWER = -1;
    private static double timerRatio = 0.0;
    private static double duckPowerMin = 0.2;  // min duck spinner speed (0 - 1.0)
    private static double duckPowerMax = 0.45;  // max duck spinner speed (0 - 1.0)
    private static double duckRampTime = 1.25;  // duck spinner ramp time (seconds, >0)
    private static double CAP_IN = 0;
    private static double CAP_MID = 0.52;
    private static double CAP_UP = 0.35;
    private static double CAP_DOWN = 0.87; */

    // Members
    private ElapsedTime runtime = new ElapsedTime();
    private boolean driveCmdRunning = false;
    private AUTO_STATE state = AUTO_STATE.BARCODE;
    private boolean redAlliance = false;
    private boolean duckSide = false;
    private InputHandler in = null;

    @Override
    public void init() {
        boolean error = false;
        telemetry.addData("Status", "Initializing...");

        try {
            super.register(new Depositor());
            super.register(new Capstone());
            super.register(new Collector());

            drive = new Drive();
            super.register(drive);
            duck = new DuckSpin();
            super.register(duck);
            distance = new Distance();
            super.register(distance);
            depositor = new Depositor();
            super.register(depositor);
            capstone = new Capstone();
            super.register(capstone);

            super.init();

            in = Globals.input(this);
            in.register("ADVANCE_STATE", GAMEPAD.driver1, PAD_KEY.x);

        } catch (Exception e) {
            telemetry.log().add(String.valueOf(e));
            error = true;
        }

        /* // Drive Motors
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
            telemetry.log().add("Could not find capstone dep");
            error = true;
        }

        // Distance Sensors
        try {
            distanceLeft = hardwareMap.get(DistanceSensor.class, "DL");
            distanceRight = hardwareMap.get(DistanceSensor.class, "DR");
        } catch (Exception e) {
            telemetry.log().add("Could not find distance sensors");
            error = true;
        } */

        // Initialization status
        String status = "Ready";
        if (error) {
            status = "Hardware Error";
        }
        /*depLow.setPosition(LOW_CLOSE);
        depMid.setPosition(MID_CLOSE);
        capstoneArm.setPosition(CAP_IN);
        collectorArm.setPosition(COLLECTOR_UP); */
        telemetry.addData("Status", status);
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_right) redAlliance = true;
        if (gamepad1.dpad_left) redAlliance = false;
        if (gamepad1.dpad_up) duckSide = true;
        if (gamepad1.dpad_down) duckSide = false;
        telemetry.addData("Alliance", redAlliance ? "Red" : "Blue");
        telemetry.addData("Direction", duckSide ? "Duck" : "Warehouse");
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        distance.startScan();
    }

    @Override
    public void loop() {
        // Stop when the autoSteps are complete
        /* if (state == AUTO_STATE.DONE) {
            requestOpModeStop();
            return;
        } */

        // Feedback
        /*telemetry.addData("Drive", "L %.2f/%d, R %.2f/%d",
                leftDrive.getPower(), leftDrive.getCurrentPosition(),
                rightDrive.getPower(), rightDrive.getCurrentPosition());*/

        // Don't start new commands until the last one is complete
/*        if (driveCmdRunning) {
            // When the motors are done
            if (!isBusy()) {
                // Clear the running flag
                driveCmdRunning = false;
                // Advance to the next autoStep
                state = state.next();
            }
            // Continue from the top of loop()
            return;
        }
        */
        super.loop();

        in.loop();

        telemetry.addData("driveBusy: ", driveIsBusy());

        telemetry.addData("TURN_RATIO: ", TURN_RATIO);
        // Moving the servo position and number should increase
        if (gamepad1.dpad_up) {
            TURN_RATIO += 0.05;
            // Moving the servo position and number should decrease
        } else if (gamepad1.dpad_down) {
            TURN_RATIO -= 0.05;
        }

        //
        // Code past here is not run when driveCmdRunning is true
        //

        // Step through the auto commands
        switch (state) {
            /* // Forward 16
            case OUT_FROM_WALL:
                if (!duckSide) {
                    driveTo(DRIVE_POWER, 15f);
                } else {
                    driveTo(DRIVE_POWER, 24.5f);
                }
                //what the inline boolean op would look like
                //driveTo(DRIVE_POWER, !duckSide ? 15f : 24.5f);
                break;
            // Counter-clockwise 90
            case TURN_TO_PARKING:
                if ((redAlliance && !duckSide) || (!redAlliance && duckSide)) {
                    turnTo(DRIVE_POWER, -90);
                } else {
                    turnTo(DRIVE_POWER, 90);
                }

                //turnTo(DRIVE_POWER, (redAlliance && !duckSide) || (!redAlliance && duckSide) ? -90 : 90);
                break;
            // Backwards 32
            case PARK:
                if (!duckSide) {
                    driveTo(-DRIVE_POWER, -40);
                } else {
                    driveTo(-DRIVE_POWER, -24.8f);
                }
                break;
                //driveTo(-DRIVE_POWER, !duckSide ? -40f : -24.8f); */

            //new cases
            case BARCODE:
                if (distance.position() == Distance.BARCODE.LEFT) {
                    depositor.setDoor(Depositor.DOOR_USED.LOW_DOOR);
                } else if (distance.position() == Distance.BARCODE.CENTER) {
                    depositor.setDoor(Depositor.DOOR_USED.MID_DOOR);
                } else {
                    depositor.setDoor(Depositor.DOOR_USED.HIGH_DOOR);
                }
                if (!driveIsBusy() && in.down("ADVANCE_STATE")) {
                    state = state.next();
                }
                break;

            case ALIGN_TO_CAPSTONE:
                capstone.armDown();
                if (!duckSide) {
                    drive.driveTo(DRIVE_POWER, 15f);
                } else {
                    drive.driveTo(DRIVE_POWER, 24.5f);
                }
                if (!drive.isBusy()) {
                    state = state.next();
                }
                break;

            case PICK_UP_CAPSTONE:
                capstone.armUp();
                depositor.prep();
                if (!drive.isBusy() && in.down("ADVANCE_STATE") && depositor.isDone()) {
                    state = state.next();
                }
                break;

            case ALIGN_TO_HUB:
                if (!duckSide) {
                    drive.driveTo(DRIVE_POWER, 15f);
                } else {
                    drive.driveTo(DRIVE_POWER, 24.5f);
                }
                if (!drive.isBusy()) {
                    state = state.next();
                }
                break;

            case DEPOSIT:
                depositor.deposit();
                if (!driveIsBusy() && in.down("ADVANCE_STATE") && depositor.isDone()) {
                    state = state.next();
                }
                break;

            case ALIGN_TO_DUCK:
                //back up

                //turn

                //move to duck
                if (!driveIsBusy() && in.down("ADVANCE_STATE")) {
                    state = state.next();
                }
                break;

            case DUCK_SPIN:
                duck.auto(redAlliance);
                if (!driveIsBusy() && in.down("ADVANCE_STATE")) {
                    state = state.next();
                }
                break;

            case FINAL_PARK:
                //back up

                //small turn

                //park
                if (!driveIsBusy() && in.down("ADVANCE_STATE")) {
                    state = state.next();
                }
                break;
            // Stop processing
            case DONE:
                driveStop();
                break;
        }

        //log what state it currently is in
        telemetry.addData("Auto Step: ", state);
    }

    @Override
    public void stop() {
        super.stop();
    }

    enum AUTO_STATE implements OrderedEnum {
        OUT_FROM_WALL,
        TURN_TO_PARKING,
        PARK,
        BARCODE,
        ALIGN_TO_CAPSTONE,
        PICK_UP_CAPSTONE,
        ALIGN_TO_HUB,
        DEPOSIT,
        ALIGN_TO_DUCK,
        DUCK_SPIN,
        FINAL_PARK,
        DONE;

        public NewAuto.AUTO_STATE next() {
            return OrderedEnumHelper.next(this);
        }
    }

    public void driveStop() {
        // Zero the drive encoders, and enable RUN_TO_POSITION
        drive.driveStop();
    }

    public boolean driveIsBusy() {
        return drive.isBusy();
    }
}