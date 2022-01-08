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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.momm.MultiOpModeManager;
import org.firstinspires.ftc.teamcode.roadrunner.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.OrderedEnum;
import org.firstinspires.ftc.teamcode.utils.OrderedEnumHelper;

@Config
//@Disabled
@Autonomous(name = "DriveTest", group = "Test")
public class DriveTest extends MultiOpModeManager {
    // Hardware
    private NewNewDrive drive;
    private Servo collectorArm = null;
    private Distance distance;
    private Depositor depositor;
    private Capstone capstone;

    // Constants
    public static double speedMin = 0.2;
    public static double speedMax = 0.4;
    public static double r = 16;
    public static double arcLength = 18;
    private static double angle = arcLength / Math.PI * 180.0 / r;
    public static double COLLECTOR_UP = 0.65;
    public static double COLLECTOR_DOWN = 0.90;

    // Members
    private AUTO_STATE state = AUTO_STATE.DONE;
    private InputHandler in;
    private boolean redAlliance = false;
    private boolean duckSide = false;

    @Override
    public void init() {
        boolean error = false;
        telemetry.addData("Status", "Initializing...");

        try {
            super.register(new Depositor());
            super.register(new Capstone());
            super.register(new Distance());

            distance = new Distance();
            super.register(distance);
            depositor = new Depositor();
            super.register(depositor);
            capstone = new Capstone();
            super.register(capstone);

            in = Globals.input(this);

            distance.startScan();

            super.init();
        } catch (Exception e) {
            telemetry.log().add(String.valueOf(e));
            error = true;
        }

        try {
            super.register(new NewNewDrive());

            drive = new NewNewDrive();
            super.register(drive);

            in = Globals.input(this);

            super.init();
        } catch (Exception e) {
            telemetry.log().add(String.valueOf(e));
            error = true;
        }

        try {
            collectorArm = hardwareMap.get(Servo.class, "CollectorArm");
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
        drive.enableLogging();
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
        drive.setDoneFalse();
        state = AUTO_STATE.MOVE_OUT;
    }

    @Override
    public void loop() {
        depositor.loop();
        distance.loop();
        in.loop();

        // Step through the auto commands
        switch (state) {
            case BARCODE:
                if (distance.state() == Distance.AUTO_STATE.DONE) {
                    if (distance.position() == Distance.BARCODE.LEFT) {
                        depositor.setDoor(Depositor.DOOR_USED.LOW_DOOR);
                    } else if (distance.position() == Distance.BARCODE.CENTER) {
                        depositor.setDoor(Depositor.DOOR_USED.MID_DOOR);
                    } else if (distance.position() == Distance.BARCODE.RIGHT) {
                        depositor.setDoor(Depositor.DOOR_USED.HIGH_DOOR);
                    } else {
                        if (!duckSide) {
                            if (redAlliance) {
                                depositor.setDoor(Depositor.DOOR_USED.LOW_DOOR);
                            } else {
                                depositor.setDoor(Depositor.DOOR_USED.HIGH_DOOR);
                            }
                        }
                    }
                    state = state.next();
                }
                break;

            case MOVE_OUT:
                depositor.prep();
                drive.driveTo(speedMin, speedMax, 14);
                collectorArm.setPosition(COLLECTOR_UP);
                if (drive.isDone() && !drive.isBusy()) {
                    drive.setDoneFalse();
                    state = state.next();
                }
                break;
            case ARC:
                drive.arcToTicks(angle, r, 0, speedMin, speedMax);
                if (drive.isDone() && !drive.isBusy() && depositor.isDone()) {
                    drive.setDoneFalse();
                    state = state.next();
                }
                break;
            case DEPOSIT:
                depositor.deposit();
                if (depositor.isDone()) {
                    state = state.next();
                }
                break;
            case PARK:
                drive.driveTo(-speedMin, -speedMax, -52);
                if (drive.isDone() && !drive.isBusy()) {
                    drive.setDoneFalse();
                    state = state.next();
                }
                break;
            // Stop processing
            case DONE:
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
        BARCODE,
        MOVE_OUT,
        ARC,
        DEPOSIT,
        PARK,
        DONE;

        public DriveTest.AUTO_STATE next() {
            return OrderedEnumHelper.next(this);
        }
    }
}