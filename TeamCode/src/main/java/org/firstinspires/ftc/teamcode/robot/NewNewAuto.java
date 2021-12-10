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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.gamepad.GAMEPAD;
import org.firstinspires.ftc.teamcode.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.gamepad.PAD_KEY;
import org.firstinspires.ftc.teamcode.momm.MultiOpModeManager;
import org.firstinspires.ftc.teamcode.utils.OrderedEnum;
import org.firstinspires.ftc.teamcode.utils.OrderedEnumHelper;

@Config
@Autonomous(name = "NewNewAuto", group = "Test")
public class NewNewAuto extends MultiOpModeManager {
    // Hardware
    private Drive drive;
    private DuckSpin duck;
    private Distance distance;
    private Depositor depositor;
    private Capstone capstone;
    private Collector collector;

    // Constants
    private static float DRIVE_POWER = 0.4f;
    private static double TURN_RATIO = 6.375;
    private int num = 0;

    // Members
    private ElapsedTime runtime = new ElapsedTime();
    private AUTO_STATE state = AUTO_STATE.DONE;
    private boolean redAlliance = false;
    private boolean duckSide = false;
    private InputHandler in;

    @Override
    public void init() {
        boolean error = false;
        telemetry.addData("Status", "Initializing...");

        try {
            super.register(new Depositor());
            super.register(new Capstone());
            super.register(new Drive());
            super.register(new DuckSpin());
            super.register(new Distance());

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

            in = Globals.input(this);

            distance.startScan();

            super.init();
        } catch (Exception e) {
            telemetry.log().add(String.valueOf(e));
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
        num = 0;
        distance.startScan();
        driveStop();
        state = AUTO_STATE.BARCODE;
    }

    @Override
    public void loop() {
        // Stop when the autoSteps are complete
        if (state == AUTO_STATE.DONE) {
            requestOpModeStop();
            return;
        }

        depositor.loop();

        in.loop();

        telemetry.addData("TURN_RATIO: ", TURN_RATIO);
        telemetry.addData("LDrive Pos: ", drive.leftDrivePos());
        telemetry.addData("RDrive Pos: ", drive.rightDrivePos());

        // Step through the auto commands
        switch (state) {
            // new routine
            case BARCODE:
                distance.startScan();
                if (distance.position() == Distance.BARCODE.LEFT) {
                    depositor.setDoor(Depositor.DOOR_USED.LOW_DOOR);
                } else if (distance.position() == Distance.BARCODE.CENTER) {
                    depositor.setDoor(Depositor.DOOR_USED.MID_DOOR);
                } else {
                    depositor.setDoor(Depositor.DOOR_USED.HIGH_DOOR);
                }
                if (!drive.isBusy() && distance.state() == Distance.AUTO_STATE.DONE && distance.position() != Distance.BARCODE.NONE) {
                    state = state.next();
                }
                break;

            case OUT_FROM_WALL:
                if (num == 0) {
                    if (!duckSide) {
                        drive.driveTo(DRIVE_POWER, 15f);
                    } else {
                        drive.driveTo(DRIVE_POWER, 24.5f);
                    }
                    num++;
                }
                telemetry.addData("Times driveTo ran: ", num);
                if (!drive.isBusy()) {
                    state = state.next();
                }
                break;

            /* case ALIGN_TO_CAPSTONE:
                capstone.armDown();
                if (capstone.isDone()) {
                    state = state.next();
                }
                break;

            case PICK_UP_CAPSTONE:
                capstone.armUp();
                depositor.prep();
                if (!drive.isBusy() && capstone.isDone() && depositor.isDone()) {
                    state = state.next();
                }
                break; */

            case TURN_TO_HUB:
                if (num == 1) {
                    if (duckSide) {
                        if (redAlliance) {
                            drive.turnTo(DRIVE_POWER, 64);
                        } else {
                            drive.turnTo(DRIVE_POWER, -64);
                        }
                    } else {
                        if (redAlliance) {
                            drive.turnTo(DRIVE_POWER, -47);
                        } else {
                            drive.turnTo(DRIVE_POWER, 47);
                        }
                    }
                    num++;
                }
                if (!drive.isBusy()) {
                    state = state.next();
                }
                break;

            case ALIGN_TO_HUB:
                if (num == 2) {
                    if (!duckSide) {
                        drive.driveTo(DRIVE_POWER, 10.8f);
                    } else {
                        drive.driveTo(DRIVE_POWER, 7.6f);
                    }
                    num++;
                }
                if (!drive.isBusy()) {
                    state = state.next();
                }
                break;

            case DEPOSIT:
                depositor.deposit();
                if (!drive.isBusy() && depositor.isDone()) {
                    state = state.next();
                }
                break;

            case BACK_UP:
                if (num == 3) {
                    if (!duckSide) {
                        drive.driveTo(-DRIVE_POWER, -10.8f);
                    } else {
                        drive.driveTo(-DRIVE_POWER, -7.6f);
                    }
                    num++;
                }
                if (!drive.isBusy()) {
                    state = state.next();
                }
                break;

            case TURN_TO_PARK:
                if (num == 4) {
                    if (duckSide) {
                        if (redAlliance) {
                            drive.turnTo(DRIVE_POWER, 26);
                        } else {
                            drive.turnTo(DRIVE_POWER, -26);
                        }
                    } else {
                        if (redAlliance) {
                            drive.turnTo(DRIVE_POWER, -43);
                        } else {
                            drive.turnTo(DRIVE_POWER, 43);
                        }
                    }
                    num++;
                }
                if (!drive.isBusy()) {
                    state = state.next();
                }
                break;

            case PARK:
                if (num == 5) {
                    if (duckSide) {
                        drive.driveTo(-DRIVE_POWER, -24.5f);
                    } else {
                        drive.driveTo(-DRIVE_POWER, -40);
                    }
                    num++;
                }
                if (!drive.isBusy()) {
                    state = state.next();
                }
                break;

            case TURN_TO_DUCK:
                if (num == 6) {
                    if (duckSide) {
                        if (redAlliance) {
                            drive.turnTo(DRIVE_POWER, 90);
                        } else {
                            drive.turnTo(DRIVE_POWER, -90);
                        }
                    }
                    num++;
                }
                if (!drive.isBusy()) {
                    state = state.next();
                }
                break;

            case ALIGN_TO_DUCK:
                if (num == 7) {
                    if (duckSide) {
                        drive.driveTo(DRIVE_POWER, 23.1f);
                    }
                    num++;
                }
                if (!drive.isBusy()) {
                    state = state.next();
                }
                break;

            case DUCK_SPIN:
                if (duckSide) {
                    duck.auto(redAlliance);
                }
                if (!drive.isBusy() && duck.isDone()) {
                    state = state.next();
                }
                break;

            case PARK_FROM_DUCK:
                if (num == 8) {
                    if (duckSide) {
                        drive.driveTo(-DRIVE_POWER, -23.1f);
                    }
                    num++;
                }
                if (!drive.isBusy()) {
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
        BARCODE,
        OUT_FROM_WALL,
        /* ALIGN_TO_CAPSTONE,
        PICK_UP_CAPSTONE,*/
        TURN_TO_HUB,
        ALIGN_TO_HUB,
        DEPOSIT,
        BACK_UP,
        TURN_TO_PARK,
        PARK,
        TURN_TO_DUCK,
        ALIGN_TO_DUCK,
        DUCK_SPIN,
        PARK_FROM_DUCK,
        DONE;

        public NewNewAuto.AUTO_STATE next() {
            return OrderedEnumHelper.next(this);
        }
    }

    public void driveStop() {
        // Zero the drive encoders, and enable RUN_TO_POSITION
        drive.driveStop();
    }
}