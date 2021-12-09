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

import org.firstinspires.ftc.teamcode.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.momm.MultiOpModeManager;
import org.firstinspires.ftc.teamcode.utils.OrderedEnum;
import org.firstinspires.ftc.teamcode.utils.OrderedEnumHelper;

@Config
@Autonomous(name = "autoTest", group = "Test")
public class autoTest extends MultiOpModeManager {
    // Hardware
    private Drive drive;
    private Distance distance;
    private Depositor depositor;

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
            super.register(new Drive());
            super.register(new Distance());

            drive = new Drive();
            super.register(drive);
            distance = new Distance();
            super.register(distance);
            depositor = new Depositor();
            super.register(depositor);

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

        distance.loop();
        depositor.loop();

        in.loop();

        telemetry.addData("TURN_RATIO: ", TURN_RATIO);
        telemetry.addData("LDrive Pos: ", drive.leftDrivePos());
        telemetry.addData("RDrive Pos: ", drive.rightDrivePos());

        // Step through the auto commands
        switch (state) {
            // new routine
            case BARCODE:
                if (num == 0) {
                    distance.startScan();
                    num++;
                }
                if (distance.position() == Distance.BARCODE.LEFT) {
                    depositor.setDoor(Depositor.DOOR_USED.LOW_DOOR);
                } else if (distance.position() == Distance.BARCODE.CENTER) {
                    depositor.setDoor(Depositor.DOOR_USED.MID_DOOR);
                } else {
                    depositor.setDoor(Depositor.DOOR_USED.HIGH_DOOR);
                }
                if (!drive.isBusy() && distance.state() != Distance.AUTO_STATE.DONE && depositor.doorUsed() != Depositor.DOOR_USED.NONE) {
                    state = state.next();
                }
                break;

            case DEPOSIT:
                depositor.deposit();
                if (!drive.isBusy() && depositor.isDone()) {
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
        DEPOSIT,
        DONE;

        public autoTest.AUTO_STATE next() {
            return OrderedEnumHelper.next(this);
        }
    }

    public void driveStop() {
        // Zero the drive encoders, and enable RUN_TO_POSITION
        drive.driveStop();
    }
}