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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.momm.MultiOpModeManager;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.OrderedEnum;
import org.firstinspires.ftc.teamcode.utils.OrderedEnumHelper;

@Config
//@Disabled
@Autonomous(name = "DriveTest", group = "Test")
public class DriveTest extends MultiOpModeManager {
    // Hardware
    private NewNewDrive drive;

    // Constants
    private static float DRIVE_POWER = 0.5f;

    // Members
    private AUTO_STATE state = AUTO_STATE.DONE;
    private InputHandler in;

    @Override
    public void init() {
        boolean error = false;
        telemetry.addData("Status", "Initializing...");

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

        // Initialization status
        String status = "Ready";
        if (error) {
            status = "Hardware Error";
        }
        telemetry.addData("Status", status);
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        state = AUTO_STATE.ARC;
    }

    @Override
    public void loop() {
        in.loop();

        // Step through the auto commands
        switch (state) {
            case ARC:
                drive.arcToTicks(50, 35.625, 0.2, 0.4);
                if (drive.isDone()) {
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
        ARC,
        DONE;

        public DriveTest.AUTO_STATE next() {
            return OrderedEnumHelper.next(this);
        }
    }

    public void driveStop() {
        // Zero the drive encoders, and enable RUN_TO_POSITION
        drive.driveStop();
    }
}