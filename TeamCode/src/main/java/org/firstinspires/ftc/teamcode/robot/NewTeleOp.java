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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name="NewTeleOp", group="Test")
public class NewTeleOp extends OpMode
{
    // Hardware
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor duckSpinner = null;
    private DcMotor depBelt = null;
    private Servo depLow = null;
    private Servo depMid = null;
    private DcMotor collector = null;

    // Constants used for hardware
    private static double DUCK_POWER = 0.5;
    private static double DEP_BELT_POWER = 0.5;
    private static double LOW_OPEN = 0.5;
    private static double LOW_CLOSE = 0.5;
    private static double MID_OPEN = 0.5;
    private static double MID_CLOSE = 0.5;
    private static double COLLECTOR_POWER = 0.5;

    // Members
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");

        // Drive Motors
        leftDrive  = hardwareMap.get(DcMotor.class, "BL");
        rightDrive = hardwareMap.get(DcMotor.class, "BR");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Duck Spinner
        duckSpinner = hardwareMap.get(DcMotor.class, "duck");

        // Depositor
        depBelt = hardwareMap.get(DcMotor.class, "Dbelt");
        depLow = hardwareMap.get(Servo.class, "Dlow");
        depMid = hardwareMap.get(Servo.class, "Dmid");

        // Collector
        collector = hardwareMap.get(DcMotor.class, "Collector");

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Ready");
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
    }

    @Override
    public void loop() {
        // PoV drive
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftDrive.setPower(Range.clip(drive + turn, -1.0, 1.0));
        rightDrive.setPower(Range.clip(drive - turn, -1.0, 1.0));

        // Duck spinner
        if (gamepad1.a) {
            duckSpinner.setPower(DUCK_POWER);
        } else {
            duckSpinner.setPower(0);
        }

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
        if (gamepad1.left_bumper) {
            collector.setPower(COLLECTOR_POWER);
        } else {
            collector.setPower(0);
        }

        // Feedback
        telemetry.addData("Drive", "L %.2f/%d, R %.2f/%d",
                leftDrive.getCurrentPosition(), leftDrive.getCurrentPosition(),
                rightDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
        telemetry.addData("Duck/Collector", "D %.2f, C (%.2f)",
                duckSpinner.getPower(), collector.getPower());
        telemetry.addData("Depositor", "B %.2f, L %.2f, M %.2f",
                depBelt.getPower(), depLow.getPosition(), depMid.getPosition());
    }

    @Override
    public void stop() {
    }
}