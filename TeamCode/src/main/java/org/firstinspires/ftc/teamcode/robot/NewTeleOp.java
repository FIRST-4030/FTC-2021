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
    private DcMotor DepBelt = null;
    private Servo DepLow = null;
    private Servo DepMid = null;
    private DcMotor Collector = null;

    // Constants used for hardware
    private static double duckPower = 0.5;
    private static double beltPower = 0.5;
    private static double LOW_OPEN = 0.5;
    private static double LOW_CLOSE = 0.5;
    private static double MID_OPEN = 0.5;
    private static double MID_CLOSE = 0.5;
    private static double collectorPower = 0.5;

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
        DepBelt = hardwareMap.get(DcMotor.class, "Dbelt");
        DepLow = hardwareMap.get(Servo.class, "Dlow");
        DepMid = hardwareMap.get(Servo.class, "Dmid");

        // Collector
        Collector = hardwareMap.get(DcMotor.class, "Collector");

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
            duckSpinner.setPower(duckPower);
        } else {
            duckSpinner.setPower(0);
        }

        // Depositor
        if (gamepad2.a || gamepad2.b || gamepad2.x) {
            DepBelt.setPower(beltPower);
        } else {
            DepBelt.setPower(0);
        }
        if (gamepad2.a) {
            DepLow.setPosition(LOW_OPEN);
        } else {
            DepLow.setPosition(LOW_CLOSE);
        }
        if (gamepad2.b) {
            DepMid.setPosition(MID_OPEN);
        } else {
            DepMid.setPosition(MID_CLOSE);
        }

        // Collector
        if (gamepad1.left_bumper) {
            Collector.setPower(collectorPower);
        } else {
            Collector.setPower(0);
        }

        // Feedback
        telemetry.addData("Drive", "L %.2f/%d, R %.2f/%d",
                leftDrive.getCurrentPosition(), leftDrive.getCurrentPosition(),
                rightDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
        telemetry.addData("Duck/Collector", "D %.2f, C (%.2f)",
                duckSpinner.getPower(), Collector.getPower());
        telemetry.addData("Depositor", "B %.2f, L %.2f, M %.2f",
                DepBelt.getPower(), DepLow.getPosition(), DepMid.getPosition());
    }

    @Override
    public void stop() {
    }
}