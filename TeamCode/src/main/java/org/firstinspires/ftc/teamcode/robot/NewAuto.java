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

@Config
@TeleOp(name = "NewAuto", group = "Test")
public class NewAuto extends OpMode {
    // Hardware
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    // Consts
    private static float DRIVE_POWER = 0.77f;
    private static double TICKS_PER_INCH = 123.45;
    private static double ANGLE_CONST = 1.23;

    // Members
    private boolean driveCmdRunning = false;
    private int autoStep = 0;

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
        driveStop();
    }

    @Override
    public void loop() {

        // Feedback
        telemetry.addData("Drive", "L %.2f/%d, R %.2f/%d",
                leftDrive.getPower(), leftDrive.getCurrentPosition(),
                rightDrive.getPower(), rightDrive.getCurrentPosition());

        // Don't start new commands until the last one is complete
        if (driveCmdRunning) {
            // When the motors are done
            if (!isBusy()) {
                // Clear the running flag
                driveCmdRunning = false;
                // Advance to the next autoStep
                autoStep++;
            }
            // Continue from the top of loop()
            return;
        }

        //
        // Code past here is not run when driveCmdRunning is true
        //

        // Step through the auto commands
        switch (autoStep) {
            // Forward 19
            case 0:
                driveTo(DRIVE_POWER, 19);
                break;
            // Counter-clockwise 135
            case 1:
                turnTo(DRIVE_POWER, -135);
                break;
            // Forward 23
            case 2:
                driveTo(DRIVE_POWER, 23);
                break;
            // Backward 35
            case 3:
                driveTo(DRIVE_POWER, 35);
                break;
            // Forward 3
            case 4:
                driveTo(DRIVE_POWER, 3);
                break;
            // Clockwise 45
            case 5:
                turnTo(DRIVE_POWER, 45);
                break;
            // Forward 31
            case 6:
                driveTo(DRIVE_POWER, 31);
                break;
        }
    }

    @Override
    public void stop() {
    }

    public void driveStop() {
        // Stop, zero the drive encoders, and enable RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setTargetPosition(leftDrive.getTargetPosition());
        leftDrive.setPower(0);

        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setTargetPosition(rightDrive.getTargetPosition());
        rightDrive.setPower(0);
    }

    public boolean isBusy() {
        return leftDrive.isBusy() || rightDrive.isBusy();
    }

    public void driveTo(float speed, int distance) {
        // Don't allow new moves if we're still busy
        if (isBusy()) {
            telemetry.log().add("driveTo(): Motors in use");
            return;
        }

        // Set a target, translated from inches to encoder ticks
        int leftTarget = leftDrive.getCurrentPosition();
        int rightTarget = rightDrive.getCurrentPosition();
        leftTarget += distance * TICKS_PER_INCH;
        rightTarget += distance * TICKS_PER_INCH;
        leftDrive.setTargetPosition(leftTarget);
        rightDrive.setTargetPosition(rightTarget);

        // Start the motors
        driveCmdRunning = true;
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
    }

    public void turnTo(float speed, int angle) {
        // Don't allow new moves if we're still busy
        if (isBusy()) {
            telemetry.log().add("driveTo(): Motors in use");
            return;
        }

        // Fake turns using a distance translation
        // We have a gyro but let's start with just one control mode
        int leftTarget = leftDrive.getCurrentPosition();
        int rightTarget = rightDrive.getCurrentPosition();
        leftTarget += angle * TICKS_PER_INCH;
        rightTarget -= angle * TICKS_PER_INCH;
        leftDrive.setTargetPosition(leftTarget);
        rightDrive.setTargetPosition(rightTarget);

        // Start the motors
        driveCmdRunning = true;
        leftDrive.setPower(speed);
        rightDrive.setPower(-speed);
    }
}