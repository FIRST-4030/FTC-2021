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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
//@TeleOp(name = "Depositor", group = "Test")
public class Depositor extends OpMode {
    // Hardware
    private DcMotor belt = null;
    private Servo low = null;
    private Servo mid = null;
    private Servo tilt = null;

    // Config
    public static boolean DEBUG = false;
    public static double BELT_SPEED = 0.9;
    public static double TILT_UP = 0.6;
    public static double TILT_DOWN = 0.44;
    public static double LOW_OPEN = 0.98;
    public static double LOW_CLOSE = 0.56;
    public static double MID_OPEN = 0.9;
    public static double MID_CLOSE = 0.47;

    // Members
    private boolean enabled = false;

    @Override
    public void init() {
        // Pull in Globals
        telemetry = Globals.opmode(this).telemetry;

        // Depositor
        try {
            belt = hardwareMap.get(DcMotor.class, "Depbelt");
            belt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            low = hardwareMap.get(Servo.class, "Deplow");
            low.setPosition(LOW_CLOSE);
            mid = hardwareMap.get(Servo.class, "Depmid");
            mid.setPosition(MID_CLOSE);

            tilt = hardwareMap.get(Servo.class, "Deptilt");

            enabled = true;
        } catch (Exception e) {
            telemetry.log().add(getClass().getSimpleName() + ": " +
                    "Could not initialize");
        }
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        low.setPosition(LOW_CLOSE);
        mid.setPosition(MID_CLOSE);
        tilt.setPosition(TILT_DOWN);
    }

    @Override
    public void loop() {
        // Skip processing if we're disabled
        if (!enabled) {
            return;
        }

        // Belt and doors
        if (gamepad2.a || gamepad2.b || gamepad2.x) {
            belt.setPower(BELT_SPEED);
        } else if (gamepad2.y) {
            belt.setPower(-BELT_SPEED);
        } else {
            belt.setPower(0);
        }
        if (gamepad2.a) {
            low.setPosition(LOW_OPEN);
        } else {
            low.setPosition(LOW_CLOSE);
        }
        if (gamepad2.b) {
            mid.setPosition(MID_OPEN);
        } else {
            mid.setPosition(MID_CLOSE);
        }

        // Tilt
        if (gamepad2.dpad_right) {
            tilt.setPosition(TILT_UP);
        } else {
            tilt.setPosition(TILT_DOWN);
        }

        // Debug when requested
        if (DEBUG) {
            telemetry.addData("Depositor Output",
                    "B %.2f/%d, L %.2f, M %.2f, T %.2f",
                    belt.getPower(), belt.getCurrentPosition(),
                    low.getPosition(), mid.getPosition(), tilt.getPosition());
        }
    }

    @Override
    public void stop() {
    }
}