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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp(name = "DUCK_SPIN", group = "Test")
public class DUCK_SPIN extends OpMode {
    // Hardware
    private DcMotor duckSpinner = null;

    // Constants used for hardware
    private static double DUCK_POWER = 0.0;
    private static double timerRatio = 0.0;
    private static double duckPowerMin = 0.215;  // min duck spinner speed (0 - 1.0)
    private static double duckPowerMax = 0.465;  // max duck spinner speed (0 - 1.0)
    private static double duckRampTime = 1.25;  // duck spinner ramp time (seconds, >0)

    // Members
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime duckTimer = new ElapsedTime();
    private boolean done = true;
    private int autoStep = 0;

    @Override
    public void init() {
        boolean error = false;
        telemetry.addData("Status", "Initializing...");

        // Duck Spinner
        try {
            duckSpinner = hardwareMap.get(DcMotor.class, "duck");
            duckSpinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            telemetry.log().add("Could not find duck spinner");
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
    }

    public void auto(int step) {
        done = false;
        autoStep = step;
    }

    public boolean isDone() {
        return done;
    }

    @Override
    public void loop() {
        // Duck spinner
        if (done) {
            if (gamepad1.a) {
                DUCK_POWER = duckPowerMin;
                duckTimer.reset();
            } else if (gamepad1.b) {
                DUCK_POWER = -duckPowerMin;
                duckTimer.reset();
            }
        } else {
            switch (autoStep) {
                case 0:
                    // Start duck ramp backward
                    DUCK_POWER = -duckPowerMin;
                    duckTimer.reset();
                    autoStep += 2;
                    break;
                case 1:
                    // Start duck ramp forward
                    DUCK_POWER = duckPowerMin;
                    duckTimer.reset();
                    autoStep++;
                    break;
                default:
                    done = true;
                    break;
            }
        }

        if (DUCK_POWER != 0 && duckTimer.seconds() < duckRampTime) {
            DUCK_POWER = (duckPowerMin + (duckTimer.seconds() / duckRampTime) *
                    (duckPowerMax - duckPowerMin)) * Math.signum(DUCK_POWER);
        } else {
            DUCK_POWER = 0.0;
        }
        duckSpinner.setPower(DUCK_POWER);

    }

    @Override
    public void stop() {
    }
}