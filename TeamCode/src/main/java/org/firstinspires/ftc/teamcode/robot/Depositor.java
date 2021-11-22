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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.OrderedEnum;
import org.firstinspires.ftc.teamcode.utils.OrderedEnumHelper;

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
    public static double TILT_BACK = 0.6;
    public static double TILT_FORWARD = 0.44;
    public static double LOW_OPEN = 0.98;
    public static double LOW_CLOSE = 0.56;
    public static double MID_OPEN = 0.9;
    public static double MID_CLOSE = 0.47;
    public static double LOW_DOOR_FLIPPER_MOVE_TIME = 0.25;
    public static double MID_DOOR_FLIPPER_MOVE_TIME = 0.5;
    public static double HIGH_DOOR_FLIPPER_MOVE_TIME = 0.75;
    public static double BLOCK_SHOVE_TIME = 0.25;
    public static double FLIPPER_MOVE_TIME_INITIAL = 1.0;   // REMOVE THIS when the magnetic sensor is installed
    public static double REVERSE_BELT_TIME = 1.0;           // for TeleOp only
    public static double DOUBLE_PRESS_TIME = 0.5;

    // Members
    private boolean enabled = false;
    private AUTO_STATE state = AUTO_STATE.DONE;
    private AUTO_STATE oldState = AUTO_STATE.DONE;
    private DOOR_USED required_Door = DOOR_USED.NONE;
    private ElapsedTime timer1 = new ElapsedTime();
    private ElapsedTime timer2 = new ElapsedTime();
    private ElapsedTime timer3 = new ElapsedTime();
    private ElapsedTime timer4 = new ElapsedTime();
    private ElapsedTime timerX = new ElapsedTime();
    private ElapsedTime timerY = new ElapsedTime();
    private ElapsedTime timerB = new ElapsedTime();
    private boolean timer1Started = false;
    private boolean timer2Started = false;
    private boolean timer3Started = false;
    private boolean timer4Started = false;
    private double FLIPPER_MOVE_TIME = 1.0;

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
        // Skip processing if we're disabled
        if (!enabled) {
            return;
        }
        state = AUTO_STATE.DONE;
        low.setPosition(LOW_CLOSE);
        mid.setPosition(MID_CLOSE);
        tilt.setPosition(TILT_FORWARD);
    }

    @Override
    public void loop() {
        // Skip processing if we're disabled
        if (!enabled) {
            return;
        }

        // if the state changed, set ModeComplete to false
        if (state != oldState) {
            oldState = state;
            timer1Started = timer2Started = timer3Started = timer4Started = false;
        }
        switch (state) {
            // Tilt forward, move flipper to start position
            case TILTED_FORWARD:
                low.setPosition(LOW_CLOSE);
                mid.setPosition(MID_CLOSE);
                tilt.setPosition(TILT_FORWARD);

                belt.setPower(BELT_SPEED);
                // start the timer
                if (!timer3Started) {
                    timer3.reset();
                    timer3Started = true;
                }
                // once the belt has been going for time required for the selected door, stop the belt
                if (timer3.seconds() >= FLIPPER_MOVE_TIME_INITIAL) {
                    belt.setPower(0);
                    state = AUTO_STATE.DONE;
                }
                break;
            case DOOR_PREP:      // Move the flipper to below the required door
                belt.setPower(BELT_SPEED);
                // start the timer
                if (!timer1Started) {
                    timer1.reset();
                    timer1Started = true;
                }
                // once the belt has been going for time required for the selected door, stop the belt
                if (timer1.seconds() >= FLIPPER_MOVE_TIME) {
                    belt.setPower(0);
                    state = AUTO_STATE.DONE;
                }
                break;
            case DOOR_OPEN:      // Open required door and run conveyor for the BLOCK_SHOVE_TIME
                switch (required_Door) {
                    case LOW_DOOR:
                        low.setPosition((LOW_OPEN));
                        break;
                    case MID_DOOR:
                        mid.setPosition(MID_OPEN);
                        break;
                    case HIGH_DOOR:
                        // this will be implemented later if they add a door
                        break;
                }
                belt.setPower(BELT_SPEED);
                if (!timer2Started) {
                    timer2.reset();
                    timer2Started = true;
                }
                // once the belt has been going for the BLOCK_SHOVE_TIME, stop the belt
                if (timer2.seconds() >= BLOCK_SHOVE_TIME) {
                    belt.setPower(0);
                    state = AUTO_STATE.DONE;
                }
                break;
            case TILTED_BACK:
                tilt.setPosition(TILT_BACK);
                state = AUTO_STATE.DONE;
                break;
            case REVERSE_RUN:      // Run the belt in reverse for a set time (for TeleOp only)
                belt.setPower(-BELT_SPEED);
                if (!timer4Started) {
                    timer4.reset();
                    timer4Started = true;
                }
                // once the belt has been going for the BLOCK_SHOVE_TIME, stop the belt
                if (timer4.seconds() >= REVERSE_BELT_TIME) {
                    belt.setPower(0);
                    state = AUTO_STATE.DONE;
                }
                break;
            case DONE:
                tilt.setPosition(tilt.getPosition());
                low.setPosition(low.getPosition());
                mid.setPosition(low.getPosition());
                belt.setPower(0);
                break;
        }
        if (state != AUTO_STATE.DONE) {
            if (gamepad2.x && timerX.seconds() < DOUBLE_PRESS_TIME) setDoor(DOOR_USED.LOW_DOOR);
            if (gamepad2.y && timerY.seconds() < DOUBLE_PRESS_TIME) setDoor(DOOR_USED.MID_DOOR);
            if (gamepad2.b && timerB.seconds() < DOUBLE_PRESS_TIME) setDoor(DOOR_USED.HIGH_DOOR);

            if (gamepad2.a) state = AUTO_STATE.TILTED_FORWARD;
            if (gamepad2.b) {
                timerB.reset();
                state = AUTO_STATE.DOOR_PREP;
            }
            if (gamepad2.x) {
                timerX.reset();
                state = AUTO_STATE.DOOR_OPEN;
            }
            if (gamepad2.y) {
                timerY.reset();
                state = AUTO_STATE.TILTED_BACK;
            }
        }

        // Debug when requested
        if (DEBUG) {
            telemetry.addData("Depositor Output",
                    "B %.2f/%d, L %.2f, M %.2f, T %.2f",
                    belt.getPower(), belt.getCurrentPosition(),
                    low.getPosition(), mid.getPosition(), tilt.getPosition());
            telemetry.addData("mode", state);
        }
    }

    enum AUTO_STATE implements OrderedEnum {
        TILTED_FORWARD,     // Tilt forward, move flipper to start position
        DOOR_PREP,          // Move the flipper to below the required door
        DOOR_OPEN,          // Open the required door and run conveyor a small amount
        TILTED_BACK,        // Tilt back
        REVERSE_RUN,        // Run the belt in reverse for a set time (for TeleOp only)
        DONE;               // Power down all servos

        public AUTO_STATE next() {
            return OrderedEnumHelper.next(this);
        }
    }

    enum DOOR_USED {
        LOW_DOOR,
        MID_DOOR,
        HIGH_DOOR,
        NONE
    }

    @Override
    public void stop() {
        state = AUTO_STATE.DONE;
    }

    public void setDoor(DOOR_USED newDoor) {
        required_Door = newDoor;
        switch (required_Door) {
            case LOW_DOOR:
                FLIPPER_MOVE_TIME = LOW_DOOR_FLIPPER_MOVE_TIME;
                break;
            case MID_DOOR:
                FLIPPER_MOVE_TIME = MID_DOOR_FLIPPER_MOVE_TIME;
                break;
            case HIGH_DOOR:
                FLIPPER_MOVE_TIME = HIGH_DOOR_FLIPPER_MOVE_TIME;
                break;
        }
    }
}