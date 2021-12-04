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
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.gamepad.GAMEPAD;
import org.firstinspires.ftc.teamcode.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.gamepad.PAD_KEY;
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
    private Servo high = null;
    private TouchSensor sensor = null;

    // Config
    public static boolean DEBUG = false;
    public static double BELT_SPEED = 0.75;
    public static double TILT_BACK = 0.42;
    public static double TILT_FORWARD = 0.2;
    public static double LOW_OPEN = 0.98;
    public static double LOW_CLOSE = 0.56;
    public static double MID_OPEN = 0.9;
    public static double MID_CLOSE = 0.47;
    public static double HIGH_OPEN = 0.13;
    public static double HIGH_INIT = 0.55;

    // Members
    private boolean enabled = false;
    private AUTO_STATE state = AUTO_STATE.DONE;
    private AUTO_STATE oldState = AUTO_STATE.DONE;
    private DOOR_USED required_Door = DOOR_USED.NONE;
    private InputHandler in;

    @Override
    public void init() {
        // Depositor
        try {
            in = Globals.input(this);

            belt = hardwareMap.get(DcMotor.class, "Depbelt");
            belt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            low = hardwareMap.get(Servo.class, "Deplow");
            low.setPosition(LOW_CLOSE);
            mid = hardwareMap.get(Servo.class, "Depmid");
            mid.setPosition(MID_CLOSE);
            high = hardwareMap.get(Servo.class, "Dephigh");
            high.setPosition(HIGH_INIT);
            tilt = hardwareMap.get(Servo.class, "Deptilt");
            sensor = hardwareMap.get(TouchSensor.class, "DS");
            in.register("LOW", GAMEPAD.driver2, PAD_KEY.x);
            in.register("MID", GAMEPAD.driver2, PAD_KEY.y);
            in.register("HIGH", GAMEPAD.driver2, PAD_KEY.b);
            in.register("REVERSE", GAMEPAD.driver2, PAD_KEY.a);
            in.register("TILT_FORWARD", GAMEPAD.driver2, PAD_KEY.dpad_left);
            in.register("TILT_BACK", GAMEPAD.driver2, PAD_KEY.dpad_right);

            enabled = true;
        } catch (Exception e) {
            telemetry.log().add(getClass().getSimpleName() + ": " +
                    "Could not initialize");
        }
    }

    @Override
    public void init_loop() {
        if (!sensor.isPressed()) {
            belt.setPower(1);
        } else {
            belt.setPower(0);
            belt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            belt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
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
        high.setPosition(HIGH_OPEN);
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
        }
        switch (state) {
            // Tilt forward, move flipper to start position
            case TILTED_FORWARD:
                low.setPosition(LOW_CLOSE);
                mid.setPosition(MID_CLOSE);
                high.setPosition(HIGH_OPEN);
                tilt.setPosition(TILT_FORWARD);

                belt.setTargetPosition(50);
                belt.setPower(BELT_SPEED);
                state = AUTO_STATE.DONE;
                break;
            case DOOR_PREP:      // Move the flipper to below the required door
                switch (required_Door) {
                    case LOW_DOOR:
                        belt.setTargetPosition(250);
                        break;
                    case MID_DOOR:
                        belt.setTargetPosition(500);
                        break;
                    case HIGH_DOOR:
                        belt.setTargetPosition(750);
                        break;
                }
                belt.setPower(BELT_SPEED);
                state = AUTO_STATE.DONE;
                break;
            case DOOR_OPEN:      // Open required door and run conveyor until the magnetic switch is active
                switch (required_Door) {
                    case LOW_DOOR:
                        low.setPosition((LOW_OPEN));
                        break;
                    case MID_DOOR:
                        mid.setPosition(MID_OPEN);
                        break;
                    case HIGH_DOOR:
                        high.setPosition(HIGH_OPEN);
                        break;
                }
                if (!sensor.isPressed()) {
                    belt.setPower(BELT_SPEED);
                } else {
                    belt.setPower(0);
                    state = AUTO_STATE.TILTED_FORWARD;
                }
                break;
            case TILTED_BACK:
                tilt.setPosition(TILT_BACK);
                state = AUTO_STATE.DONE;
                break;
            case REVERSE_RUN:      // Run the belt in reverse for a set time (for TeleOp only)
                belt.setPower(-BELT_SPEED);
                break;
            case DONE:
                tilt.setPosition(TILT_FORWARD);
                low.setPosition(low.getPosition());
                mid.setPosition(mid.getPosition());
                high.setPosition(high.getPosition());
                belt.setPower(0);
                break;
        }
        if (state == AUTO_STATE.DONE && !belt.isBusy()) {
            /* if (gamepad2.x && timerX.seconds() < DOUBLE_PRESS_TIME) setDoor(DOOR_USED.LOW_DOOR);
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
            */
            if (in.down("REVERSE")){
                state = AUTO_STATE.REVERSE_RUN;
            } else if (in.up("REVERSE")) {
                state = AUTO_STATE.DONE;
            }
            if (in.down("LOW")) {
                if (required_Door == DOOR_USED.LOW_DOOR) {
                    state = AUTO_STATE.DOOR_OPEN;
                } else {
                    setDoor(DOOR_USED.LOW_DOOR);
                    state = AUTO_STATE.DOOR_PREP;
                }
            }
            if (in.down("MID")) {
                if (required_Door == DOOR_USED.MID_DOOR) {
                    state = AUTO_STATE.DOOR_OPEN;
                } else {
                    setDoor(DOOR_USED.MID_DOOR);
                    state = AUTO_STATE.DOOR_PREP;
                }
            }
            if (in.down("HIGH")) {
                if (required_Door == DOOR_USED.HIGH_DOOR) {
                    state = AUTO_STATE.DOOR_OPEN;
                } else {
                    setDoor(DOOR_USED.HIGH_DOOR);
                    state = AUTO_STATE.DOOR_PREP;
                }
            }
            if (in.down("TILT_FORWARD")) state = AUTO_STATE.TILTED_FORWARD;
            if (in.down("TILT_BACK")) {
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
                break;
            case MID_DOOR:
                break;
            case HIGH_DOOR:
                break;
        }
    }
}