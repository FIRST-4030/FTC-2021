package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp//(name = "Sensor: REV2mDistance", group = "Sensor")
public class DistanceTestDetailed extends LinearOpMode {

    private DistanceSensor Distance1;
    private DistanceSensor Distance2;

    @Override
    public void runOpMode() {
        // you can use this as a regular DistanceSensor.
        Distance1 = hardwareMap.get(DistanceSensor.class, "Dr");
        Distance2 = hardwareMap.get(DistanceSensor.class, "DL");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        //ModernRoboticsI2cRangeSensor sensorTimeOfFlight = (ModernRoboticsI2cRangeSensor)Distance1;

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            // generic DistanceSensor methods.
            //telemetry.addData("deviceName", Distance.getDeviceName() );
            if (Distance1.getDistance(DistanceUnit.INCH) <= 19) {
                telemetry.addData("In Range1: ", "Yes");
            } else { // Otherwise, stop the motor
                telemetry.addData("In Range1: ", "No");
            }
            if (Distance2.getDistance(DistanceUnit.INCH) <= 19) {
                //motor.setPower(0.3);
                telemetry.addData("In Range2: ", "Yes");
            } else { // Otherwise, stop the motor
                //motor.setPower(0);
                telemetry.addData("In Range2: ", "No");
            }
            telemetry.addData("D1 range", String.format("%.01f mm", Distance1.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", Distance1.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", Distance1.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", Distance1.getDistance(DistanceUnit.INCH)));
            telemetry.addData("D2 range", String.format("%.01f mm", Distance2.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", Distance2.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", Distance2.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", Distance2.getDistance(DistanceUnit.INCH)));

            /*// Rev2mDistanceSensor specific methods.
            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));*/

            telemetry.update();
        }
    }

}
