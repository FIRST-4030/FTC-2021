package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp//(name = "Sensor: REV2mDistance", group = "Sensor")
public class DistanceTestDetailed extends LinearOpMode {

    private DistanceSensor Distance;

    @Override
    public void runOpMode() {
        // you can use this as a regular DistanceSensor.
        Distance = hardwareMap.get(DistanceSensor.class, "Distance");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)Distance;

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            // generic DistanceSensor methods.
            telemetry.addData("deviceName", Distance.getDeviceName() );
            telemetry.addData("range", String.format("%.01f mm", Distance.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", Distance.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", Distance.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", Distance.getDistance(DistanceUnit.INCH)));

            // Rev2mDistanceSensor specific methods.
            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

            telemetry.update();
        }
    }

}
