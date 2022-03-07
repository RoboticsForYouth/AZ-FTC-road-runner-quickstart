package org.firstinspires.ftc.teamcode.freightFrenzy.tools;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class FreightSensor extends LinearOpMode {
    DistanceSensor sensor;
    LinearOpMode opMode = null;
    boolean freightDetected = false;

    public FreightSensor() {

    }

    public FreightSensor(LinearOpMode opMode) {
        this.opMode = opMode;
        setUp();
    }

    private void setUp() {
        sensor = opMode.hardwareMap.get(DistanceSensor.class, "freightSensor");
    }

    public double getSensorDistance() {
        return sensor.getDistance(DistanceUnit.CM);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        this.opMode = this;
        setUp();
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Object distance: ", sensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }

    public void detectFreight() {
        AZUtil.runInParallelPool(
                () -> {
                    double sensorDistance = getSensorDistance();
                    while (sensorDistance > 23.0 && opModeIsActive()) {
                        sensorDistance = getSensorDistance();
                        AZUtil.print(telemetry, "Object distance: ", sensorDistance);

                        sleep(100);
                    }
                    freightDetected = true;
                }
        );
    }

    public boolean isFreightDetected() {
        return freightDetected;
    }
}
