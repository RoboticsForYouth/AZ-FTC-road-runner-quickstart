package org.firstinspires.ftc.teamcode.freightFrenzy.tools;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@Autonomous
public class FreightInIntakeSensor extends LinearOpMode {
    DistanceSensor sensor;
    LinearOpMode opMode = null;
    public final static double DETECTION_DISTANCE_CM = 7;
    private boolean freightDetected = false;
    private boolean stopDetection = false;
    private Telemetry telemetry = null;

    public FreightInIntakeSensor() {

    }

    public FreightInIntakeSensor(LinearOpMode opMode) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        setUp();
    }

    private void setUp() {
        sensor = opMode.hardwareMap.get(DistanceSensor.class, "intakeSensor");
    }

    public double getSensorDistance() {
        return sensor.getDistance(DistanceUnit.CM);
    }

    public void startDetection() {
        freightDetected = false;
        stopDetection = false;

        AZUtil.runInParallelPool(
                () -> {
                    double sensorDistance = getSensorDistance();
                     do {
                        sensorDistance = getSensorDistance();
                        AZUtil.print(telemetry,"Intake Sensor (cm):", sensorDistance);
                        sleep(50);
                    } while (sensorDistance > DETECTION_DISTANCE_CM && !stopDetection);
                    freightDetected = true;
                });
    }

    public void stopDetection() {
        stopDetection = true;
    }

    public boolean isFreightDetected() {
        return freightDetected;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        this.opMode = this;
        setUp();
        waitForStart();
        while (opModeIsActive()) {
            AZUtil.print(opMode.telemetry, "Object distance: ", getSensorDistance());
        }
    }
}
