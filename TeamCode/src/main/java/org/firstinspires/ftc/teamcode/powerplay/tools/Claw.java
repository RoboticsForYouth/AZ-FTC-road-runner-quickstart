package org.firstinspires.ftc.teamcode.powerplay.tools;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "ClawAuto")
public class Claw extends LinearOpMode {

    private Servo claw;
    LinearOpMode opMode;
    public enum ClawPos {

        OPEN(0.6),
        CLOSE(0.3);

        private double value;

        ClawPos(double val) {
            this.value = val;
        }

        public double getValue() {
            return this.value;
        }
    }


    public void setup() {
        claw = opMode.hardwareMap.get(Servo.class, "claw");
        setupPos();
    }

    public void setupPos() {
        claw.close();
    }

    public Claw() {
        opMode = this;
    }

    public Claw(LinearOpMode newOpMode) {
        this.opMode = newOpMode;
        setup();
    }

    public void open() {
        claw.setPosition(ClawPos.OPEN.value);
    }
   public void close() {
        claw.setPosition(ClawPos.CLOSE.value);
    }


    @Override
    public void runOpMode() {
        setup();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        MultipleTelemetry telemetry;
        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());

        waitForStart();
        int count = 0;
        while(opModeIsActive() & count < 1) {
            open();
            sleep(5000);
            telemetry.addLine("Opened");
            telemetry.update();

            close();
            sleep(5000);
            telemetry.addLine("Closed");
            telemetry.update();

            sleep(15000);
            count++;
        }
    }
}