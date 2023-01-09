package org.firstinspires.ftc.teamcode.powerplay.tools;

import static android.graphics.Color.RED;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "ClawAuto")
public class Claw extends LinearOpMode {

    //set Color Mode (Red, Blue)
    private int coneColor = RED;


    private static int tolerance = 15;

    private static int grabThresholdColor = 180;

    private Servo claw;
    private ColorSensor colorSensor;
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
        colorSensor = opMode.hardwareMap.get(ColorSensor.class, "clawSensor");
        setupPos();
    }

    public boolean isConeDetected(){
        opMode.telemetry.addData("Red:", colorSensor.red());
        opMode.telemetry.addData("Blue:", colorSensor.blue());
        opMode.telemetry.addData("Threshold:", grabThresholdColor);
        opMode.telemetry.addData("tolerance:", tolerance);
        opMode.telemetry.update();
        if( coneColor == RED){
            return colorSensor.red() > (grabThresholdColor-tolerance);
        } else {
            return colorSensor.blue() > (grabThresholdColor-tolerance);
        }

    }

    public void setGrabThreshold(){
        if( coneColor == RED) {
            grabThresholdColor = colorSensor.red();
        } else {
            grabThresholdColor = colorSensor.blue();
        }
        telemetry.addData("coneColor:", coneColor);
        telemetry.addData("Threshold:", grabThresholdColor);
        telemetry.update();
    }

    public void setConeColor(int color){
        coneColor = color;
    }



    public void setupPos() {
        claw.close();
    }

    public Claw() {
        super();
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
        telemetry.addLine("Init");
        telemetry.update();
        test();
    }

    private void test() {
        this.opMode = this;

        FtcDashboard dashboard = FtcDashboard.getInstance();
//        MultipleTelemetry telemetry;
        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());
        telemetry.addLine("Initialized");
        telemetry.update();
        setup();

        open();
        setConeColor(RED);
        for(int i=0; i<3; i++) {
            setGrabThreshold();
            sleep(500);
        }

        waitForStart();
        int count = 0;
        while (opModeIsActive() ) {
            if( isConeDetected() ){
                close();
            }
        }
    }
}