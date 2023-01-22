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


    private static int tolerance = 50;

    private static int grabThresholdColor = 180;

    private Servo claw;
    private ColorSensor colorSensor;
    LinearOpMode opMode;

    public enum ClawPos {

        OPEN(0.61),
        CLOSE(0.4);

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
        opMode.telemetry.addData("coneColor:", coneColor);
        opMode.telemetry.addData("Threshold:", grabThresholdColor);
        opMode.telemetry.update();
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
        this.opMode = this;

        telemetry.addLine("Init");
        telemetry.update();
        test();
    }

    private void test() {

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