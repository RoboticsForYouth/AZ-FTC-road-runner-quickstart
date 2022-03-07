package org.firstinspires.ftc.teamcode.freightFrenzy.tools;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class Wrist extends LinearOpMode {

    Servo wrist;
    LinearOpMode opMode;

    private static final double INTAKE_POS = 0.85;
    private static final double LEVEL_1_DROP_POS = 0.7;
    private static final double LEVEL_2_DROP_POS = 0.7;
    private static final double LEVEL_3_DROP_POS = 0.55;
    private static final double HOME_POS = 1;
    private static final double SECURED_POS = 0.7;

    public enum WristPos{
        HOME (1.0),
        LEVEL1(0.7),
        LEVEL2(0.7),
        LEVEL3(0.55),
        INTAKE(.85);

        private double value;
        WristPos(double pos){
            value = pos;
        }

        public double getValue(){
            return value;
        }
    }

    public void setup() {
        wrist = opMode.hardwareMap.get(Servo.class, "wrist");
        telemetry = AZUtil.getMultiTelemetry(opMode);
    }


    public void setupPos() {
        wrist.setPosition(WristPos.HOME.getValue());
    }

    public Wrist() {
        opMode = this;
    }

    public Wrist(LinearOpMode opMode) {
        this.opMode = opMode;
        setup();
    }

    public void intakePos() {
        wrist.setPosition(INTAKE_POS);
    }

    public void setLevel1DropPos() {
        wrist.setPosition(LEVEL_1_DROP_POS);
    }

    public void setLevel2DropPos() {
        wrist.setPosition(LEVEL_2_DROP_POS);
    }

    public void setLevel3DropPos() {
        wrist.setPosition(LEVEL_3_DROP_POS);
    }

    public void homePos() {
        wrist.setPosition(HOME_POS);
    }

    public void setSecuredPos() {
        if( wrist.getPosition() != SECURED_POS) {
            wrist.setPosition(SECURED_POS);
        }
    }

    @Override
    public void runOpMode() {
        setup();
        waitForStart();
        setupPos();
        printState("Home Pos");
        sleep(4000);
        intakePos();
        printState("Intake Pos");

        sleep(4000);

        setLevel3DropPos();
        printState("Level 3");
        sleep(4000);
        homePos();
        printState("Home Pos");

        sleep(4000);
    }

    private void printState(String s) {
        AZUtil.print(telemetry, "Current Pos", this.wrist.getPosition());
        AZUtil.print(telemetry, "Current State", s);
    }


}
