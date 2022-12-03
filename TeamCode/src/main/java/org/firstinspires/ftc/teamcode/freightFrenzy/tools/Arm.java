package org.firstinspires.ftc.teamcode.freightFrenzy.tools;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.text.MessageFormat;

//@Autonomous(name = "ArmAuto")
public class Arm extends LinearOpMode {

    private static final int INC = 30;
    DcMotor armMotor;
    //Need to set Home level to -80 because it needs that extra momentum to go to zero position
    //OLD!!!!!!

    public enum ArmLevel {
        ZERO(0), //0
        HOME(0), //-80
        LEVEL1(450), //720
        SHARED_HUB(500), //720
        LEVEL2(800), //690
        LEVEL3(1300), //1600

        MOVE(400), //700
        ARM_ROTATE(600), //625
        INTAKE(0), //350
        TAPE_DRIVE(500); //750

        private int value;

        ArmLevel(int val) {
            this.value = val;
        }

        public int getValue() {
            return this.value;
        }
    }

    LinearOpMode opMode;

    public void setup() {

        armMotor = opMode.hardwareMap.get(DcMotor.class, "arm");/* code */
        //set encode r to zero
        setupPos();
    }

    public void setupPos() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        moveToLevel(ArmLevel.ZERO);
    }

    public void setup0(){
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        AZUtil.setMotorTargetPosition(armMotor, 0, 0.4);
    }

    public void moveUpManual(){
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        armMotor.setPower(0.5);
        opMode.sleep(1000);
        stopArm();
    }
    public void stopArm(){
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        armMotor.setPower(0);
    }

    public void moveDownManual() {
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        armMotor.setPower(-0.5);
        opMode.sleep(1000);
        stopArm();
    }
    public Arm(LinearOpMode opMode) {
        this.opMode = opMode;
        setup();
        setupPos();
    }

    public Arm() {
        super();
    }

    public synchronized void moveTo0() {
        AZUtil.setMotorTargetPosition(armMotor, ArmLevel.ZERO.getValue(), 0.4);
        AZUtil.waitUntilMotorAtPos(opMode, armMotor, ArmLevel.ZERO.getValue());
    }

    public synchronized void moveToLevel(ArmLevel armLevel) {
        if( !AZUtil.isMotorAtPosition(armMotor, armLevel.getValue())) {
            AZUtil.setMotorTargetPosition(armMotor, armLevel.getValue(), 0.6);
            AZUtil.waitUntilMotorAtPos(opMode, armMotor, armLevel.getValue());
        }
    }

    public String getDisplayValues() {
        return MessageFormat.format("Arm Position; {0}", armMotor.getCurrentPosition());

    }

    public double getCurrentPosition() {
        return armMotor.getCurrentPosition();
    }

    public void moveToMinLevel(ArmLevel armLevel) {
        if (getCurrentPosition() < armLevel.getValue()) {
            moveToLevel(armLevel);
        }
    }
    public void moveUp() {
        int newPos = armMotor.getCurrentPosition() + INC;
        AZUtil.setMotorTargetPosition(armMotor, newPos, 0.4);
        AZUtil.waitUntilMotorAtPos(opMode, armMotor, newPos);
    }
    public void moveDown() {
        int newPos = armMotor.getCurrentPosition() - INC;
        AZUtil.setMotorTargetPosition(armMotor, newPos, 0.4);
        AZUtil.waitUntilMotorAtPos(opMode, armMotor, newPos);
    }

    public boolean isBusy() {
        return armMotor.isBusy();
    }

    @Override
    public void runOpMode() {
        opMode = this;
        setup();

        waitForStart();

        setupPos();

        moveToLevel(ArmLevel.INTAKE);
        sleep(1000);
        moveToLevel(ArmLevel.LEVEL3);
        sleep(1000);
        moveToLevel(ArmLevel.LEVEL1);
        sleep(1000);
        moveToLevel(ArmLevel.MOVE);
        sleep(1000);
        moveToLevel(ArmLevel.INTAKE);
        sleep(2000);
    }
}