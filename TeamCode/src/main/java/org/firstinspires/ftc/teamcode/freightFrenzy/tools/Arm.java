package org.firstinspires.ftc.teamcode.freightFrenzy.tools;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.text.MessageFormat;

@Autonomous(name = "ArmAuto")
public class Arm extends LinearOpMode {

    private static final int INC = 30;
    DcMotor armMotor;
    //Need to set Home level to -80 because it needs that extra momentum to go to zero position
    private int[] level = {0, -80, 700, 690, 1500, 350, 600, 300}; //OLD!!!!!!

    public enum ArmLevel {
        ZERO(0), //0
        HOME(0), //-80
        LEVEL1(600), //720
        LEVEL2(900), //690
        LEVEL3(1425), //1600
        MOVE(400), //700
        ARM_ROTATE(275), //625
        INTAKE(205), //350
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
        //set encoder to zero
        setupPos();
    }

    public void setupPos() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        moveToLevel(ArmLevel.ZERO);
    }

    public Arm(LinearOpMode opMode) {
        this.opMode = opMode;
        setup();
        setupPos();
    }

    public Arm() {
        super();
    }

    public void moveToLevel(int index) {
        AZUtil.setMotorTargetPostion(armMotor, level[index], 0.6);
        AZUtil.waitUntilMotorAtPos(opMode, armMotor, level[index]);
    }

    public void moveTo0() {
        AZUtil.setMotorTargetPostion(armMotor, ArmLevel.ZERO.getValue(), 0.4);
        AZUtil.waitUntilMotorAtPos(opMode, armMotor, ArmLevel.ZERO.getValue());
    }

    public void moveToLevel(ArmLevel armLevel) {
        if( !AZUtil.isMotorAtPosition(armMotor, armLevel.getValue())) {
            AZUtil.setMotorTargetPostion(armMotor, armLevel.getValue(), 0.6);
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
        AZUtil.setMotorTargetPostion(armMotor, newPos, 0.4);
        AZUtil.waitUntilMotorAtPos(opMode, armMotor, newPos);
    }
    public void moveDown() {
        int newPos = armMotor.getCurrentPosition() - INC;
        AZUtil.setMotorTargetPostion(armMotor, newPos, 0.4);
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
        sleep(2000);
    }
}