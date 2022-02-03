package org.firstinspires.ftc.teamcode.freightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.text.MessageFormat;


public class Arm extends LinearOpMode {
    
    DcMotor armMotor;
    private int[] level = {200, 500, 690, 980, 350, 600};

    public enum ArmLevel {
        HOME, LEVEL1, LEVEL2, LEVEL3, MOVE, ARMROTATE
    }
    LinearOpMode opMode;

    public void setup() {
        //opMode = this;
        armMotor = opMode.hardwareMap.get(DcMotor.class, "arm");/* code */
        //set encoder to zero
    }
    public void setupPos(){
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        moveToLevel(0);
    }
    public Arm(LinearOpMode opMode) {
        this.opMode = opMode;
        setup();
    }
    
    public void moveToLevel(int index) {
        AZUtil.setMotorTargetPostion(armMotor, level[index], 0.6);
        AZUtil.waitUntilMotorAtPos(opMode, armMotor, level[index]);
    }
    public void moveTo0() {
        AZUtil.setMotorTargetPostion(armMotor, -80, 0.4);
        AZUtil.waitUntilMotorAtPos(opMode, armMotor, -80);
    }

    public void moveToLevel(ArmLevel armLevel) {
        AZUtil.setMotorTargetPostion(armMotor, level[armLevel.ordinal()], 0.6);
        AZUtil.waitUntilMotorAtPos(opMode, armMotor, level[armLevel.ordinal()]);
    }

    public String getDisplayValues() {
        return MessageFormat.format("Arm Position; {0}", armMotor.getCurrentPosition());

    }

    public boolean isBusy() {
        return armMotor.isBusy();
    }
    @Override
    public void runOpMode(){
        setup();

        waitForStart();
        setupPos();

        moveToLevel(1);
        sleep(2000);
        moveToLevel(2);
        sleep(2000);
        moveToLevel(3);
        sleep(2000);
    }
}