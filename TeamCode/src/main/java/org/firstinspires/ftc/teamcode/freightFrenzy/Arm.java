package org.firstinspires.ftc.teamcode.freightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


public class Arm extends LinearOpMode {
    
    DcMotor armMotor;
    private int[] level = {200, 495, 765, 1007, 350};
    LinearOpMode opMode;

    public void setup() {
      
        armMotor = opMode.hardwareMap.get(DcMotor.class, "arm");/* code */
        //set encoder to zero
    }
    public void setupPos(){
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        moveToLevel(0);
    }
    public Arm() {
        opMode = this;
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