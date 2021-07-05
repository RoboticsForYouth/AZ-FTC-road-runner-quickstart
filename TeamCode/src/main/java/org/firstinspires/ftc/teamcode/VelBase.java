package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


public class VelBase extends Base {

    public VelBase(LinearOpMode opMode) {
       super(opMode);
    }
    
    public void moveStraight(double vel, long time) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        backL.setVelocity(vel);
        backR.setVelocity(vel);
        frontL.setVelocity(vel);
        frontR.setVelocity(vel);
        linearOpMode.sleep(time);
        linearOpMode.telemetry.addData("velocity: ", backL.getVelocity());
        linearOpMode.telemetry.update();
        stopMotors(); 
        
    }
    public void moveLeftv(double vel, long time) {
        backL.setVelocity(0.7* -vel);
        frontL.setVelocity(vel);
        backR.setVelocity(0.7*vel);
        frontR.setVelocity(-vel);
        linearOpMode.sleep(time);
        linearOpMode.telemetry.addData("velocity: ", backL.getVelocity());
        linearOpMode.telemetry.update();
        stopMotors(); 
    }
    public void moveRightv(double vel, long time) {
        backL.setVelocity(0.7*vel);
        frontL.setVelocity(-vel);
        backR.setVelocity(0.7*-vel);
        frontR.setVelocity(vel);
        linearOpMode.sleep(time);
        linearOpMode.telemetry.addData("velocity: ", backL.getVelocity());
        linearOpMode.telemetry.update();
        stopMotors(); 
    }
    public void printPID() {
        linearOpMode.telemetry.addData("pid: ", backL.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        linearOpMode.telemetry.update();
    }
}