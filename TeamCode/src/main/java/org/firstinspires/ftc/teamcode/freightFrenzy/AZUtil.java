package org.firstinspires.ftc.teamcode.freightFrenzy;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;


public class AZUtil {

    public static void setMotorTargetPostion(DcMotor motor, int pos, double power){
        motor.setTargetPosition(pos);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }
    
    public static void waitUntilMotorAtPos(LinearOpMode opMode, DcMotor motor, int pos)  {
        int tolerance = 3;
        while(opMode.opModeIsActive() && motor.isBusy() && 
                !(motor.getCurrentPosition() > (pos-3) &&
                motor.getCurrentPosition() < (pos+3))
           ){
            opMode.sleep(100);
       }
    }
    public static boolean isAtPos(DcMotor motor, int targetPos) {
        if(motor.getCurrentPosition() == targetPos)
            return true;
        return false;
    }
    static ExecutorService pool = Executors.newFixedThreadPool(1);

    // todo: write your code here

    public static void runInParallel(Runnable r){
        // pool.submit(r );
        new Thread(r).start();
    }



}