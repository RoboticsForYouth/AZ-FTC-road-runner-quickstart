package org.firstinspires.ftc.teamcode.freightFrenzy;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.HashMap;
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
        while(opMode.opModeIsActive() && motor.isBusy() ||(
                !(motor.getCurrentPosition() > (pos-3) &&
                motor.getCurrentPosition() < (pos+3)))

           ){
            opMode.telemetry.addData("Motor Position", motor.getCurrentPosition());
            opMode.telemetry.update();
            opMode.sleep(100);
       }
    }

    static HashMap<String, ExecutorService> poolMap = new HashMap<>();

    public static final String FREIGHT_TOOL = "FreightTool";

    public static final String CAROUSEL = "Carousel";

    public static final String TAPE_DRIVE = "TapeDrive";

    static {
       poolMap.put(FREIGHT_TOOL, Executors.newFixedThreadPool(1));
       poolMap.put(CAROUSEL, Executors.newFixedThreadPool(1));
       poolMap.put(TAPE_DRIVE, Executors.newFixedThreadPool(1));
    }
    static ExecutorService DefaultPool = poolMap.get(FREIGHT_TOOL);

    // todo: write your code here

    public static void runInParallel(Runnable r){
        // pool.submit(r );
        new Thread(r).start();
    }

    public static void runInParallelPool(Runnable r){
        DefaultPool.execute(r);
    }

    public static void runInParallelPool(Runnable r, String poolName){
        ExecutorService executorService = poolMap.get(poolName);
        if( executorService == null){
            executorService = DefaultPool;
        }
        executorService.execute(r);
    }



}