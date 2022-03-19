package org.firstinspires.ftc.teamcode.freightFrenzy.tools;

import android.os.Build;
import android.util.Pair;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Set;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;


public class AZUtil {

    private static final HashMap<String, Object> printMap = new HashMap<>();


    public static void setMotorTargetPostion(DcMotor motor, int pos, double power) {
        motor.setTargetPosition(pos);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    public static void waitUntilMotorAtPos(LinearOpMode opMode, DcMotor motor, int pos) {
        int tolerance = 3;
        while (opMode.opModeIsActive() && motor.isBusy()
//                &&
//                !(motor.getCurrentPosition() > (pos-3) &&
//                motor.getCurrentPosition() < (pos+3))
        ) {
            opMode.sleep(100);
        }
    }

    public static boolean isAtPos(DcMotor motor, int targetPos) {
        if (motor.getCurrentPosition() == targetPos)
            return true;
        return false;
    }

    static ExecutorService pool = Executors.newFixedThreadPool(1);

    static HashMap<String, ExecutorService> poolMap = new HashMap<>();

    public static final String TURN_TABLE = "TurnTable";
    public static final String FREIGHT_INTAKE_SENSOR = "FreightIntakeSensor";

    static {
        poolMap.put(TURN_TABLE, Executors.newFixedThreadPool(1));
        poolMap.put(FREIGHT_INTAKE_SENSOR, Executors.newFixedThreadPool(1));
    }

    // todo: write your code here

    public static void runInParallel(Runnable r) {
        // pool.submit(r );
        new Thread(r).start();
    }

    //runs in specified pool. will create pool if does not exist. Recommended that
    //the pool is pre-created ro avoid latency
    public static void runInParallelPool(String poolName, Runnable r) {
        poolMap.get(poolName).execute(r);
    }

    //runs in default pool
    public static void runInParallelPool(Runnable r) {
        pool.execute(r);
    }

    public static void print(Telemetry telemetry, List<Pair<String, Object>> list) {
        list.stream().forEach(p ->printMap.put(p.first, p.second));
        printMapToTelemetry(telemetry);
    }

    private static void printMapToTelemetry(Telemetry telemetry){
        telemetry.addLine(String.valueOf(printMap));
        telemetry.update();
    }
    public static void print(Telemetry telemetry, String str, Object obj) {
        printMap.put(str, obj);
        printMapToTelemetry(telemetry);
//        Set<String> strings = printMap.keySet();
//        Iterator iterator = strings.iterator();
//        while (iterator.hasNext()) {
//            String key = (String) iterator.next();
//            telemetry.addData(key, printMap.get(key));
//
//        }
    }

    public static boolean isMotorAtPosition(DcMotor motor, int pos){
        int currPos = motor.getCurrentPosition();
        return (currPos > pos - 3 && currPos < pos + 3);
    }

    public static Telemetry getMultiTelemetry(LinearOpMode opMode){
        FtcDashboard dashboard = FtcDashboard.getInstance();
        return new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());
    }
}