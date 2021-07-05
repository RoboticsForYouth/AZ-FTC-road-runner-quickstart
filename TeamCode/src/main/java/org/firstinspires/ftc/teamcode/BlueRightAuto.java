package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.ArrayList;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;

@Autonomous(name="1BlueRightAuto", group="Linear Opmode")
public class BlueRightAuto extends AutoBase {

    @Override
    public void runOpMode() {
        
        initAuto("WebcamRight");

        while (opModeIsActive()) {
            if (first) {
                detect(false); //true - Powershot, false - highgoal (for starting velocity)
                powerShot(true); //true - Right; false - Left
                //moveToShoot(true); //true - Right; false - Left
                
                if (numRings.equals("Quad")) {
                    base.moveRight(500, 0.6);
                    base.runToPosition(-2500, 0.45);
                    sleep(500);
                    imu.turnRight90();
                    base.runToPosition(200, 0.45);
                    sleep(1000);
                    wobbleTool.down();
                    sleep(1000);
                    wobbleTool.release();
                    wobbleTool.up();
                    imu.rotate(0, 0, 0.3); 
                    sleep(500);
                    imu.rotate(0, 0, 0.3); 
                    base.runToPosition(2000, 0.45);
                }
                else if (numRings.equals("Single")) {
                    base.moveLeft(700, 0.6);
                    sleep(100);
                    imu.rotate(0, 0, 0.3);
                    sleep(100);
                    imu.rotate(0, 0, 0.25);
                    base.runToPosition(-2000, 0.45);
                    imu.turnRight90();
                    sleep(500);
                    base.runToPosition(-50, 0.35);
                    wobbleTool.down();                    
                    sleep(1000);
                    wobbleTool.release();
                    wobbleTool.up();
                    imu.rotate(0, 0, 0.35);
                    base.runToPosition(1500, 0.5);
                }
                else {
                    base.moveLeft(200, 0.6);
                    sleep(100);
                    imu.rotate(0, 0, 0.3);
                    sleep(100);
                    imu.rotate(0, 0, 0.25);
                    base.runToPosition(-700, 0.45);
                    sleep(500);
                    imu.turnRight90(); 
                    sleep(200);
                    base.runToPosition(400, 0.45);
                    wobbleTool.down();
                    sleep(1000);
                    wobbleTool.release();
                    wobbleTool.pos(0.6);
                    sleep(200);
                    base.runToPosition(-400, 0.45);

                }
                first = false;
            }
            
            
        }
        launcher.stopLauncher();
        base.stopMotors();
    }
}