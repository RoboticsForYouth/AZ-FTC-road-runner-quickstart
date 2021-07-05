package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.ArrayList;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;

@Autonomous(name="2RedRightAuto", group="Linear Opmode")
public class RedRightAuto extends AutoBase {

    
    @Override
    public void runOpMode() {
        
        initAuto("WebcamRight");

        while (opModeIsActive()) {
            if (first) {
                detect(false); //true - Powershot, false - highgoal (for starting velocity)
                moveToShoot(true, false); //true - Right; false - Left
                
                if (numRings.equals("Quad")) {
                    base.moveLeft(400, 0.6);
                    base.runToPosition(-1600, 0.7);
                    imu.rotate(130, 0, 0.5);
                    sleep(500);
                    wobbleTool.down();
                    sleep(1000);
                    wobbleTool.release();
                    wobbleTool.up();
                    imu.rotate(180, 0, 0.5);
                    base.runToPosition(-1000, 0.6);
                   
                    
                }
                else if (numRings.equals("Single")) {
                    base.moveLeft(200, 0.6);
                    base.runToPosition(-1700, 0.6);
                    imu.turnRight90();
                    sleep(500);
                    base.runToPosition(-150, 0.35);
                    wobbleTool.down();
                    wobbleTool.release();
                    sleep(1000);
                    wobbleTool.up();
                    imu.rotate(0, 0, 0.35);
                    base.runToPosition(1300, 0.7);
                }
                else { 
                    base.moveLeft(200, 0.6);
                    base.runToPosition(70, 0.6);
                    imu.rotate(130, 0, 0.6);
                    sleep(500);
                    wobbleTool.down();
                    sleep(1000);
                    wobbleTool.release();
                    wobbleTool.up();
                    
                    /*
                    base.runToPosition(100, 0.45);
                    imu.rotate(180, 0, 0.4);
                    base.runToPosition(100, 0.45);
                    base.moveRight(400, 0.45);
                    */
                    /*
                    base.runToPosition(100, 0.6);
                    imu.rotate(180, 0, 0.6);
                    base.runToPosition(-1000, 0.6);
                    base.moveRight(200, 0.45);
                    while(runtime.seconds() < 26) {
                        sleep(100);
                    }           
                    base.runToPosition(950, 0.7);
                    */
                    base.runToPosition(100, 0.6);
                    imu.rotate(0, 0, 0.6);
                    base.runToPosition(1000, 0.6);
                    imu.turnRight90();
                    base.runToPosition(2000, 0.6);
                    imu.rotate(180, 0, 0.6);
                    base.runToPosition(1500, 0.7);

                    

                    
                    
                }
                first = false;
            }
            
            
        }

    }
}
