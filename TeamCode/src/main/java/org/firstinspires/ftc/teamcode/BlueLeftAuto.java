package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.ArrayList;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;

@Autonomous(name="2BlueLeftAuto", group="Linear Opmode")
public class BlueLeftAuto extends AutoBase {

    
    @Override
    public void runOpMode() {
        initAuto("WebcamLeft");
        
        while (opModeIsActive()) {
            if (first) {
                detect(false);
                powerShot(false); //true - Right; false - Left

                if (numRings.equals("Quad")) {
                    
                    base.runToPosition(-1500, 0.7);
                    imu.rotate(181, 0, 0.5);
                    sleep(500);
                    wobbleTool.down();
                    sleep(1000);
                    wobbleTool.release();
                    wobbleTool.up();
                    base.runToPosition(-900, 0.6);
                   
                    
                }
                else if (numRings.equals("Single")) {
                    base.moveRight(150, 0.6);
                    base.runToPosition(-1400, 0.45);
                    imu.turnLeft90();
                    sleep(500);
                    base.runToPosition(-150, 0.35);
                    wobbleTool.down();                    
                    sleep(1000);
                    wobbleTool.release();
                    wobbleTool.up();
                   // base.runToPosition(-200, 0.35);
                    imu.rotate(0, 0, 0.5);
                    base.runToPosition(900, 0.35);
                    
                }
                else { 
                    base.moveRight(200, 0.6);
                    imu.rotate(180, 0, 0.4);
                    base.runToPosition(-200, 0.45);
                    sleep(500);
                    wobbleTool.down();
                    sleep(1000);
                    wobbleTool.release();
                    wobbleTool.pos(0.65);
                    base.runToPosition(150, 0.35);


                  
                }
                first = false;
            }
            
            
        }
    }
}
