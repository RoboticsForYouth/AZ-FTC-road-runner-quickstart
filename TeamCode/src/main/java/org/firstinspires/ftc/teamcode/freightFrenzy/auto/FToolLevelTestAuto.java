package org.firstinspires.ftc.teamcode.freightFrenzy.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.freightFrenzy.tools.FreightTool;

//@Autonomous
public class FToolLevelTestAuto extends LinearOpMode {
    FreightTool freightTool = null;

    @Override
    public void runOpMode() throws InterruptedException {
        freightTool = new FreightTool(this);
        waitForStart();

        //Set to move
        freightTool.move();
        sleep(2000);
        //set to intake
        freightTool.intake();
        sleep(2000);

        //set to drop at shared hub
        freightTool.setShareHubDropPos();
        sleep(2000);

        freightTool.setLevel2HubDropPos();
        sleep(2000);
//
//        //set to drop at alliance hub
        freightTool.setAllianceHubDrop();
        sleep(2000);

        //prep for level 1

        //prep for level 2

        //prep for tape drive

        //back to move
        sleep(5000);
        freightTool.moveTo0();
        sleep(5000);
    }
}
