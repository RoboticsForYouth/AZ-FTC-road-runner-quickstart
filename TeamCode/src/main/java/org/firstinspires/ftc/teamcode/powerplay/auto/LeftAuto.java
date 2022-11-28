package org.firstinspires.ftc.teamcode.powerplay.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "sample")
public class LeftAuto extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        initAuto();
        waitForStart();
        if (isStopRequested()) return;
        setupPos();
       // detection();
        sleep(3000);
        dropCone(true); //right
    }
}


