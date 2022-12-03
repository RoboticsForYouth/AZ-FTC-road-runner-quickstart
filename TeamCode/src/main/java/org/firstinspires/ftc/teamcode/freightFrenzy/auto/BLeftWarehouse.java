package org.firstinspires.ftc.teamcode.freightFrenzy.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Autonomous(group = "sample")

public class BLeftWarehouse extends BlueAutoBase {

    @Override
    public void runOpMode() throws InterruptedException {
        initAuto();
        waitForStart();
        if (isStopRequested()) return;
        setupPos();
        detection();
        sleep(4000);
        moveToDrop("left");
        moveToPark("left", "warehouse");
    }
}
