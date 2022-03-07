package org.firstinspires.ftc.teamcode.freightFrenzy.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "sample")

public class RRightCarouselWarehouse extends RedAutoBase {

    @Override
    public void runOpMode() throws InterruptedException {
        initAuto();
        waitForStart();
        if (isStopRequested()) return;
        setupPos();
        detection();
        moveToDrop("right");
        moveToCarousel("right");
        moveToPark("right", "warehouse");

    }
}
