package org.firstinspires.ftc.teamcode.freightFrenzy.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "sample")

public class RLeftCarouselWarehouse extends RedAutoBase {

    @Override
    public void runOpMode() throws InterruptedException {
        initAuto();
        waitForStart();
        if (isStopRequested()) return;
        setupPos();
        detection();
        moveToDrop("left");
        moveToCarousel("left");
        moveToPark("left", "warehouse");
    }
}
