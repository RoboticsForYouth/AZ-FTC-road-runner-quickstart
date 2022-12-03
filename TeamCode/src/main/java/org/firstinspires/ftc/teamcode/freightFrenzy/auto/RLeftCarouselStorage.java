package org.firstinspires.ftc.teamcode.freightFrenzy.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Autonomous(group = "sample")

public class RLeftCarouselStorage extends RedAutoBase {

    @Override
    public void runOpMode() throws InterruptedException {
        initAuto();
        waitForStart();
        if (isStopRequested()) return;
        setupPos();
        detection();
        sleep(3000);
        moveToDrop("left");
        moveToCarousel("left");
        moveToPark("left", "storage");
    }
}
