package org.firstinspires.ftc.teamcode.freightFrenzy.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Autonomous(group = "sample")

public class BRightCarouselStorage extends BlueAutoBase {

    @Override
    public void runOpMode() throws InterruptedException {
        initAuto();
        waitForStart();
        if (isStopRequested()) return;
        setupPos();
        detection();
        sleep(3000);
        moveToDrop("right");
        moveToCarousel("right");
        moveToPark("right", "storage");
    }
}
