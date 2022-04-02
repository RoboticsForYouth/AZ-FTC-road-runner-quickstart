package org.firstinspires.ftc.teamcode.freightFrenzy.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.freightFrenzy.tools.Intake;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.Wrist;

@Autonomous
public class DuckIntakeTest extends LinearOpMode {
    Intake intake = null;
    Wrist wrist = null;
    LinearOpMode opMode = null;
    @Override
    public void runOpMode() throws InterruptedException {
        setup();
        waitForStart();
        wrist.intakePos();
        intake.intake();
        sleep(4000);
//        wrist.setSecuredPos();
//        sleep(2000);

        intake.stopIntake();
        sleep(4000);

        intake.reverseIntake();
        sleep(5000);

    }

    private void setup() {
        intake = new Intake(this);
        wrist = new Wrist(this);


    }
}
