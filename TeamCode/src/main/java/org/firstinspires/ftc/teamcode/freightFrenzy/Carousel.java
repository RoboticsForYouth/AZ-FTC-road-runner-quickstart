package org.firstinspires.ftc.teamcode.freightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;


public class Carousel extends LinearOpMode {

    CRServo carouselTool;
    private static final double DUCK_POWER = 0.3;
    LinearOpMode opMode;

    public void setup() {
        carouselTool = opMode.hardwareMap.get(CRServo.class, "duckyTool");
    }

    public Carousel() {
        opMode = this;
    }
    public Carousel(LinearOpMode opMode) {
        this.opMode = opMode;
        setup();
    }
    public void blueDuckyDrop() {
        carouselTool.setPower(DUCK_POWER);
        sleep(3500);
        carouselTool.setPower(0);
    }
    public void redDuckyDrop() {
        carouselTool.setPower(-DUCK_POWER);
        sleep(3500);
        carouselTool.setPower(0);
    }


    @Override
    public void runOpMode(){
        setup();
        waitForStart();

    }

    public void stopDuckyTool() {
        carouselTool.setPower(0);
    }
}