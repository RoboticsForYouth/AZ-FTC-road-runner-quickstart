package org.firstinspires.ftc.teamcode.freightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Carousel extends LinearOpMode {

    CRServo carouselTool;
    private ElapsedTime runtime = new ElapsedTime();
    private static final double DUCK_POWER = 0.3;
    private static final double OG_POWER = 0.1;

    LinearOpMode opMode;
    int first = 1;
    int second = 1;
    int next;

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

    public void rampDrop() {
        // 1 1 2 3 5 8
        for(int i = 0; i < 6; i++) {
            next = first + second;
            carouselTool.setPower(OG_POWER * second);
            sleep(600);
            first = second;
            second = next;
        }
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