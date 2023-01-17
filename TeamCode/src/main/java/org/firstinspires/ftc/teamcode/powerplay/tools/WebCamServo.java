package org.firstinspires.ftc.teamcode.powerplay.tools;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class WebCamServo extends LinearOpMode {
    LinearOpMode opMode;
    private Servo webcamServo;

    public WebCamServo(LinearOpMode opMode) {
        this.opMode = opMode;
        setup();
    }

    public WebCamServo() {
        super();
    }

    private void setup() {
        webcamServo = opMode.hardwareMap.get(Servo.class, "webcamServo");
        webcamServo.setPosition(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        this.opMode = this;
        setup();

        waitForStart();
        sleep(5000);

    }
}
