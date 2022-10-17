package org.firstinspires.ftc.teamcode.powerplay.test;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class SliderTest extends LinearOpMode {
    private DcMotor slider = null;
    private Servo elbow = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        elbow = hardwareMap.get(Servo.class, "elbow");
        slider = hardwareMap.get(DcMotor.class, "slider");

        slider.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        elbow.setPosition(0);
        sleep(5000);
        setMotorTargetPosition(slider, 8000, 1);
        sleep(5000);
        elbow.setPosition(0.4);
        sleep(5000);
        elbow.setPosition(0);
        sleep(5000);
        setMotorTargetPosition(slider, 0, 0.6);
        sleep(5000);
    }
    public void setMotorTargetPosition(DcMotor motor, int pos, double power) {
        motor.setTargetPosition(pos);
        motor.setPower(power);
    }
}
