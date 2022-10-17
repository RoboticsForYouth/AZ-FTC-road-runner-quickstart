package org.firstinspires.ftc.teamcode.powerplay.test;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class SliderTest2 extends LinearOpMode{
    private ElapsedTime runTime = new ElapsedTime();
    private DcMotor slider = null;
    private Servo elbow = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        slider = hardwareMap.get(DcMotor.class, "slider");
        elbow = hardwareMap.get(Servo.class, "elbow");

        slider.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runTime.reset();
//        boolean sliderForward = false;
//        boolean elbowForward = false;
        while(opModeIsActive()) {
            if (gamepad1.x) {
                setMotorTargetPosition(slider, 1000, 1);
                elbow.setPosition(0.3);
                telemetry.addLine("x pressed");
                //sliderForward = true;
            } else if (gamepad1.y) {
                setMotorTargetPosition(slider, 0, 1);
                telemetry.addLine("y pressed");
                //sliderForward = false;
            }
            if (gamepad1.a) {
                elbow.setPosition(0.6);
                telemetry.addLine("a pressed");
                //elbowForward = true;
            } else if (gamepad1.b) {
                elbow.setPosition(0);
                setMotorTargetPosition(slider,0,1);
                telemetry.addLine("b pressed");
                //elbowForward = false;
            }
//            telemetry.addData("elbowForward", elbowForward);
//            telemetry.addData("sliderForward", sliderForward);
            telemetry.addData("gamepad", gamepad1);
            telemetry.addData("motor pos", slider.getCurrentPosition());

            telemetry.update();

        }
    }
    public void setMotorTargetPosition(DcMotor motor, int pos, double power) {
        motor.setTargetPosition(pos);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }
}
