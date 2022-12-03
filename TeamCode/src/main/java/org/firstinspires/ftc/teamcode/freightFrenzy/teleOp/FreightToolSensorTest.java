package org.firstinspires.ftc.teamcode.freightFrenzy.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.freightFrenzy.tools.AZUtil;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.FreightSensor;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.FreightTool;

//@AutonomouseOp
public class FreightToolSensorTest extends LinearOpMode {
    FreightTool freightTool = null;
    FreightSensor freightSensor = null;

    public void setUp() {
        freightTool = new FreightTool(this);
        freightSensor = new FreightSensor(this);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        setUp();
        waitForStart();
        freightTool.intake();
        while(true){
            AZUtil.print(telemetry,"distance", freightSensor.getSensorDistance());

        }
    }
}
