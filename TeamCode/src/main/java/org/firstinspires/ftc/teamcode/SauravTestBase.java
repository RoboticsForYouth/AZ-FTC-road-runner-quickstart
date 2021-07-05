/*
Copyright 2019 FIRST Tech Challenge Team 12096

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.lang.Math;



public class SauravTestBase {
    
    private ElapsedTime runtime = new ElapsedTime();
    DcMotorEx bLeft, fLeft, bRight, fRight;
    LinearOpMode linearOpMode;
    
    public SauravTestBase(LinearOpMode opMode) {
        this.linearOpMode = opMode;
        bLeft  = opMode.hardwareMap.get(DcMotorEx.class, "leftBack");
        fLeft = opMode.hardwareMap.get(DcMotorEx.class, "leftFront");
        bRight  = opMode.hardwareMap.get(DcMotorEx.class, "rightBack");
        fRight = opMode.hardwareMap.get(DcMotorEx.class, "rightFront");
        
        bLeft.setDirection(DcMotor.Direction.REVERSE);
        bRight.setDirection(DcMotor.Direction.FORWARD);
        fLeft.setDirection(DcMotor.Direction.REVERSE);
        fRight.setDirection(DcMotor.Direction.FORWARD);

        this.runtime.reset();
    }
    
    public void stop() {
        fLeft.setVelocity(0.0);
        bLeft.setVelocity(0.0);
        fRight.setVelocity(0.0);
        bRight.setVelocity(0.0);
    }
    
    public void setPowers(double bl, double fl, double br, double fr) {
        bLeft.setVelocity(bl);
        fLeft.setVelocity(fl);
        bRight.setVelocity(br);
        fRight.setVelocity(fr);
    }
    
    public void moveY(double speed) {
        bLeft.setVelocity(speed);
        fLeft.setVelocity(speed);
        bRight.setVelocity(speed);
        fRight.setVelocity(speed);
    }
    
    public void turn(double speed) {
        bLeft.setVelocity(-speed);
        fLeft.setVelocity(-speed);
        bRight.setVelocity(speed);
        fRight.setVelocity(speed);
    }
    
}
