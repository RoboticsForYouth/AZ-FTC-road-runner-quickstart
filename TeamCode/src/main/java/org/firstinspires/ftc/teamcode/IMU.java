package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.Func;

public class IMU {

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    LinearOpMode opMode;
    Telemetry telemetry;
    Base base;
    
    
    public IMU(LinearOpMode opMode, Base base ){
        this.opMode = opMode;
        telemetry = opMode.telemetry;
        this.base = base;
        init(opMode);
    }
    public void init(LinearOpMode opMode){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu 1");
        imu.initialize(parameters);

        composeTelemetry();
        
    }
    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
                {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity  = imu.getGravity();
                }
            });

        telemetry.addLine()
            .addData("status", new Func<String>() {
                @Override public String value() {
                    return imu.getSystemStatus().toShortString();
                    }
                })
            .addData("calib", new Func<String>() {
                @Override public String value() {
                    return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
            .addData("heading", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
            .addData("roll", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
            .addData("pitch", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                })
            .addData("angle", new Func<Double>() {
                @Override public Double value() {
                    return getAngle();
                    }
                });
                
        

    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    public void resetAngle()
    {
        //angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
    
    public double getAngle() {
        
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle = angles.firstAngle;
        
        if(angle < 0) {
            angle = 180 + (180 + angle);
        } 
        
        return angle;
    }
    
    public void rotate(int degrees, int error, double power)
    {
        
        // restart imu movement tracking.
        resetAngle();
        double ang = getAngle();
        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).
        if (degrees > 180)
        {   // turn right.
            base.turnRight(power);
        }
        else if (degrees > 0 && degrees <= 180)
        {   // turn left.
            base.turnLeft(power);
        } else if (degrees == 0) {
            if (ang > 180) {
                base.turnLeft(power);
            } else {
                base.turnRight(power);
            }
        }
        
        else return;
        opMode.telemetry.update();
        // rotate until turn is completed.
        if (degrees > 180)
        {
            // On right turn we have to get off zero first.
            
            while (opMode.opModeIsActive() && getAngle() == 0) {
                opMode.sleep(50);
                }
            while (opMode.opModeIsActive() && Math.abs(getAngle() - (degrees-error)) > 5) {
                //opMode.telemetry.update(); 
                opMode.sleep(50);
                
            }
        }
        else    // left turn.
            while (opMode.opModeIsActive() && Math.abs(getAngle() - (degrees-error)) > 5) {
            opMode.telemetry.update();
                
            }
        
        // turn the motors off.
        base.stopRobot();
        telemetry.addLine("exited loop: " + getAngle() + "; " + (degrees-error));
        opMode.telemetry.update();

        //resetAngle();
    }
    
    public void rotate2(int degrees, int error, double power)
    {
        
        double ang = getAngle();
        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).
        if (degrees > 0)
        {   // turn right.
            base.turnRight(power);
        }
        else {   // turn left.
            base.turnLeft(power);
        } 
        
        opMode.telemetry.update();
        // rotate until turn is completed.
        if (degrees > 180)
        {
            // On right turn we have to get off zero first.
            
            while (opMode.opModeIsActive() && getAngle() == 0) {
                opMode.sleep(50);
                }
            while (opMode.opModeIsActive() && Math.abs(getAngle() - (degrees-error)) > 5) {
                //opMode.telemetry.update(); 
                opMode.sleep(50);
                
            }
        }
        else    // left turn.
            while (opMode.opModeIsActive() && Math.abs(getAngle() - (degrees-error)) > 5) {
            opMode.telemetry.update();
                
            }
        
        // turn the motors off.
        base.stopRobot();
        telemetry.addLine("exited loop: " + getAngle() + "; " + (degrees-error));
        opMode.telemetry.update();

        //resetAngle();
    }
    
    
    public void turnLeft90(){
        rotate(90, 0, 0.5);
    }
    
    public void turnRight90(){
        rotate(270, -6, 0.5);
    }
    // public void turnTo0(double power, double sleep) {
    //     opMode.sleep(sleep);
    //     rotate(0, 0, power);
    //     opMode.sleep(sleep);
    //     rotate(0, 0, power);
    //     opMode.sleep(sleep);
    // }
   
    
}
    
    
    
