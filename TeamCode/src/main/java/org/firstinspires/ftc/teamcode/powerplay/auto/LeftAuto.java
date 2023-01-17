package org.firstinspires.ftc.teamcode.powerplay.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Size;

@Autonomous(group = "sample")
public class LeftAuto extends Auto {

    public LeftAuto() {
        super();
        setFieldPos(FieldPos.LEFT);
    }

    @Override
    public Rect getSleeveDetectionBoundingBox(){
        return new Rect(new Point(320, 149), new Size(60, 90));
    }

}


