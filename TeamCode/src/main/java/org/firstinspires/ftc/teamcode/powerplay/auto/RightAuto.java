package org.firstinspires.ftc.teamcode.powerplay.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Size;

@Autonomous
public class RightAuto extends Auto {

    public RightAuto() {
        super();
        setFieldPos(FieldPos.RIGHT);
    }

    @Override
    public Rect getSleeveDetectionBoundingBox(){
        return new Rect(new Point(300, 149), new Size(60, 90));
    }
}


