package org.firstinspires.ftc.teamcode.powerplay.teleOp;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class BluePowerPlayTeleOp extends RedPowerPlayTeleOp{
    public int getConeColor() {
        return Color.BLUE;
    }
}
