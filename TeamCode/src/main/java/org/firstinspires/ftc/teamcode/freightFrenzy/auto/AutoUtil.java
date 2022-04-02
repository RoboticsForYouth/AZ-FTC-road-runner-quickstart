package org.firstinspires.ftc.teamcode.freightFrenzy.auto;

import org.firstinspires.ftc.teamcode.freightFrenzy.tools.Arm;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.TurnTable;

public class AutoUtil {

    public enum AutoVars{
        LEVEL1(1, 0, 30, 150, TurnTable.Direction.COUNTER_CLOCKWISE, Arm.ArmLevel.LEVEL1),
        LEVEL2(1, -4, 34, 150, TurnTable.Direction.COUNTER_CLOCKWISE, Arm.ArmLevel.LEVEL2),
        LEVEL3(1, -6, 35, 150, TurnTable.Direction.COUNTER_CLOCKWISE, Arm.ArmLevel.LEVEL3),
        BLUE_LEVEL3(1, -6, -35, 25, TurnTable.Direction.COUNTER_CLOCKWISE, Arm.ArmLevel.LEVEL3),
        BLUE2_LEVEL3(1, -6, -35, 150, TurnTable.Direction.CLOCKWISE, Arm.ArmLevel.LEVEL3);

        public int turnTableAngle;
        private TurnTable.Direction direction;
        public Arm.ArmLevel level;
        public int tsePos;
        public int initX;
        public int initY;

        AutoVars(int tsePos, int initX, int initY, int turnTableAngle, TurnTable.Direction direction, Arm.ArmLevel level) {

            this.tsePos = tsePos;
            this.initX = initX;
            this.initY = initY;
            this.turnTableAngle = turnTableAngle;
            this.direction = direction;
            this.level = level;
        }

        public TurnTable.Direction getDirection() {
            return direction;
        }

        public int getTurnTableAngle() {
            return turnTableAngle;
        }

        public int getTsePos() {
            return tsePos;
        }

        public int getInitX() {
            return initX;
        }

        public Arm.ArmLevel getLevel() {
            return level;
        }

        public int getInitY() {
            return initY;
        }
    }
}
