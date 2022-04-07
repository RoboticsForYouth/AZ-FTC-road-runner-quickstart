package org.firstinspires.ftc.teamcode.freightFrenzy.auto;

import org.firstinspires.ftc.teamcode.freightFrenzy.tools.Arm;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.TurnTable;

public class AutoUtil {

    public enum AutoVars{
        BW_LEVEL1(1, -8, -23, 45, TurnTable.Direction.COUNTER_CLOCKWISE, Arm.ArmLevel.LEVEL1),
        BW_LEVEL2(2, -4, -27, 35, TurnTable.Direction.COUNTER_CLOCKWISE, Arm.ArmLevel.LEVEL2),
        LEVEL3(3, -6, -35, 25, TurnTable.Direction.CLOCKWISE, Arm.ArmLevel.LEVEL3),
        BW_LEVEL3(3, -6, -35, 25, TurnTable.Direction.COUNTER_CLOCKWISE, Arm.ArmLevel.LEVEL3),
        BW2_LEVEL3(3, -12, -36, 150, TurnTable.Direction.CLOCKWISE, Arm.ArmLevel.LEVEL3),
        BC_LEVEL1(1, -6, 23, 135,TurnTable.Direction.COUNTER_CLOCKWISE, Arm.ArmLevel.LEVEL1),
        BC_LEVEL2(2, -3, 28, 130,TurnTable.Direction.COUNTER_CLOCKWISE, Arm.ArmLevel.LEVEL2),
        BC_LEVEL3(3, -6, 35, 150,TurnTable.Direction.COUNTER_CLOCKWISE, Arm.ArmLevel.LEVEL3),
        RC_LEVEL1(1, 8, 23, 45, TurnTable.Direction.COUNTER_CLOCKWISE, Arm.ArmLevel.LEVEL1),
        RC_LEVEL2(2, 4, 27, 35,TurnTable.Direction.COUNTER_CLOCKWISE, Arm.ArmLevel.LEVEL2),
        RC_LEVEL3(3, 7, 34, 20,TurnTable.Direction.COUNTER_CLOCKWISE, Arm.ArmLevel.LEVEL3),
        RW_LEVEL1(1, -5, 23, 135,TurnTable.Direction.COUNTER_CLOCKWISE, Arm.ArmLevel.LEVEL1),
        RW_LEVEL2(2, -3, 28, 125,TurnTable.Direction.COUNTER_CLOCKWISE, Arm.ArmLevel.LEVEL2),
        RW_LEVEL3(3, -6, 35, 150,TurnTable.Direction.COUNTER_CLOCKWISE, Arm.ArmLevel.LEVEL3);
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
