package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveAuto;
import org.firstinspires.ftc.teamcode.ExplosivesUtils.Robot;

@Autonomous(name = "Test Auto")
public class TestAuto extends ExplosiveAuto {

    @Override
    protected void initialize() {

    }

    @Override
    protected void begin() throws InterruptedException {

//        robot.leftSide(1.0);

        waitMillis(200);
        robot.stop();
//
//        robot.drive(0.8);
//
//        while(robot.leftEncoder()<1000 && robot.rightEncoder()<1000) {
//
//        }

        robot.turnToAngle(180);

        robot.stop();

    }
}
