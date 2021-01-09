package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveAuto;
import org.firstinspires.ftc.teamcode.ExplosivesUtils.Robot;

@Autonomous(name = "Test Auto")
public class TestAuto extends ExplosiveAuto {

    Robot robot;

    @Override
    protected void initialize() {

    }

    @Override
    protected void begin() throws InterruptedException {
        robot.driveStraight(1.0,2500);
        wait(1000);
        robot.turn(90,1.0);
        wait(1000);
        robot.turn(-90,1.0);
        wait(1000);
        robot.driveStraight(-1.0,2500);
    }
}
