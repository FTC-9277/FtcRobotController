package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveAuto;
import org.firstinspires.ftc.teamcode.ExplosivesUtils.Robot;

@Autonomous(name = "Shoot")
public class ShootAuto extends ExplosiveAuto {
    @Override
    protected void initialize() {

    }

    @Override
    protected void begin() throws InterruptedException {
        robot.drive(1);
        waitMillis(625);
        robot.strafe(0.5, Robot.Direction.LEFT);
        waitMillis(500);
        robot.stop();

//        robot.turn(0.2, Robot.Direction.LEFT);
        waitMillis(100);

        robot.stop();

        robot.shoot(-1);
        waitMillis(2000);

        robot.conveyor(-1.0);
        robot.intake();
        waitMillis(3000);

        robot.stop();
        waitMillis(1000);

        robot.drive(0.5);
        waitMillis(200);

        robot.stop();
    }
}
