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

        robot.driveForwardEncoders(0.6,1100);

        waitMillis(500);

        robot.turnToAngle(90);

        waitMillis(500);

        robot.driveForwardEncoders(0.6,100);
        robot.stop();

        waitMillis(500);

        robot.turnToAngle(0);

        robot.shoot(1850);
        waitMillis(2000);

        robot.conveyor(-1.0);
        robot.intake();
        waitMillis(5000);

        robot.stop();
        waitMillis(1000);

        robot.stopIntake();
        robot.conveyor(0);
        robot.shoot(0);

        waitMillis(250);

        robot.driveForwardEncoders(0.6,250);

        robot.turnToAngle(180);

        waitMillis(500);

        robot.dropWobbler();
        waitMillis(1000);

        robot.openGrabber();
        waitMillis(500);

        robot.liftWobbler();
        waitMillis(500);
        robot.closeGrabber();

        robot.stop();

    }
}
