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
//        waitMillis(1000);
//        robot.turnToAngle(0);
//        waitMillis(3000);
        robot.turnToAngle(90);
        waitMillis(3000);
        log("Going next now");
        waitMillis(3000);
        robot.turnToAngle(0);
    }
}
