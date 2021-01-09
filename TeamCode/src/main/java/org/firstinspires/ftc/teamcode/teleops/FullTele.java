package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveTele;

@TeleOp(name = "FullTele")
public class FullTele extends ExplosiveTele {

    @Override
    protected void initialize() {

    }

    @Override
    protected void looping() {

        if(Math.abs(gamepad1.left_stick_y) > 0.2 || Math.abs(gamepad1.left_stick_x) > 0.2 || Math.abs(gamepad1.right_stick_x) > 0.2) {
            robot.fright.setPower(0.6*((gamepad1.left_stick_y+gamepad1.left_stick_x) - gamepad1.right_stick_x));
            robot.bright.setPower(0.6*((gamepad1.left_stick_y-gamepad1.left_stick_x) - gamepad1.right_stick_x));
            robot.fleft.setPower(0.6*((gamepad1.left_stick_y-gamepad1.left_stick_x) + gamepad1.right_stick_x));
            robot.bleft.setPower(0.6*((gamepad1.left_stick_y+gamepad1.left_stick_x) + gamepad1.right_stick_x));
        } else {
            robot.stop();
        }

        if(Math.abs(gamepad2.right_trigger)>0.2) {
            robot.shoot(-gamepad2.right_trigger);
        } else if(Math.abs(gamepad2.left_trigger)>0.2) {
            robot.shoot(gamepad2.left_trigger);
        } else {
            robot.shoot(0);
        }

        if(gamepad2.a) {
            robot.intake();
        } else if (gamepad2.y) {
            robot.outtake();robot.outtake();
        } else {
            robot.stopIntake();
        }

        if(gamepad2.dpad_up) {
            robot.conveyor(-1);
        } else if(gamepad2.dpad_down) {
            robot.conveyor(1);
        } else {
            robot.conveyor(0);
        }

    }
}
