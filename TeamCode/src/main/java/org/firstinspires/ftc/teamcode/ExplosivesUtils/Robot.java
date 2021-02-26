package org.firstinspires.ftc.teamcode.ExplosivesUtils;

import android.graphics.drawable.DrawableContainer;
import android.graphics.drawable.GradientDrawable;
import android.hardware.Sensor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;
import java.util.HashMap;

public class Robot {

    public DcMotor intake, conveyor;
    public DcMotorEx shootL, shootR, fleft, fright, bright, bleft;
    public ModernRoboticsI2cRangeSensor sonic;
    public Servo wobbler, grabber;

    BNO055IMU imu;

    OpMode opMode;

    HardwareMap hardwareMap;

    public enum Direction {
        LEFT, RIGHT
    }

    public Robot(HardwareMap hardwareMap, OpMode opMode) {
        this.hardwareMap=hardwareMap;
        this.opMode=opMode;
        init();
    }

    /*
        Create all the hardware objects and set them to their respective variables. Called upon initialization of the class.
     */
    private void init() {

        initGyro();

        // Initialize all the variables
        fleft = hardwareMap.get(DcMotorEx.class, "fleft");
        fright = hardwareMap.get(DcMotorEx.class, "fright");
        bleft = hardwareMap.get(DcMotorEx.class, "bleft");
        bright = hardwareMap.get(DcMotorEx.class, "bright");

        shootL = hardwareMap.get(DcMotorEx.class, "shootL");
        shootR = hardwareMap.get(DcMotorEx.class, "shootR");

        intake = hardwareMap.get(DcMotor.class, "intake");

        conveyor = hardwareMap.get(DcMotor.class, "conveyor");

        wobbler = hardwareMap.get(Servo.class, "wobbler");
        grabber = hardwareMap.get(Servo.class, "grabber");

        sonic = hardwareMap.get(ModernRoboticsI2cRangeSensor.class,"sonic");

        bright.setDirection(DcMotorSimple.Direction.REVERSE);
        fright.setDirection(DcMotorSimple.Direction.REVERSE);

        fright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        conveyor.setDirection(DcMotorSimple.Direction.REVERSE);
        shootL.setDirection(DcMotorSimple.Direction.REVERSE);

        shootLPID = new PIDController(0.35,0,0);
        shootRPID = new PIDController(0.35,0,0);

        shootL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        PIDFCoefficients coefficients = new PIDFCoefficients(0.1,0.001,0.0001,0);
//
//        shootL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,coefficients);
//        shootR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,coefficients);

//        wobbler.setPosition(0.0);
        closeGrabber();
        liftWobbler();

    }

    /*
        Returns an ArrayList of SensorData objects that should represent every used sensor on the robot.
     */
    public ArrayList<SensorData> getSensorData() {
        ArrayList<SensorData> list = new ArrayList<>();
        list.add(new SensorData("gyro heading", getHeading()));
        list.add(new SensorData("fleft encoder", fleft.getCurrentPosition()));
        list.add(new SensorData("fright encoder", fright.getCurrentPosition()));
        list.add(new SensorData("bleft encoder", bleft.getCurrentPosition()));
        list.add(new SensorData("bright encoder", bright.getCurrentPosition()));
        list.add(new SensorData("leftside enc", leftEncoder()));
        list.add(new SensorData("rightside end", rightEncoder()));
        list.add(new SensorData("shootL encoder", shootL.getCurrentPosition()));
        list.add(new SensorData("shootR encoder", shootR.getCurrentPosition()));
        list.add(new SensorData("sonic", sonic.getDistance(DistanceUnit.INCH)));
        return list;
    }

    long lastShooterCheckTime=0;
    double lastShootLEncoder, lastShootREncoder;

    PIDController shootLPID, shootRPID;

    private void shoot(int ticksPerSecond) {
        shootL.setVelocity(-ticksPerSecond);
        shootR.setVelocity(-ticksPerSecond);
    }

    public void shoot(ShooterSpeed speed) {
        switch (speed) {
            case FULL_FORWARD:
                shootL.setVelocity(-1950);
                shootR.setVelocity(-1850);
                break;
            case FULL_BACKWARD:
                shootL.setVelocity(2000);
                shootR.setVelocity(2000);
                break;
            case AUTO_PS:
                //Original -1950, -1850
                shootL.setVelocity(-850);
                shootR.setVelocity(-750);
                break;
            case STOP:
                shootL.setVelocity(0);
                shootR.setVelocity(0);
                break;
            case AUTO_TOWERGOAL:
                shootL.setVelocity(-850);
                shootR.setVelocity(-850);
                break;
        }
    }

    public void stopShooter() {
        shootL.setVelocity(0);
        shootR.setVelocity(0);
    }

    public enum ShooterSpeed {
        FULL_FORWARD,AUTO_PS,FULL_BACKWARD,STOP,AUTO_TOWERGOAL;
    }

    public double roundToDigit(double num, int places) {
        double scale = Math.pow(10, places);
        return Math.round(num * scale) / scale;
    }

    public void intake() {
        intake.setPower(1.0);
    }

    public void outtake() {
        intake.setPower(-1.0);
    }

    public void stopIntake() {
        intake.setPower(0.0);
    }

    public void stop() {
        drive(0);
    }

    public void conveyor(double speed) {
        conveyor.setPower(speed);
    }

    public void openGrabber() {
        grabber.setPosition(0.7);
    }

    public void closeGrabber() {
        grabber.setPosition(0.993);
    }

    public void dropWobbler() {
        wobbler.setPosition(0.863);
    }

    public void liftWobbler() {
        wobbler.setPosition(0.25);
    }

    public void middleWobbler() {
        wobbler.setPosition(0.7);
    }

    public void satisfyingWobblerDrop() {
        while(wobbler.getPosition()<0.7) {
            middleWobbler();
        }

        openGrabber();
        waitMillis(800);
        closeGrabber();
        liftWobbler();
    }

    public void drive(double speed) {

        speed = -speed;

        if(speed < -1 || speed > 1) { return; }

        fleft(speed);
        fright(speed);
        bleft(speed);
        bright(speed);

    }

    public void turn(double speed, Direction direction) {
        speed = Math.abs(speed);
        if(direction == Direction.LEFT) {
            rightSide(speed);
            leftSide(-speed);
        } else {
            rightSide(-speed);
            leftSide(speed);
        }
    }

    public void turn(double speed) {
        rightSide(-speed);
        leftSide(speed);
    }

    public void turn(int degrees, double speed) {

        long startTime = System.currentTimeMillis();

        double initialAngle = getHeading();
        double targetAngle = getHeading()+degrees;

        // If the degrees is positive, sign is 1; else -1
        int sign = degrees>0 ? 1 : -1;

        double diff = getHeading()-targetAngle;
        double calcSpeed = speed;

        while(diff >= 2 || System.currentTimeMillis()-startTime>TURN_TIMEOUT) {

            // Calculate speed that each side should go at
            if(diff>25) {
                calcSpeed=speed;
            } else if (diff>15) {
                calcSpeed=speed*0.6;
            } else if (diff>5) {
                calcSpeed=speed*0.4;
            } else {
                calcSpeed=speed*0.2;
            }

            if(calcSpeed<=MIN_TURN_SPEED) {
                calcSpeed=MIN_TURN_SPEED;
            }

            leftSide(sign*calcSpeed);
            rightSide(-sign*calcSpeed);

            diff = getHeading()-targetAngle;
        }

    }

    public double getSonic() {
        return sonic.getDistance(DistanceUnit.INCH);
    }

    private void initGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        while(!imu.isGyroCalibrated()) {
            //Wait
        }

    }

    public void leftSide(double speed) {
        bleft(speed);
        fleft(speed);
    }

    public void rightSide(double speed) {
        bright(speed);
        fright(speed);
    }

    public Orientation getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public double getHeading() {
        return getAngle().firstAngle;
    }

    public void newDriveStraight(double speed, int ticks) {
//        resetEncoders();

        drive(0.5);

        int count=0;

        while(leftEncoder()<ticks && rightEncoder()<ticks) {
            opMode.telemetry.addLine(" " + count);
            opMode.telemetry.addLine("leftEnc: " + leftEncoder());
            opMode.telemetry.addLine("rightEnc: " + rightEncoder());
            opMode.telemetry.update();
            count++;
        }

        stop();
    }

    public void driveStraight(double speed, int ticks) {
//        resetEncoders();
        speed= -speed;
        long startTime = System.currentTimeMillis();
        double leftMult = 1;
        double rightMult = 1;
        while(System.currentTimeMillis()-startTime<=15000) {

            opMode.telemetry.addLine("leftEnc: " + leftEncoder());
            opMode.telemetry.addLine("rightEnc: " + rightEncoder());
            opMode.telemetry.addLine("leftMult: " + leftMult);
            opMode.telemetry.addLine("rightMult: " + rightMult);
            opMode.telemetry.update();

            leftSide(speed);
            rightSide(speed);

            // If the difference between the two is less than 10 ticks, have them both move at the same speed
            if(Math.abs(leftEncoder()-rightEncoder())<10) {
                leftMult=1;
                rightMult=1;
            } else {

                double avg = (leftEncoder()+rightEncoder())/2;

                double leftDiff = Math.abs(avg-leftEncoder());
                double rightDiff = Math.abs(avg-rightEncoder());

                if(leftEncoder()>rightEncoder()) {
                    rightMult=1.1;
                    leftMult=0.9;
                } else {
                    leftMult=1.1;
                    rightMult=0.9;
                }

            }

            if(Math.abs(ticks-leftEncoder())<10) {
                leftMult=0;
            }

            if(Math.abs(ticks-rightEncoder())<10) {
                rightMult=0;
            }

//            leftSide(speed*leftMult);
//            rightSide(speed*rightMult);



            if(leftMult==0 && rightMult==0) {
                opMode.telemetry.addLine("DONE");
                opMode.telemetry.update();
                return;
            }

        }

        opMode.telemetry.addLine("DONE");
        opMode.telemetry.update();
    }

    final int ALLOWED_ERROR = 10;

    public void driveForwardEncoders(double speed, int ticks) {
        resetEncoders();
        double initL = leftEncoder();
        double initR = rightEncoder();

        double targetL = initL+ticks;
        double targetR = initR+ticks;

        double initGyro = getHeading();

        while((Math.abs(rightEncoder()-targetR) > ALLOWED_ERROR && Math.abs(leftEncoder()-targetL) > ALLOWED_ERROR)) {
            opMode.telemetry.addLine("leftPow: " + speed+(0.3*motorPowerFunction(getHeading()-initGyro)));
            opMode.telemetry.addLine("rightPow: " + speed+(-0.3*motorPowerFunction(getHeading()-initGyro)));
            opMode.telemetry.addLine("leftTurnPow: " + 0.3*motorPowerFunction(getHeading()-initGyro));
            opMode.telemetry.addLine("rightTurnPow: " + -0.3*motorPowerFunction(getHeading()-initGyro));
            opMode.telemetry.addLine("targetL: " + targetL);
            opMode.telemetry.addLine("targetR: " + targetR);
            opMode.telemetry.addLine("leftenc: " + leftEncoder());
            opMode.telemetry.addLine("rightenc: " + rightEncoder());
            opMode.telemetry.addLine("lefterr: " + Math.abs(leftEncoder()-targetL));
            opMode.telemetry.addLine("righterr: " + Math.abs(rightEncoder()-targetR));
            opMode.telemetry.update();
            leftSide(-(speed+(0.3*motorPowerFunction(getHeading()-initGyro))));
            rightSide(-(speed-(0.3*motorPowerFunction(getHeading()-initGyro))));
//            drive(10);

            if((rightEncoder()>targetR+ALLOWED_ERROR) || (leftEncoder()>targetL+ALLOWED_ERROR)) {
                break;
            }
        }

        stop();
    }

    public void driveBackwardEncoders(double speed, int ticks) {
        resetEncoders();
        double initL = leftEncoder();
        double initR = rightEncoder();

        double targetL = initL-ticks;
        double targetR = initR-ticks;

        double initGyro = getHeading();

        while((Math.abs(rightEncoder()-targetR) > ALLOWED_ERROR && Math.abs(leftEncoder()-targetL) > ALLOWED_ERROR)) {
            opMode.telemetry.addLine("leftPow: " + speed+(0.3*motorPowerFunction(getHeading()-initGyro)));
            opMode.telemetry.addLine("rightPow: " + speed+(-0.3*motorPowerFunction(getHeading()-initGyro)));
            opMode.telemetry.addLine("leftTurnPow: " + 0.3*motorPowerFunction(getHeading()-initGyro));
            opMode.telemetry.addLine("rightTurnPow: " + -0.3*motorPowerFunction(getHeading()-initGyro));
            opMode.telemetry.addLine("targetL: " + targetL);
            opMode.telemetry.addLine("targetR: " + targetR);
            opMode.telemetry.addLine("leftenc: " + leftEncoder());
            opMode.telemetry.addLine("rightenc: " + rightEncoder());
            opMode.telemetry.addLine("lefterr: " + Math.abs(leftEncoder()-targetL));
            opMode.telemetry.addLine("righterr: " + Math.abs(rightEncoder()-targetR));
            opMode.telemetry.update();
            leftSide(speed-(0.3*motorPowerFunction(getHeading()-initGyro)));
            rightSide(speed+(0.3*motorPowerFunction(getHeading()-initGyro)));
//            drive(10);

            if((rightEncoder()<targetR+ALLOWED_ERROR) || (leftEncoder()<targetL+ALLOWED_ERROR)) {
                break;
            }
        }

        stop();
    }

    public void strafe(double speed, Direction direction) {
        if(direction == Direction.LEFT) {
            fright(-speed);
            bright(speed);
            fleft(speed);
            bleft(-speed);
        } else {
            fright(speed);
            bright(-speed);
            fleft(-speed);
            bleft(speed);
        }
    }

    public void strafeEncoders(double speed, int ticks, Direction direction) {
        speed = Math.abs(speed);
        resetEncoders();
        double initL = leftEncoder();
        double initR = rightEncoder();

        double targetL = initL+ticks;
        double targetR = initR+ticks;

        double initGyro = getHeading();

        while((Math.abs(rightEncoder()-targetR) > ALLOWED_ERROR && Math.abs(leftEncoder()-targetL) > ALLOWED_ERROR)) {
            opMode.telemetry.addLine("leftPow: " + speed+(0.3*motorPowerFunction(getHeading()-initGyro)));
            opMode.telemetry.addLine("rightPow: " + speed+(-0.3*motorPowerFunction(getHeading()-initGyro)));
            opMode.telemetry.addLine("leftTurnPow: " + 0.3*motorPowerFunction(getHeading()-initGyro));
            opMode.telemetry.addLine("rightTurnPow: " + -0.3*motorPowerFunction(getHeading()-initGyro));
            opMode.telemetry.addLine("targetL: " + targetL);
            opMode.telemetry.addLine("targetR: " + targetR);
            opMode.telemetry.addLine("leftenc: " + leftEncoder());
            opMode.telemetry.addLine("rightenc: " + rightEncoder());
            opMode.telemetry.addLine("lefterr: " + Math.abs(leftEncoder()-targetL));
            opMode.telemetry.addLine("righterr: " + Math.abs(rightEncoder()-targetR));
            opMode.telemetry.update();

            fright(-speed);
            fleft(speed);
            bright(speed);
            bleft(-speed);

//            if(direction == Direction.LEFT) {
//                fright.setPower(-speed+(0*motorPowerFunction(getHeading()-initGyro)));
//                fleft.setPower(speed-(0*motorPowerFunction(getHeading()-initGyro)));
//                bright.setPower(speed+(0*motorPowerFunction(getHeading()-initGyro)));
//                bleft.setPower(-speed-(0*motorPowerFunction(getHeading()-initGyro)));
//            } else {
//                fright.setPower(speed+(0*motorPowerFunction(getHeading()-initGyro)));
//                fleft.setPower(-speed-(0*motorPowerFunction(getHeading()-initGyro)));
//                bright.setPower(-speed+(0*motorPowerFunction(getHeading()-initGyro)));
//                bleft.setPower(speed-(0*motorPowerFunction(getHeading()-initGyro)));
//            }

            if((rightEncoder()>targetR+ALLOWED_ERROR) || (leftEncoder()>targetL+ALLOWED_ERROR)) {
                break;
            }
        }

        stop();
    }

    public void resetEncoders() {
        bleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double leftEncoder() {
        return -bleft.getCurrentPosition();
//        return (bleft.getCurrentPosition()+fleft.getCurrentPosition())/2;
    }

    public double rightEncoder() {
        return bright.getCurrentPosition();
//        return (bright.getCurrentPosition()+fright.getCurrentPosition())/2;
    }

    final double MIN_TURN_SPEED = 0.2;
    final int TURN_TIMEOUT = 4000;
    final static double MAX_TURN_DIFF = 2;

    public void turnToAngle(double angle) {
        autoturn(angle);
        autoturn(angle);
    }

    private void autoturn(double angle) {
        resetEncoders();
        long startTime = System.currentTimeMillis();

        double initial = getHeading();
        double target = angle;

        double diff = calcdiff(initial,target);

        opMode.telemetry.addData(("Head: " + getHeading() + " Diff: " + diff + " Target: " + target),null);
        opMode.telemetry.update();

        double speed = 0.0;

        while(System.currentTimeMillis()<startTime+TURN_TIMEOUT &&  Math.abs(diff)>MAX_TURN_DIFF) {
            opMode.telemetry.addData(("Head: " + getHeading() + " Diff: " + diff + " Target: " + target),null);
            opMode.telemetry.addLine("speed: " + speed);
            opMode.telemetry.update();
            speed = 0.6*motorPowerFunction(diff);
            turn(speed);
            diff = calcdiff(getHeading(),target);
        }

        turn(0);

        waitMillis(100);

        diff = calcdiff(getHeading(),target);

        while(System.currentTimeMillis()<startTime+TURN_TIMEOUT/2 && Math.abs(diff)>MAX_TURN_DIFF/2) {
            opMode.telemetry.addData(("Head: " + getHeading() + " Diff: " + diff + " Target: " + target),null);
            opMode.telemetry.update();
            speed = 0.5*motorPowerFunction(diff);
            turn(speed);
            diff = calcdiff(getHeading(),target);
        }

        turn(0);

        waitMillis(50);

        diff = calcdiff(getHeading(),target);

        while(System.currentTimeMillis()<startTime+TURN_TIMEOUT &&  Math.abs(diff)>MAX_TURN_DIFF) {
            opMode.telemetry.addData(("Head: " + getHeading() + " Diff: " + diff + " Target: " + target),null);
            opMode.telemetry.addLine("speed: " + speed);
            opMode.telemetry.update();
            speed = 0.4*motorPowerFunction(diff);
            turn(speed);
            diff = calcdiff(getHeading(),target);
        }

        turn(0);

    }

    private double calcdiff(double heading, double target) {
        heading+=180;
        target+=180;

        if(target-heading>180) {
            return -(heading+(360-target));
        }

        return target-heading;
    }

    final double RIGHT_PS=23;
    final double MID_PS=27;
    final double LEFT_PS=31.5;

    public void turnToPS(PowerShotPos pos) {
        switch(pos) {
            case RIGHT:
                turnToAngle(RIGHT_PS);
                break;
            case MID:
                turnToAngle(MID_PS);
                break;
            case LEFT:
                turnToAngle(LEFT_PS);
                break;
        }
    }

    public enum PowerShotPos {
        LEFT,MID,RIGHT;
    }

    // Custom function found on Desmos to control the power of the motors based off of the given angle
    public double motorPowerFunction(double angle) {
        if(angle>0) {
            return (Math.sqrt(angle/600)+0.2)*1.2;
        } else {
            return (-Math.sqrt(-angle/600)-0.2)*1.2;
        }
    }

    // Manipulates a given angle so that it is between 0º and 360º
    public double stripAngle(double angle) {
        if(angle>360) {
            angle-=Math.floor(angle/360)*360;
        } else if (angle<-360) {
            angle+=(Math.floor(angle/360)*-360)-360;
        }
        return angle;
    }

    public void waitMillis(int millis) {
        try {
            Thread.sleep(millis);
        } catch(InterruptedException e) {
            e.printStackTrace();
        }
    }

    public double map(double targetUp, double value) {
        return targetUp*value;
    }

    final int MAX_DRIVE_VELOCITY=2700;

    public void fright(double val) {
        fright.setVelocity(map(MAX_DRIVE_VELOCITY,val));
    }

    public void fleft(double val) {
        fleft.setVelocity(map(MAX_DRIVE_VELOCITY,val));
    }

    public void bleft(double val) {
        bleft.setVelocity(map(MAX_DRIVE_VELOCITY,val));
    }

    public void bright(double val) {
        bright.setVelocity(map(MAX_DRIVE_VELOCITY,val));
    }

}
