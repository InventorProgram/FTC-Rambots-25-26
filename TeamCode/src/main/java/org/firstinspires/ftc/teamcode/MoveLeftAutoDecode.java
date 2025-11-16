package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Move Left Auto Decode", group="Use This")
public class MoveLeftAutoDecode extends OpMode {
    final double FEED_TIME_SECONDS = 0.20;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;

    final double LAUNCHER_TARGET_VELOCITY = 1500;
    final double LAUNCHER_MIN_VELOCITY = 1450;
    final double STRAFING_CORRECTION = 1.1;

    private DcMotorEx launcher = null;
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;

    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    double frontLeftPower = 0;
    double frontRightPower = 0;
    double backLeftPower = 0;
    double backRightPower = 0;

    double moveTime = 5.0; // seconds

    private ElapsedTime timer = new ElapsedTime();
    private boolean hasMoved = false; // to run the movement only once

    private enum Direction {LEFT, RIGHT, FORWARD, BACKWARD}
    private enum Alliance { RED, BLUE }

    private Alliance alliance = Alliance.RED;

    @Override
    public void init() {

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(BRAKE);
        frontRightMotor.setZeroPowerBehavior(BRAKE);
        backLeftMotor.setZeroPowerBehavior(BRAKE);
        backRightMotor.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        // --- RESTORED FEEDER LOGIC ---
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
        // --- END FEEDER LOGIC RESTORE ---

        telemetry.addData("Status", "Initialized");

        timer.reset();
    }

    @Override
    public void init_loop() {
        if (gamepad2.square){
            alliance = Alliance.RED;
        }
        else if (gamepad2.circle) alliance = Alliance.BLUE;

        telemetry.addData("Press circle", "for BLUE");
        telemetry.addData("Press square", "for RED");
        telemetry.addData("Selected Alliance", alliance);
    }

    @Override
    public void loop() {
        if (!hasMoved) {
            move(Direction.LEFT);
            // Stop after moveTime seconds
            if (timer.seconds() >= moveTime) {
                mecanum_drivetrain(0, 0, 0); // stop robot
                hasMoved = true;
            }
        }

        general_telemetry();
    }

    public void mecanum_drivetrain(double y, double x, double r) {
        // --- Controller Variables ---
        //double y = gamepad2.left_stick_y;
        //double x = -gamepad2.left_stick_x * STRAFING_CORRECTION; //This correction empowers the strafing
        //double r = -gamepad2.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1); //This is used to scale power variables to [-1,1]

        // --- The Hallowed Mecanum Equations ---
        frontLeftPower = (y + x + r) / denominator;
        frontRightPower = (y - x - r) / denominator;
        backLeftPower = (y - x + r) / denominator;
        backRightPower = (y + x - r) / denominator;

        // --- Set Power ---
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    public void move(Direction direction) {
        switch (direction) {
            case LEFT:
                mecanum_drivetrain(0, 0.5, 0); // strafe left
                break;
            case RIGHT:
                mecanum_drivetrain(0, -0.5, 0); // strafe right
                break;
            case FORWARD:
                mecanum_drivetrain(0.5, 0, 0); // move forward
                break;
            case BACKWARD:
                mecanum_drivetrain(-0.5, 0, 0); // move backward
                break;
        }
    }

    public void general_telemetry() {
        telemetry.addData("triangle: ", gamepad2.triangle);
        telemetry.addData("cross: ", gamepad2.cross);
        telemetry.addData("square: ", gamepad2.square);
        telemetry.addData("circle: ", gamepad2.circle);
        telemetry.addData("left stick x: ", gamepad2.left_stick_x);
        telemetry.addData("left stick y: ", gamepad2.left_stick_y);
        telemetry.addData("right stick x: ", gamepad2.right_stick_x);

        telemetry.addData("Motors", "FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f",
                frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        telemetry.addData("motorSpeed", launcher.getVelocity());
    }
}