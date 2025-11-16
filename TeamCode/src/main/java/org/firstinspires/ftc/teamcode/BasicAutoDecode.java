package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Basic Autonomous Decode", group="Use This")
public class BasicAutoDecode extends OpMode {
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
    private ElapsedTime timer = new ElapsedTime();
    private boolean movementComplete = false; // to run the movement only once


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

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

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
        public void loop() {
            double moveTime = 2.0; // seconds

            if (!movementComplete) {
                mecanum_drivetrain(0, 1, 0); // strafe left
                if (timer.seconds() >= moveTime) {
                    // Stop the robot after moveTime seconds
                    mecanum_drivetrain(0, 0, 0);
                    movementComplete = true;
                }
            }

            general_telemetry();
        }
    }

    public void mecanum_drivetrain(float y, float x, float r) {
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