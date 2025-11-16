package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Teleop Decode", group = "Use This")
public class TeleopDecode extends OpMode {
    final double FEED_TIME_SECONDS = 0.20;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;

    double LAUNCHER_TARGET_VELOCITY = 2000;
    double LAUNCHER_MIN_VELOCITY = 1950;
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

    ElapsedTime feederTimer = new ElapsedTime();

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;

    @Override
    public void init() {
        launchState = LaunchState.IDLE;

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
    }

    @Override
    public void loop() {
        mecanum_drivetrain();
       
/*
        if (gamepad2.circle) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else if (gamepad2.square) {
            launcher.setVelocity(STOP_SPEED);
        }
*/
        launch(gamepad2.triangle);

        adjust_launch_velocity();

        general_telemetry();
    }

    public void adjust_launch_velocity(){
        if (gamepad2.dpad_up){
            LAUNCHER_TARGET_VELOCITY += 50;
            LAUNCHER_MIN_VELOCITY += 50;
        }

        if (gamepad2.dpad_up){
            LAUNCHER_TARGET_VELOCITY -= 50;
            LAUNCHER_MIN_VELOCITY -= 50;
        }

    }

    public void mecanum_drivetrain() {
        // --- Controller Variables ---
        double y = gamepad2.left_stick_y;
        double x = -gamepad2.left_stick_x * STRAFING_CORRECTION; //This correction empowers the strafing
        double r = -gamepad2.right_stick_x;

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

        telemetry.addData("State", launchState);
        telemetry.addData("Motors", "FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f",
                frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        telemetry.addData("motorSpeed", launcher.getVelocity());
    }

    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }else{
                    launcher.setVelocity(0);
                }
                leftFeeder.setPower(STOP_SPEED);
                rightFeeder.setPower(STOP_SPEED);
                break;

            case SPIN_UP:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;

            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;

                }
                break;
        }
    }
}
