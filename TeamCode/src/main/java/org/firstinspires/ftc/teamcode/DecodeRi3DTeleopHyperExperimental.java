package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "DECODE Ri3D Teleop Hyper-Experimental", group = "Competition 2-15-26")
public class DecodeRi3DTeleopHyperExperimental extends OpMode {

    // Drive scaling
    private static final double PRECISION_SCALE = 0.4;

    // Launcher constants
    private static final double FEED_TIME_SECONDS = 0.8;
    private static final double FULL_SPEED = 1.0;
    private static final double STOP_SPEED = 0.0;

    private static final double LAUNCHER_CLOSE_TARGET_VELOCITY = 1200;
    private static final double LAUNCHER_CLOSE_MIN_VELOCITY = 1175;
    private static final double LAUNCHER_FAR_TARGET_VELOCITY = 1350;
    private static final double LAUNCHER_FAR_MIN_VELOCITY = 1325;

    private double launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY;
    private double launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;

    private static final double LEFT_POSITION = 0.4;
    private static final double RIGHT_POSITION = 0;

    private boolean rightFeederActive = false;

    // Drive motors
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    // Mechanisms
    private DcMotorEx leftLauncher;
    private DcMotor intake;
    private CRServo leftFeeder;
    private CRServo rightFeeder;
    private Servo diverter;

    private ElapsedTime leftFeederTimer = new ElapsedTime();
    private ElapsedTime rightFeederTimer = new ElapsedTime();

    private enum LaunchState {IDLE, SPIN_UP, LAUNCH, LAUNCHING}
    private LaunchState leftLaunchState;

    private enum DiverterDirection {LEFT, RIGHT}
    private DiverterDirection diverterDirection = DiverterDirection.LEFT;

    private boolean intakeOn = false;

    private enum LauncherDistance {CLOSE, FAR}
    private LauncherDistance launcherDistance = LauncherDistance.CLOSE;

    double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;

    @Override
    public void init() {
        leftLaunchState = LaunchState.IDLE;

        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRightMotor");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeftMotor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRightMotor");

        leftLauncher = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        diverter = hardwareMap.get(Servo.class, "diverter");

        diverter.setPosition(LEFT_POSITION);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        leftLauncher.setZeroPowerBehavior(BRAKE);

        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Drive with optional precision
        double scale = gamepad2.left_trigger > 0.1 ? PRECISION_SCALE : 1.0;
        mecanumDrive(-gamepad2.left_stick_y * scale, gamepad2.left_stick_x * scale, gamepad2.right_stick_x * scale);

        // Launcher control
        if (gamepad2.triangle) {
            leftLauncher.setVelocity(launcherTarget);
        } else if (gamepad2.circle) {
            leftLauncher.setVelocity(STOP_SPEED);
            leftLaunchState = LaunchState.IDLE;
            rightFeeder.setPower(STOP_SPEED);
            rightFeederActive = false;
        }

        // Diverter toggle
        if (gamepad2.dpad_down) {
            if (diverterDirection == DiverterDirection.LEFT) {
                diverterDirection = DiverterDirection.RIGHT;
                diverter.setPosition(RIGHT_POSITION);
            } else {
                diverterDirection = DiverterDirection.LEFT;
                diverter.setPosition(LEFT_POSITION);
            }
        }

        // Intake toggle
        if (gamepad2.cross) {
            intakeOn = !intakeOn;
            intake.setPower(intakeOn ? 1 : 0);
        }

        // Launcher distance toggle
        if (gamepad2.dpad_up) {
            if (launcherDistance == LauncherDistance.CLOSE) {
                launcherDistance = LauncherDistance.FAR;
                launcherTarget = LAUNCHER_FAR_TARGET_VELOCITY;
                launcherMin = LAUNCHER_FAR_MIN_VELOCITY;
            } else {
                launcherDistance = LauncherDistance.CLOSE;
                launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY;
                launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;
            }
        }

        // Launch routines
        launchLeft(gamepad2.left_bumper);
        launchRight(gamepad2.right_bumper);

        // Telemetry
        telemetry.addData("Drive (LF, RF, LB, RB)", "%.2f, %.2f, %.2f, %.2f", leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        telemetry.addData("Launcher", leftLaunchState);
        telemetry.addData("Launcher Velocity", leftLauncher.getVelocity());
        telemetry.addData("Feeder Active", rightFeederActive);
        telemetry.update();
    }

    void mecanumDrive(double forward, double strafe, double rotate){
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);
        leftFrontPower = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower = (forward - strafe + rotate) / denominator;
        rightBackPower = (forward + strafe - rotate) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    void launchLeft(boolean shotRequested) {
        switch (leftLaunchState) {
            case IDLE:
                if (shotRequested) leftLaunchState = LaunchState.SPIN_UP;
                break;
            case SPIN_UP:
                leftLauncher.setVelocity(launcherTarget);
                if (leftLauncher.getVelocity() > launcherMin) leftLaunchState = LaunchState.LAUNCH;
                break;
            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                leftFeederTimer.reset();
                leftLaunchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (leftFeederTimer.seconds() > FEED_TIME_SECONDS) {
                    leftFeeder.setPower(STOP_SPEED);
                    leftLaunchState = LaunchState.IDLE;
                }
                break;
        }
    }

    void launchRight(boolean shotRequested) {
        if (shotRequested && !rightFeederActive) {
            leftLauncher.setVelocity(launcherTarget);
            if (leftLauncher.getVelocity() > launcherMin) {
                rightFeeder.setPower(FULL_SPEED);
                rightFeederTimer.reset();
                rightFeederActive = true;
            }
        }
        if (rightFeederActive && rightFeederTimer.seconds() > FEED_TIME_SECONDS) {
            rightFeeder.setPower(STOP_SPEED);
            rightFeederActive = false;
        }
    }
}
