package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Autonomous for your 4-wheel mecanum robot based on the StarterBot Decode template.
 * Simplified and corrected to work with your TeleOp hardware configuration.
 */

//This is AI generated based on the teleop that has been tailored to our robot and the starter bot autonomous.
@Autonomous(name="Autonomous Decode (Mecanum)", group="StarterBot")
public class AutonomousDecode extends OpMode {

    // --- Launcher constants ---
    final double FEED_TIME = 0.20;
    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;
    final double TIME_BETWEEN_SHOTS = 2;

    // --- Drive constants ---
    final double DRIVE_SPEED = 0.5;
    final double ROTATE_SPEED = 0.25;
    final double WHEEL_DIAMETER_MM = 96;
    final double ENCODER_TICKS_PER_REV = 537.7;
    final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    final double TRACK_WIDTH_MM = 404;

    int shotsToFire = 3;
    double robotRotationAngle = 45;

    // Timers
    private ElapsedTime shotTimer = new ElapsedTime();
    private ElapsedTime driveTimer = new ElapsedTime();

    // Motors
    private DcMotorEx launcher = null;
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;

    // --- Enums ---
    private enum LaunchState { IDLE, PREPARE, LAUNCH }
    private enum AutonomousState { LAUNCH, WAIT_FOR_LAUNCH, DRIVE_BACK, ROTATE, DRIVE_OFF_LINE, COMPLETE }
    private enum Alliance { RED, BLUE }

    private LaunchState launchState;
    private AutonomousState autonomousState;
    private Alliance alliance = Alliance.RED;

    @Override
    public void init() {
        autonomousState = AutonomousState.LAUNCH;
        launchState = LaunchState.IDLE;

        // --- Hardware map ---
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        // --- Motor directions (match TeleOp) ---
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // --- Zero power behaviors ---
        frontLeftMotor.setZeroPowerBehavior(BRAKE);
        frontRightMotor.setZeroPowerBehavior(BRAKE);
        backLeftMotor.setZeroPowerBehavior(BRAKE);
        backRightMotor.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        // Reset encoders
        resetDriveEncoders();

        // --- Launcher setup ---
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        if (gamepad1.b) alliance = Alliance.RED;
        else if (gamepad1.x) alliance = Alliance.BLUE;

        telemetry.addData("Press X", "for BLUE");
        telemetry.addData("Press B", "for RED");
        telemetry.addData("Selected Alliance", alliance);
    }

    @Override
    public void start() {
        // nothing
    }

    @Override
    public void loop() {
        switch (autonomousState) {
            case LAUNCH:
                launch(true);
                autonomousState = AutonomousState.WAIT_FOR_LAUNCH;
                break;

            case WAIT_FOR_LAUNCH:
                if (launch(false)) {
                    shotsToFire--;
                    if (shotsToFire > 0) {
                        autonomousState = AutonomousState.LAUNCH;
                    } else {
                        launcher.setVelocity(0);
                        resetDriveEncoders();
                        autonomousState = AutonomousState.DRIVE_BACK;
                    }
                }
                break;

            case DRIVE_BACK:
                if (drive(DRIVE_SPEED, -4, DistanceUnit.INCH, 1)) {
                    resetDriveEncoders();
                    autonomousState = AutonomousState.ROTATE;
                }
                break;

            case ROTATE:
                robotRotationAngle = (alliance == Alliance.RED) ? 45 : -45;
                if (rotate(ROTATE_SPEED, robotRotationAngle, AngleUnit.DEGREES, 1)) {
                    resetDriveEncoders();
                    autonomousState = AutonomousState.DRIVE_OFF_LINE;
                }
                break;

            case DRIVE_OFF_LINE:
                if (drive(DRIVE_SPEED, -26, DistanceUnit.INCH, 1)) {
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;

            case COMPLETE:
                stopAllMotors();
                break;
        }

        telemetry.addData("AutoState", autonomousState);
        telemetry.addData("LauncherState", launchState);
        telemetry.addData("Launcher Velocity", launcher.getVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        stopAllMotors();
    }

    // --- LAUNCHER CONTROL ---
    boolean launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.PREPARE;
                    shotTimer.reset();
                }
                break;

            case PREPARE:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;

            case LAUNCH:
                if (shotTimer.seconds() > TIME_BETWEEN_SHOTS) {
                    launchState = LaunchState.IDLE;
                    return true;
                }
                break;
        }
        return false;
    }

    // --- DRIVE HELPERS ---
    boolean drive(double speed, double distance, DistanceUnit distanceUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;
        double targetPosition = distanceUnit.toMm(distance) * TICKS_PER_MM;

        // All four motors should move equally to drive straight
        setAllDriveTarget((int) targetPosition);
        setAllDrivePower(speed);

        if (Math.abs(targetPosition - frontLeftMotor.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)) {
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }

    boolean rotate(double speed, double angle, AngleUnit angleUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;
        double targetMm = angleUnit.toRadians(angle) * (TRACK_WIDTH_MM / 2);
        double targetTicks = targetMm * TICKS_PER_MM;

        // Opposite directions for rotation
        setDriveTargets(-(int) targetTicks, (int) targetTicks, -(int) targetTicks, (int) targetTicks);
        setAllDrivePower(speed);

        if (Math.abs(targetTicks - frontLeftMotor.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)) {
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }

    // --- UTILITY METHODS ---
    void resetDriveEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void setAllDriveTarget(int targetTicks) {
        frontLeftMotor.setTargetPosition(targetTicks);
        frontRightMotor.setTargetPosition(targetTicks);
        backLeftMotor.setTargetPosition(targetTicks);
        backRightMotor.setTargetPosition(targetTicks);
    }

    void setDriveTargets(int fl, int fr, int bl, int br) {
        frontLeftMotor.setTargetPosition(fl);
        frontRightMotor.setTargetPosition(fr);
        backLeftMotor.setTargetPosition(bl);
        backRightMotor.setTargetPosition(br);
    }

    void setAllDrivePower(double power) {
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
    }

    void stopAllMotors() {
        setAllDrivePower(0);
        launcher.setPower(0);
    }
}
