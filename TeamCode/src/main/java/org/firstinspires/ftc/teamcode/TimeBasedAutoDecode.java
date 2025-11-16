package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Time-Based Auto Decode", group="Linear Opmode")
public class TimeBasedAutoDecode extends LinearOpMode {

    // --- DRIVETRAIN ---
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // --- LAUNCHER + FEEDER ---
    private DcMotor launcher;
    private DcMotor leftFeeder;
    private DcMotor rightFeeder;

    // --- CONSTANTS ---
    private static final double DRIVE_POWER = 0.4;
    private static final double STRAFE_POWER = 0.5;
    private static final double TURN_POWER = 0.4;

    // Launcher timing constants
    private static final double LAUNCHER_SPINUP_TIME = 1.0;   // seconds
    private static final double FEED_TIME = 0.4;              // seconds
    private static final double TIME_BETWEEN_SHOTS = 0.6;     // seconds

    private enum State {
        DRIVE_FORWARD,
        PREPARE,
        LAUNCH_THREE,
        DRIVE_BACK,
        ROTATE,
        DRIVE_OFF_LINE,
        COMPLETE
    }

    private State state = State.DRIVE_FORWARD;

    private enum LaunchState {
        PREPARE,
        LAUNCH,
        WAIT_FOR_NEXT,
        COMPLETE
    }

    private LaunchState launchState = LaunchState.PREPARE;

    private ElapsedTime driveTimer = new ElapsedTime();
    private ElapsedTime shotTimer = new ElapsedTime();
    private ElapsedTime feederTimer = new ElapsedTime();

    private int shotsFired = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();

        telemetry.addLine("Ready (No Encoders)");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        driveTimer.reset();

        while (opModeIsActive()) {
            switch (state) {

                case DRIVE_FORWARD:
                    driveMM(800);   // time-based forward drive
                    state = State.PREPARE;
                    break;

                case PREPARE:
                    launcher.setPower(1.0);   // spin up using raw power only
                    shotTimer.reset();
                    launchState = LaunchState.PREPARE;
                    state = State.LAUNCH_THREE;
                    break;

                case LAUNCH_THREE:
                    handleLauncherSequence();
                    if (launchState == LaunchState.COMPLETE) {
                        launcher.setPower(0);
                        leftFeeder.setPower(0);
                        rightFeeder.setPower(0);
                        state = State.DRIVE_BACK;
                        driveTimer.reset();
                    }
                    break;

                case DRIVE_BACK:
                    driveMM(-500);  // back up time-based
                    state = State.ROTATE;
                    break;

                case ROTATE:
                    rotateDegrees(90); // time-based turn
                    state = State.DRIVE_OFF_LINE;
                    break;

                case DRIVE_OFF_LINE:
                    driveMM(600);  // drive to parking
                    state = State.COMPLETE;
                    break;

                case COMPLETE:
                    stopMotors();
                    return;
            }

            telemetry.addData("State", state);
            telemetry.addData("LaunchState", launchState);
            telemetry.addData("Shots Fired", shotsFired);
            telemetry.update();
        }
    }

    // ----------------------------------------------------------------------
    // HARDWARE INIT
    // ----------------------------------------------------------------------

    private void initHardware() {
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        launcher   = hardwareMap.get(DcMotor.class, "launcher");
        leftFeeder = hardwareMap.get(DcMotor.class, "leftFeeder");
        rightFeeder= hardwareMap.get(DcMotor.class, "rightFeeder");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        launcher.setDirection(DcMotor.Direction.FORWARD);
        leftFeeder.setDirection(DcMotor.Direction.FORWARD);
        rightFeeder.setDirection(DcMotor.Direction.REVERSE);

        // IMPORTANT: NO ENCODERS ANYWHERE
        setRunWithoutEncoder(frontLeft);
        setRunWithoutEncoder(frontRight);
        setRunWithoutEncoder(backLeft);
        setRunWithoutEncoder(backRight);
        setRunWithoutEncoder(launcher);
        setRunWithoutEncoder(leftFeeder);
        setRunWithoutEncoder(rightFeeder);
    }

    private void setRunWithoutEncoder(DcMotor m) {
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // ----------------------------------------------------------------------
    // LAUNCH SEQUENCE (NO ENCODERS)
    // ----------------------------------------------------------------------

    private void handleLauncherSequence() {
        switch (launchState) {

            case PREPARE:
                // Just wait for spin-up
                if (shotTimer.seconds() >= LAUNCHER_SPINUP_TIME) {
                    leftFeeder.setPower(1);
                    rightFeeder.setPower(1);
                    feederTimer.reset();
                    launchState = LaunchState.LAUNCH;
                }
                break;

            case LAUNCH:
                // Feed for fixed time
                if (feederTimer.seconds() >= FEED_TIME) {
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);
                    shotsFired++;

                    if (shotsFired >= 3) {
                        launchState = LaunchState.COMPLETE;
                    } else {
                        shotTimer.reset();
                        launchState = LaunchState.WAIT_FOR_NEXT;
                    }
                }
                break;

            case WAIT_FOR_NEXT:
                if (shotTimer.seconds() >= TIME_BETWEEN_SHOTS) {
                    leftFeeder.setPower(1);
                    rightFeeder.setPower(1);
                    feederTimer.reset();
                    launchState = LaunchState.LAUNCH;
                }
                break;

            case COMPLETE:
                break;
        }
    }

    // ----------------------------------------------------------------------
    // MOTION HELPERS (TIME-BASED)
    // ----------------------------------------------------------------------

    private void driveMM(long durationMS) {
        double power = durationMS > 0 ? DRIVE_POWER : -DRIVE_POWER;

        driveTimer.reset();
        while (opModeIsActive() && Math.abs(driveTimer.milliseconds()) < Math.abs(durationMS)) {
            setDrivePower(power, power, power, power);
        }
        stopMotors();
    }

    private void rotateDegrees(double degrees) {
        // Rough estimation — tune constants as needed
        long turnTime = (long)(Math.abs(degrees) * 8); // assuming ~8ms per degree

        double power = degrees > 0 ? TURN_POWER : -TURN_POWER;

        driveTimer.reset();
        while (opModeIsActive() && driveTimer.milliseconds() < turnTime) {
            setDrivePower(-power, power, -power, power);
        }
        stopMotors();
    }

    private void setDrivePower(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    private void stopMotors() {
        setDrivePower(0, 0, 0, 0);
    }
}
