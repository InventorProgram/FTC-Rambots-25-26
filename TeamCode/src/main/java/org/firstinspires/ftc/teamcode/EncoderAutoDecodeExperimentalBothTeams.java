package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Autonomous: Decode (w/ Encoders) Experimental Both Teams V2", group = "Competition 2-15-26")
public class EncoderAutoDecodeExperimentalBothTeams extends LinearOpMode {

    // Hardware
    private DcMotorEx leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotorEx leftLauncher;
    private CRServo leftFeeder, rightFeeder;
    private Servo diverter;
    private IMU imu;
    private HuskyLens husky;

    // Constants
    private final double LAUNCHER_TARGET = 1200;
    private final double LAUNCHER_MIN    = 1175;
    private final double FEED_TIME       = 0.80;
    private final double FULL_SPEED      = 1.0;
    private final double STOP_SPEED      = 0.0;

    private final double TICKS_PER_INCH  = 42.8;
    private final double DRIVE_POWER     = 0.35;
    private final double HEADING_P       = 0.018;
    private final double AUTO_TIMEOUT_S  = 28.0;

    private ElapsedTime runtime = new ElapsedTime();

    // Alliance enum
    private enum Alliance {
        RED, BLUE
    }
    private Alliance alliance = Alliance.RED; // default

    @Override
    public void runOpMode() {
        // Hardware mapping
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        leftBackDrive   = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        rightBackDrive  = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        leftLauncher    = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        leftFeeder      = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder     = hardwareMap.get(CRServo.class, "rightFeeder");
        diverter        = hardwareMap.get(Servo.class, "diverter");

        // Directions & modes
        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftLauncher.setDirection(DcMotorEx.Direction.REVERSE);
        rightFeeder.setDirection(DcMotorEx.Direction.REVERSE);

        leftLauncher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // IMU setup
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(params);
        imu.resetYaw();

        // HuskyLens
        husky = hardwareMap.get(HuskyLens.class, "huskyLens");
        husky.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        // Initial servo states
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
        diverter.setPosition(0.15);

        // ─────────────── Alliance Selection ───────────────
        telemetry.addData("Alliance", "Press △ for BLUE, X for RED (default RED)");
        telemetry.update();
        while (!isStarted() && !isStopRequested()) {
            if (gamepad2.triangle) alliance = Alliance.BLUE;
            else if (gamepad2.cross) alliance = Alliance.RED;
            telemetry.addData("Selected", alliance);
            telemetry.update();
        }

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            imu.resetYaw();

            // ───────────── Detect motif tag ─────────────
            int motifTag = -1;
            for (int i = 0; i < 30 && opModeIsActive(); i++) {
                HuskyLens.Block[] blocks = husky.blocks();
                for (HuskyLens.Block b : blocks) {
                    if (b.id >= 21 && b.id <= 23) {
                        motifTag = b.id;
                        break;
                    }
                }
                if (motifTag > 0) break;
                sleep(100);
            }

            String motif;
            if (motifTag == 21)      motif = "GPP";
            else if (motifTag == 22) motif = "PGP";
            else if (motifTag == 23) motif = "PPG";
            else                     motif = "UNKNOWN - default center";

            telemetry.addData("Motif Tag", motifTag + " → " + motif);
            telemetry.update();

            // ───────────── Drive to shooting range ─────────────
            int forwardTicks = 1600; // default
            if (alliance == Alliance.BLUE) forwardTicks = 1600; // same distance for blue, can adjust if needed
            else forwardTicks = 1600; // red

            driveStraightTicks(forwardTicks, DRIVE_POWER);

            // ───────────── Spin up launcher ─────────────
            leftLauncher.setVelocity(LAUNCHER_TARGET);
            while (opModeIsActive() && leftLauncher.getVelocity() < LAUNCHER_MIN && runtime.seconds() < AUTO_TIMEOUT_S) {
                telemetry.addData("Launcher", "%.0f", leftLauncher.getVelocity());
                telemetry.update();
                sleep(40);
            }

            // ───────────── Set diverter & shoot 3× ─────────────
            double diverterPos = 0.15; // default
            if (motif.startsWith("G")) diverterPos = alliance == Alliance.RED ? 0.0 : 1.0;
            else if (motif.startsWith("P")) diverterPos = alliance == Alliance.RED ? 0.2962 : 0.7; // example

            diverter.setPosition(diverterPos);

            for (int shot = 1; shot <= 3 && opModeIsActive(); shot++) {
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                sleep((long)(FEED_TIME * 1000));
                leftFeeder.setPower(STOP_SPEED);
                rightFeeder.setPower(STOP_SPEED);
                sleep(550);
            }

            // ───────────── Simple park back ─────────────
            double parkPower = DRIVE_POWER * 0.75;
            if (alliance == Alliance.BLUE) parkPower = -parkPower; // reverse for blue if mirrored
            driveStraightTimed(-1.4, parkPower);

            // Cleanup
            leftLauncher.setVelocity(0);
            stopDrive();

            telemetry.addData("Auto", "Complete - %.1f s", runtime.seconds());
            telemetry.update();
        }
    }

    // ───────────── Helpers ─────────────
    private void driveStraightTicks(int targetTicks, double basePower) {
        if (targetTicks == 0) return;

        leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setTargetPosition(targetTicks);
        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        double startHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double startTime = runtime.seconds();

        while (opModeIsActive() && leftFrontDrive.isBusy() && (runtime.seconds() - startTime < 8.0)) {
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = startHeading - currentHeading;
            double correction = error * HEADING_P;

            double lf = basePower - correction;
            double rf = basePower + correction;
            double lb = basePower - correction;
            double rb = basePower + correction;

            lf = Math.max(-1.0, Math.min(1.0, lf));
            rf = Math.max(-1.0, Math.min(1.0, rf));
            lb = Math.max(-1.0, Math.min(1.0, lb));
            rb = Math.max(-1.0, Math.min(1.0, rb));

            leftFrontDrive.setPower(lf);
            rightFrontDrive.setPower(rf);
            leftBackDrive.setPower(lb);
            rightBackDrive.setPower(rb);

            telemetry.addData("Drive", "target %d  pos %d  err %.1f", targetTicks, leftFrontDrive.getCurrentPosition(), error);
            telemetry.update();
            sleep(25);
        }
        stopDrive();
        sleep(150);
    }

    private void driveStraightTimed(double seconds, double power) {
        double startHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double start = runtime.seconds();

        while (opModeIsActive() && runtime.seconds() - start < seconds) {
            double curr = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = startHeading - curr;
            double corr = error * HEADING_P;

            double lf = power - corr;
            double rf = power + corr;
            double lb = power - corr;
            double rb = power + corr;

            leftFrontDrive.setPower(lf);
            rightFrontDrive.setPower(rf);
            leftBackDrive.setPower(lb);
            rightBackDrive.setPower(rb);

            sleep(25);
        }
        stopDrive();
    }

    private void stopDrive() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}
