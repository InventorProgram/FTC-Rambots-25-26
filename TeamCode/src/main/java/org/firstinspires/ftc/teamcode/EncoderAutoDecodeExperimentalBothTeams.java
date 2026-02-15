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

@Autonomous(name = "Autonomous: Decode (w/ Encoders) Experimental Both Teams", group = "Competition 2-15-26")
public class EncoderAutoDecodeExperimentalBothTeams extends LinearOpMode {

    // Hardware
    private DcMotorEx leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotorEx leftLauncher;
    private CRServo leftFeeder, rightFeeder;
    private Servo diverter;
    private IMU imu;
    private HuskyLens husky;

    // Launcher & feeder constants
    private final double LAUNCHER_TARGET = 1200;
    private final double LAUNCHER_MIN    = 1175;
    private final double FEED_TIME       = 0.80;
    private final double FULL_SPEED      = 1.0;
    private final double STOP_SPEED      = 0.0;

    // Drive tuning
    private final double TICKS_PER_INCH  = 42.8;
    private final double DRIVE_POWER     = 0.35;
    private final double HEADING_P       = 0.018;
    private final double AUTO_TIMEOUT_S  = 28.0;

    private ElapsedTime runtime = new ElapsedTime();

    // Alliance flag
    private boolean isRedAlliance = true; // default

    @Override
    public void runOpMode() {
        // ──────────────────────
        // Hardware mapping
        // ──────────────────────
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

        // ──────────────────────
        // Alliance selection via gamepad
        // ──────────────────────
        telemetry.addLine("Select Alliance: A = RED, B = BLUE");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.a) {
                isRedAlliance = true;
                telemetry.addLine("Alliance: RED");
                telemetry.update();
            } else if (gamepad1.b) {
                isRedAlliance = false;
                telemetry.addLine("Alliance: BLUE");
                telemetry.update();
            }
            sleep(50);
        }

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            imu.resetYaw();

            // ──────────────────────
            // Step 1: Detect motif
            // ──────────────────────
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
            else                     motif = "UNKNOWN";

            telemetry.addData("Motif Tag", motifTag + " → " + motif);
            telemetry.update();

            // ──────────────────────
            // Step 2: Drive forward to shooting range
            // ──────────────────────
            int forwardTicks = isRedAlliance ? 1600 : -1600; // mirror for blue
            driveStraightTicks(forwardTicks, DRIVE_POWER);

            // ──────────────────────
            // Step 3: Spin up launcher
            // ──────────────────────
            leftLauncher.setVelocity(LAUNCHER_TARGET);
            while (opModeIsActive() && leftLauncher.getVelocity() < LAUNCHER_MIN && runtime.seconds() < AUTO_TIMEOUT_S) {
                telemetry.addData("Launcher", "%.0f", leftLauncher.getVelocity());
                telemetry.update();
                sleep(40);
            }

            // ──────────────────────
            // Step 4: Set diverter & shoot
            // ──────────────────────
            double diverterPos = 0.15;
            if (motif.startsWith("G")) {
                diverterPos = isRedAlliance ? 0.0 : 0.2962;
            } else if (motif.startsWith("P")) {
                diverterPos = isRedAlliance ? 0.2962 : 0.0;
            }
            diverter.setPosition(diverterPos);

            for (int shot = 1; shot <= 3 && opModeIsActive(); shot++) {
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                sleep((long)(FEED_TIME * 1000));

                leftFeeder.setPower(STOP_SPEED);
                rightFeeder.setPower(STOP_SPEED);

                sleep(550);
            }

            // ──────────────────────
            // Step 5: Simple park back
            // ──────────────────────
            double parkTime = 1.4;
            if (!isRedAlliance) parkTime *= -1; // mirror timing for blue
            driveStraightTimed(-parkTime, DRIVE_POWER * 0.75);

            // Cleanup
            leftLauncher.setVelocity(0);
            stopDrive();

            telemetry.addData("Auto", "Complete - %.1f s", runtime.seconds());
            telemetry.update();
        }
    }

    // ──────────────────────
    // Helper methods
    // ──────────────────────
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

        while (opModeIsActive() && Math.abs(runtime.seconds() - start) < Math.abs(seconds)) {
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
