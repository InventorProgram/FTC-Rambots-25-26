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

@Autonomous(name = "Autonomous: Decode Hyper-Experimental", group = "Competition 2-15-26")
public class EncoderAutoDecodeHyperExperimental extends LinearOpMode {

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
    private final double DRIVE_POWER     = 0.35;
    private final double HEADING_P       = 0.018;
    private final double TICKS_PER_INCH  = 42.8;
    private final double AUTO_TIMEOUT_S  = 28.0;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Hardware mapping
        mapHardware();
        setupIMU();
        setupHuskyLens();
        initServos();

        telemetry.addData("Status", "Ready - Hyper-Experimental Autonomous");
        telemetry.update();

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            imu.resetYaw();

            int motifTag = detectMotifTag();
            String motif = interpretMotifTag(motifTag);
            telemetry.addData("Motif Tag", motifTag + " → " + motif);
            telemetry.update();

            driveStraightTicks(1600, DRIVE_POWER);
            spinUpLauncher(LAUNCHER_TARGET, LAUNCHER_MIN);
            setDiverterForMotif(motif);
            shootBalls(3);
            driveStraightTimed(-1.4, DRIVE_POWER * 0.75);

            leftLauncher.setVelocity(0);
            stopDrive();

            telemetry.addData("Auto", "Complete - %.1f s", runtime.seconds());
            telemetry.update();
        }
    }

    // ─────────────── Hardware / Setup ───────────────
    private void mapHardware() {
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        leftBackDrive   = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        rightBackDrive  = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        leftLauncher    = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        leftFeeder      = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder     = hardwareMap.get(CRServo.class, "rightFeeder");
        diverter        = hardwareMap.get(Servo.class, "diverter");

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
    }

    private void setupIMU() {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(params);
        imu.resetYaw();
    }

    private void setupHuskyLens() {
        husky = hardwareMap.get(HuskyLens.class, "huskyLens");
        husky.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    }

    private void initServos() {
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
        diverter.setPosition(0.15);
    }

    // ─────────────── Autonomous Steps ───────────────

    private int detectMotifTag() {
        int tag = -1;
        for (int i = 0; i < 30 && opModeIsActive(); i++) {
            HuskyLens.Block[] blocks = husky.blocks();
            for (HuskyLens.Block b : blocks) {
                if (b.id >= 21 && b.id <= 23) {
                    tag = b.id;
                    break;
                }
            }
            if (tag > 0) break;
            sleep(100);
        }
        return tag;
    }

    private String interpretMotifTag(int tag) {
        if (tag == 21) return "GPP";
        if (tag == 22) return "PGP";
        if (tag == 23) return "PPG";
        return "UNKNOWN - default center";
    }

    private void spinUpLauncher(double target, double min) {
        leftLauncher.setVelocity(target);
        while (opModeIsActive() && leftLauncher.getVelocity() < min && runtime.seconds() < AUTO_TIMEOUT_S) {
            telemetry.addData("Launcher Velocity", leftLauncher.getVelocity());
            telemetry.update();
            sleep(25);
        }
    }

    private void setDiverterForMotif(String motif) {
        double pos = 0.15;
        if (motif.startsWith("G")) pos = 0.0;
        else if (motif.startsWith("P")) pos = 0.2962;
        diverter.setPosition(pos);
        telemetry.addData("Diverter Pos", pos);
        telemetry.update();
    }

    private void shootBalls(int count) {
        for (int i = 1; i <= count && opModeIsActive(); i++) {
            leftFeeder.setPower(FULL_SPEED);
            rightFeeder.setPower(FULL_SPEED);
            telemetry.addData("Shooting", "Shot %d", i);
            telemetry.update();
            sleep((long)(FEED_TIME * 1000));

            leftFeeder.setPower(STOP_SPEED);
            rightFeeder.setPower(STOP_SPEED);
            sleep(550);
        }
    }

    // ─────────────── Drive Helpers ───────────────

    private void driveStraightTicks(int targetTicks, double basePower) {
        if (targetTicks == 0) return;

        leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setTargetPosition(targetTicks);
        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        double startHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double startTime = runtime.seconds();

        while (opModeIsActive() && leftFrontDrive.isBusy() && (runtime.seconds() - startTime < 8.0)) {
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double correction = (startHeading - currentHeading) * HEADING_P;

            double lf = clamp(basePower - correction);
            double rf = clamp(basePower + correction);
            double lb = clamp(basePower - correction);
            double rb = clamp(basePower + correction);

            setDrivePower(lf, rf, lb, rb);
            telemetry.addData("Drive", "Target %d  Pos %d  Error %.1f", targetTicks, leftFrontDrive.getCurrentPosition(), startHeading - currentHeading);
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
            double correction = (startHeading - curr) * HEADING_P;

            setDrivePower(clamp(power - correction), clamp(power + correction), clamp(power - correction), clamp(power + correction));
            sleep(25);
        }
        stopDrive();
    }

    private void setDrivePower(double lf, double rf, double lb, double rb) {
        leftFrontDrive.setPower(lf);
        rightFrontDrive.setPower(rf);
        leftBackDrive.setPower(lb);
        rightBackDrive.setPower(rb);
    }

    private double clamp(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }

    private void stopDrive() {
        setDrivePower(0, 0, 0, 0);
    }
}
