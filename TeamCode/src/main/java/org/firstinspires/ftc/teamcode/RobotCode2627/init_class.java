package org.firstinspires.ftc.teamcode.RobotCode2627;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import java.util.List;

/**
 * BIO-BUZZ hardware: mecanum, intake, direct-drive flywheels and 200-degree turret.
 * Gate servo intentionally NOT initialized or commanded.
 */
// Main hardware class responsible for initialising and controlling the robot's drive system, intake, flywheels, turret, Limelight and servos.
public class init_class {
    public enum Alliance { RED, BLUE }
    public enum TargetMode { AUTO, SCORING, AUDIENCE }

    // Drive motors used for mecanum movement.
    private DcMotorEx frontLeft, backLeft, frontRight, backRight;

    // Motors responsible for collecting game elements and launching them.
    private DcMotorEx intake, flywheelLeft, flywheelRight, turret;

    // Servos used to control the turret gate and flower intake mechanisms
    private Servo TurretGate, FLowerIntakeL, FlowerIntakeR;
    private Limelight3A limelight;

    // ================== Config Settings ==================
    //Change these settings based on testing the robot
    private static final int APRILTAG_PIPELINE = 0; // Must be AprilTag pipeline.
    private static final long MAX_CAMERA_AGE_MS = 250;
    private static final long AUTO_CELL_CHANGE_DELAY_MS = 350;

    // Turret: REV HD Hex no gearbox (28 ticks/rev at motor), nominal 5:1 + 4:1
    // UltraPlanetary cartridges, then 25-tooth pinion to 100-tooth ring.
    // IMPORTANT: change these ratios if the installed 20:1 gearbox differs.
    private static final double MOTOR_ENCODER_TICKS_PER_REV = 28.0;

    // Turret configuration values used to calculate encoder movement, limit the turret's rotation and control its movement speed.
    private static final double GEARBOX_RATIO = (68.0 / 13.0) * (76.0 / 21.0);
    private static final double EXTERNAL_GEAR_RATIO = 100.0 / 25.0;
    private static final double TURRET_TICKS_PER_REV =
            MOTOR_ENCODER_TICKS_PER_REV * GEARBOX_RATIO * EXTERNAL_GEAR_RATIO;
    private static final double TURRET_SOFT_LIMIT_DEG = 90.0; // Turrets hardware limit is around 200 degrees, however would rather have the turret
    // have a 10 degree safety margin
    private static final double TURRET_LIMIT_SLOW_ZONE_DEG = 12.0;
    private static final double MAX_TURRET_POWER = 0.15;
    private static final double MANUAL_STICK_DEADZONE = 0.08;
    // Test at LOW power: if +power decreases encoder counts, change to -1.
    private static final double TURRET_ENCODER_DIRECTION = 1.0;
    // If auto-aim turns AWAY from tag, change +1 to -1.
    private static final double AUTO_AIM_MOTOR_SIGN = 1.0;
    private static final double AIM_OFFSET_DEGREES = 0.0;
    private static final double AIM_TOLERANCE_DEGREES = 1.0;
    private static final double AIM_KP = 0.012; // Tune on robot, start low.

    // Flywheels: REV HD Hex no gearbox, directly attached to 90 mm wheels.
    // Set these motor types correctly in the FTC Robot Configuration.
    // FTC setVelocity(rpm*6, AngleUnit.DEGREES) uses configured encoder spec.

    // Flywheel settings used to control shooting speed.
    // RPM values will need to be calibrated through actual robot testing.
    private static final double FLYWHEEL_TEST_RPM = 900.0;
    private static final double MAX_SHOOTER_RPM = 3800.0;

    // Distance-based shooting table.
    // Each distance corresponds to a target flywheel RPM.
    // These are placeholder values until proper testing is completed.
    private static final double RPM_TOLERANCE = 150.0;
    // Set TRUE only after replacing the sample distance/RPM pairs with tests.
    public static final boolean DISTANCE_RPM_CALIBRATED = false;
    // PLACEHOLDERS ONLY; not correct/proven speeds for a 15-degree launcher.
    private static final double[] SHOOT_DIST_M = {1.0, 2.0, 3.0, 4.0};
    private static final double[] SHOOT_RPM = {1400, 2000, 2700, 3400}; // to be calibrated when testing
    private static final double MIN_VALID_DIST_M = 0.3;
    private static final double MAX_VALID_DIST_M = 8.0;
    // ===========================================================

    private Alliance alliance = Alliance.RED;
    private boolean allianceSelected = false;
    private TargetMode targetMode = TargetMode.AUTO;
    private TargetMode activeCell = TargetMode.SCORING;
    private long lastActiveCellVisibleMs = 0;
    private int selectedTagId = -1;
    private double txDegrees = Double.NaN;
    private double horizontalDistanceM = Double.NaN;
    private boolean targetVisible = false;
    private boolean turretZeroed = false;
    private double appliedTurretPower = 0;
    private double flywheelTargetRPM = 0;
    private String shooterStatus = "OFF";
    private final ElapsedTime clock = new ElapsedTime();


    // Initialises all robot hardware using the FTC HardwareMap. Sets motor directions, operating modes, braking behaviour and starting positions.
    public void init(HardwareMap hwMap) {
        frontLeft = hwMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hwMap.get(DcMotorEx.class, "backLeft");
        frontRight = hwMap.get(DcMotorEx.class, "frontRight");
        backRight = hwMap.get(DcMotorEx.class, "backRight");
        intake = hwMap.get(DcMotorEx.class, "intakeMotor");
        flywheelLeft = hwMap.get(DcMotorEx.class, "FlywheelLeft");
        flywheelRight = hwMap.get(DcMotorEx.class, "FlywheelRight");
        turret = hwMap.get(DcMotorEx.class, "Turret");
        limelight = hwMap.get(Limelight3A.class, "limelight");
        TurretGate = hwMap.get(Servo.class, "turretGate");
        FlowerIntakeR = hwMap.get(Servo.class, "FlowerIntakeR");
        FLowerIntakeL = hwMap.get(Servo.class, "FlowerIntakeL");

        // Sets motor directions to ensure that positive power produces the expected physical movement.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelRight.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        FLowerIntakeL.setDirection(Servo.Direction.REVERSE);

        // Configures the motors to use encoders where required.The turret uses open-loop power control while still reading encoder positions.
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // For power control, encoders can still be read in RUN_WITHOUT_ENCODER.
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Start stationary. In particular, NEVER auto-reset turret encoder here.
        // Ensures all mechanisms begin in a safe stationary state before the robot starts operating.
        turret.setPower(0);
        intake.setVelocity(0);
        flywheelLeft.setVelocity(0);
        flywheelRight.setVelocity(0);
        turretZeroed = false;
        TurretGate.setPosition(0);
        FlowerIntakeR.setPosition(0);
        FLowerIntakeL.setPosition(0);
        clock.reset();

        // Starts the Limelight and selects the AprilTag detection pipeline used by the robot's automatic targeting system.
        limelight.setPollRateHz(60);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();
    }

    // Controls the robot's mecanum drive using forward, sideways and rotational movement inputs.
    public void driveMotors(double forward, double strafe, double turn) {
        // Optional joystick deadzone applied independently.
        // Ignores small joystick movements to prevent unwanted movement caused by minor controller stick input.
        if (Math.abs(forward) < 0.1) forward = 0;
        if (Math.abs(strafe) < 0.1) strafe = 0;
        if (Math.abs(turn) < 0.1) turn = 0;
        // Calculates the required power for each wheel based on the requested forward, strafe and turning movements.
        double flPower = forward + strafe + turn;
        double blPower = forward - strafe + turn;
        double frPower = forward + strafe - turn;
        double brPower = forward - strafe - turn;
        // Normalises the motor powers so that no wheel receives a power value greater than the allowed range.
        double denom = Math.max(1.0,
                Math.max(Math.max(Math.abs(flPower), Math.abs(blPower)),
                        Math.max(Math.abs(frPower), Math.abs(brPower))));
        frontLeft.setPower(flPower / denom);
        backLeft.setPower(blPower / denom);
        frontRight.setPower(frPower / denom);
        backRight.setPower(brPower / denom);
    }

    public void setIntakeVelocity(double ticksPerSecond) {
        // Sets the intake motor's operating speed in encoder ticks per second.
        intake.setVelocity(ticksPerSecond);
    }
    public void setGatePOS(double GatePos){
        // Sets the position of the turret gate servo.
        TurretGate.setPosition(GatePos);
    }
    public void setFIpos(double FIpos) {
        // Moves both flower intake servos to the requested position.
        FLowerIntakeL.setPosition(FIpos);
        FlowerIntakeR.setPosition(FIpos);
    }

    public Alliance getAlliance() { return alliance; }
    public boolean isAllianceSelected() { return allianceSelected; }
    public TargetMode getTargetMode() { return targetMode; }
    public TargetMode getActiveCell() { return activeCell; }

    // Sets the robot's alliance colour.
    // Clears existing targeting information when the alliance changes.
    public void setAlliance(Alliance selected) {
        allianceSelected = true;
        if (alliance != selected) {
            alliance = selected;
            clearTarget();
            activeCell = TargetMode.SCORING;
            lastActiveCellVisibleMs = 0;
        }
    }

    // Changes the targeting mode between automatic, scoring and audience.
    // Resets the current target when a new mode is selected.
    public void setTargetMode(TargetMode mode) {
        if (mode != targetMode) {
            targetMode = mode;
            // Up/AUTO starts by preferring scoring, then follows whichever CELL
            // remains visible when the currently tracked one disappears.
            activeCell = (mode == TargetMode.AUDIENCE)
                    ? TargetMode.AUDIENCE : TargetMode.SCORING;
            lastActiveCellVisibleMs = 0;
            clearTarget();
        }
    }

    // Checks whether an AprilTag ID belongs to the selected alliance and target group.
    private boolean isInGroup(int id, TargetMode cell) {
        if (alliance == Alliance.RED) {
            return cell == TargetMode.SCORING
                    ? id >= 30 && id <= 33 : id >= 34 && id <= 37;
        }
        return cell == TargetMode.SCORING
                ? id >= 42 && id <= 45 : id >= 38 && id <= 41;
    }

    // Finds the most suitable visible AprilTag.
    // Prioritises the previously selected tag to prevent unnecessary switching between targets.
    private LLResultTypes.FiducialResult bestTag(
            @NonNull List<LLResultTypes.FiducialResult> tags, TargetMode cell) {
        LLResultTypes.FiducialResult best = null;
        double minAbsTx = Double.POSITIVE_INFINITY;
        for (LLResultTypes.FiducialResult tag : tags) {
            if (!isInGroup(tag.getFiducialId(), cell)) continue;
            double tx = tag.getTargetXDegrees();
            if (Double.isNaN(tx) || Double.isInfinite(tx)) continue;
            // Keep previous AprilTag while it remains visible.
            if (tag.getFiducialId() == selectedTagId) return tag;
            if (Math.abs(tx) < minAbsTx) {
                minAbsTx = Math.abs(tx);
                best = tag;
            }
        }
        return best;
    }

    // Resets all stored targeting information when no valid AprilTag target is available.
    private void clearTarget() {
        targetVisible = false;
        selectedTagId = -1;
        txDegrees = Double.NaN;
        horizontalDistanceM = Double.NaN;
    }

    // Updates the robot's targeting information using the latest Limelight camera result.
    public void updateVision() {
        if (!allianceSelected) {
            clearTarget();
            return;
        }
        // Rejects invalid, outdated or incorrect-pipeline camera results to prevent the robot from using unreliable targeting information.
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()
                || limelight.getTimeSinceLastUpdate() < 0
                || limelight.getTimeSinceLastUpdate() > MAX_CAMERA_AGE_MS
                || result.getPipelineIndex() != APRILTAG_PIPELINE) {
            clearTarget();
            return;
        }

        // AUTO mode initially prefers scoring targets.
        // If the current target group disappears, the robot waits before switching to the other visible target group.
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) {
            clearTarget();
            return;
        }
        LLResultTypes.FiducialResult scoring = bestTag(tags, TargetMode.SCORING);
        LLResultTypes.FiducialResult audience = bestTag(tags, TargetMode.AUDIENCE);
        LLResultTypes.FiducialResult chosen;

        if (targetMode == TargetMode.SCORING) {
            activeCell = TargetMode.SCORING;
            chosen = scoring;
        } else if (targetMode == TargetMode.AUDIENCE) {
            activeCell = TargetMode.AUDIENCE;
            chosen = audience;
        } else {
            long nowMs = (long) clock.milliseconds();
            LLResultTypes.FiducialResult current =
                    activeCell == TargetMode.SCORING ? scoring : audience;
            LLResultTypes.FiducialResult other =
                    activeCell == TargetMode.SCORING ? audience : scoring;
            if (current != null) {
                lastActiveCellVisibleMs = nowMs;
            } else if (other != null &&
                    nowMs - lastActiveCellVisibleMs >= AUTO_CELL_CHANGE_DELAY_MS) {
                activeCell = activeCell == TargetMode.SCORING
                        ? TargetMode.AUDIENCE : TargetMode.SCORING;
                current = other;
                lastActiveCellVisibleMs = nowMs;
            }
            chosen = current;
        }

        if (chosen == null) {
            clearTarget();
            return;
        }
        targetVisible = true;
        selectedTagId = chosen.getFiducialId();
        txDegrees = chosen.getTargetXDegrees();
        horizontalDistanceM = distanceToTagMeters(chosen);
    }

    // Calculates the horizontal distance between the camera and an AprilTag using the camera-space X and Z coordinates.
    private double distanceToTagMeters(LLResultTypes.FiducialResult tag) {
        // Limelight target pose in camera space: X right, Y vertical, Z forward.
        // X-Z plane gives horizontal camera-to-tag distance (not goal opening).
        // Requires a valid 3D AprilTag pipeline with the correct tag size.
        Pose3D pose = tag.getTargetPoseCameraSpace();
        if (pose == null || pose.getPosition() == null) return Double.NaN;
        Position p = pose.getPosition().toUnit(DistanceUnit.METER);
        double d = Math.hypot(p.x, p.z);
        if (Double.isNaN(d) || Double.isInfinite(d)
                || d < MIN_VALID_DIST_M || d > MAX_VALID_DIST_M) {
            return Double.NaN;
        }
        return d;
    }

    // Resets the turret encoder to establish its centre position.
    //This must be performed deliberately by the operator during INIT.
    public void zeroTurretAtCenter() {
        // Called only by deliberate operator press in INIT.
        turret.setPower(0);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretZeroed = true;
    }

    public boolean isTurretReady() { return turretZeroed; }
    public int getTurretPosition() { return turret.getCurrentPosition(); }

    // Converts the turret's maximum allowed rotation in degrees into the corresponding encoder tick limit.
    public int getTurretLimitTicks() {
        return (int) Math.round(TURRET_TICKS_PER_REV * TURRET_SOFT_LIMIT_DEG / 360.0);
    }
    public double getTurretAngleDegrees() {
        return (turret.getCurrentPosition() / TURRET_TICKS_PER_REV)
                * 360.0 * TURRET_ENCODER_DIRECTION;
    }
    public double getTurretPower() { return appliedTurretPower; }

    // Applies safety limits to turret movement.
    // Prevents movement beyond the software rotation limits and gradually reduces power near the limits.
    private void limitedTurretPower(double requested) {
        if (!turretZeroed) requested = 0;
        requested = Math.max(-MAX_TURRET_POWER,
                Math.min(MAX_TURRET_POWER, requested));
        double angle = getTurretAngleDegrees();
        // TURRET_ENCODER_DIRECTION maps encoder counts to the direction of
        // increasing positive motor power; verify this before use.
        double signedTicks = getTurretPosition() * TURRET_ENCODER_DIRECTION;
        int limitTicks = getTurretLimitTicks();
        if ((requested > 0 && signedTicks >= limitTicks)
                || (requested < 0 && signedTicks <= -limitTicks)) {
            requested = 0;
        }
        double direction = Math.signum(requested);
        double remaining = direction > 0
                ? TURRET_SOFT_LIMIT_DEG - angle
                : angle + TURRET_SOFT_LIMIT_DEG;
        if (requested == 0 || remaining <= 0) {
            requested = 0;
        } else if (remaining < TURRET_LIMIT_SLOW_ZONE_DEG) {
            requested *= Math.max(0.0,
                    remaining / TURRET_LIMIT_SLOW_ZONE_DEG);
        }
        turret.setPower(requested);
        appliedTurretPower = requested;
    }

    // Controls the turret using either automatic Limelight aiming or manual controller input.
    public void controlTurret(boolean autoAimHeld, double manualX) {
        if (!turretZeroed) {
            limitedTurretPower(0);
            return;
        }
        if (autoAimHeld) {

            if (!targetVisible) {
                limitedTurretPower(0); // Never search blindly.
                return;
            }
            // Calculates the horizontal aiming error between the detected AprilTag and the desired camera alignment.
            double error = txDegrees - AIM_OFFSET_DEGREES;
            if (Math.abs(error) <= AIM_TOLERANCE_DEGREES) {
                limitedTurretPower(0);
                return;
            }
            limitedTurretPower(AUTO_AIM_MOTOR_SIGN * AIM_KP * error);
        } else {
            if (Math.abs(manualX) < MANUAL_STICK_DEADZONE) manualX = 0;
            limitedTurretPower(manualX * MAX_TURRET_POWER);
        }
    }

    // Calculates the required shooting RPM based on the measured distance to the AprilTag using linear interpolation.
    private double speedForDistance(double meters) {
        if (Double.isNaN(meters) || Double.isInfinite(meters)
                || meters < SHOOT_DIST_M[0]
                || meters > SHOOT_DIST_M[SHOOT_DIST_M.length - 1]) {
            return 0; // Outside tested distance range: don't fire.
        }
        for (int i = 0; i < SHOOT_DIST_M.length - 1; i++) {
            if (meters <= SHOOT_DIST_M[i + 1]) {
                double fraction = (meters - SHOOT_DIST_M[i]) /
                        (SHOOT_DIST_M[i + 1] - SHOOT_DIST_M[i]);
                return SHOOT_RPM[i] + fraction * (SHOOT_RPM[i + 1] - SHOOT_RPM[i]);
            }
        }
        return SHOOT_RPM[SHOOT_RPM.length - 1];
    }

    // Controls the flywheel shooting system.
    // Supports a fixed test speed and an automatic distance-based speed.
    public void controlFlywheels(double testTrigger, double autoTrigger) {
        double rpm = 0;
        if (testTrigger > 0.5) {
            // Automatic shooting is disabled until the distance-to-RPM calibration values have been tested and confirmed.
            rpm = FLYWHEEL_TEST_RPM;
            shooterStatus = "LOW-RPM TEST";
        } else if (autoTrigger > 0.5) {
            if (!DISTANCE_RPM_CALIBRATED) {
                shooterStatus = "AUTO DISABLED: CALIBRATE RPM";
            } else if (!targetVisible || Double.isNaN(horizontalDistanceM)) {
                shooterStatus = "NO VALID TAG DISTANCE";
            } else {
                rpm = speedForDistance(horizontalDistanceM);
                shooterStatus = rpm > 0 ? "DISTANCE AUTO RPM" : "OUTSIDE RPM TABLE";
            }
        } else {
            shooterStatus = "OFF";
        }
        // Converts the requested RPM into degrees per second for the FTC motor velocity control method.
        rpm = Math.max(0.0, Math.min(MAX_SHOOTER_RPM, rpm));
        flywheelTargetRPM = rpm;
        // This overload works in degrees/second; RPM x 360/60 = RPM x 6.
        // Robot configuration MUST use the correct encoder motor type.
        flywheelLeft.setVelocity(rpm * 6.0, AngleUnit.DEGREES);
        flywheelRight.setVelocity(rpm * 6.0, AngleUnit.DEGREES);
    }

    public double getLeftFlywheelRPM() {
        return Math.abs(flywheelLeft.getVelocity(AngleUnit.DEGREES)) / 6.0;
    }
    public double getRightFlywheelRPM() {
        return Math.abs(flywheelRight.getVelocity(AngleUnit.DEGREES)) / 6.0;
    }
    public double getFlywheelTargetRPM() { return flywheelTargetRPM; }
    public String getShooterStatus() { return shooterStatus; }

    // Checks whether both flywheels have reached their target RPM within the allowed speed tolerance.
    public boolean isFlywheelAtSpeed() {
        return flywheelTargetRPM > 0
                && Math.abs(getLeftFlywheelRPM() - flywheelTargetRPM) <= RPM_TOLERANCE
                && Math.abs(getRightFlywheelRPM() - flywheelTargetRPM) <= RPM_TOLERANCE;
    }

    public boolean isTargetVisible() { return targetVisible; }
    public int getTargetId() { return selectedTagId; }
    public double getTargetXDegrees() { return txDegrees; }
    public double getTargetDistanceMeters() { return horizontalDistanceM; }


    // Stops all motors and shuts down the Limelight.
    // Resets stored motor power and target speed values when the robot operation ends.
    public void stopAll() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        intake.setVelocity(0);
        flywheelLeft.setVelocity(0);
        flywheelRight.setVelocity(0);
        turret.setPower(0);
        appliedTurretPower = 0;
        flywheelTargetRPM = 0;
        limelight.stop();
    }
}
