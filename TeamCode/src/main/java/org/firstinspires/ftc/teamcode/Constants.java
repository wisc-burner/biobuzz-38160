package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.core.Field;

/**
 * Every number a student should need to tune lives here.
 *
 * Units are inches, seconds and degrees unless the name says otherwise.
 * Values marked ESTIMATE have not been measured on a real robot yet.
 * TeamCode/README.md lists the order to tune them in.
 *
 * This file must stay free of FTC SDK imports so the core package can be tested on a laptop.
 */
public final class Constants {
    private Constants() {}

    // Robot configuration names. The first six match REV's REVStarterBot2026.xml.
    public static final String LEFT_DRIVE = "leftDrive";
    public static final String RIGHT_DRIVE = "rightDrive";
    public static final String INTAKE = "intakeMotor";
    public static final String FLYWHEEL = "flywheel";
    public static final String FEEDER = "crServo";
    public static final String POKER = "pollenServo";
    public static final String WEBCAM = "Webcam 1";
    public static final String IMU = "imu";

    // Robot shape.
    public static final double ROBOT_LENGTH_IN = 17;            // ESTIMATE: measure bumper to bumper.
    public static final double ROBOT_WIDTH_IN = 16;             // ESTIMATE
    /** Inferred from REV's auto, which backs away from the wall to launch. Verify in Diagnostics. */
    public static final boolean LAUNCHER_FACES_REAR = false;
    /** REV's docs say "side", one code comment says "front". Check your robot. */
    public static final boolean POKER_ON_LEFT = true;
    public static final double POKER_FORWARD_IN = 4;            // ESTIMATE: poker ahead of robot center.
    /** The lowest point of a Hive is 30.6 in above the tiles (manual Figure 9-10). */
    public static final boolean CAN_DRIVE_UNDER_HIVE = true;

    // Drivetrain.
    /** ESTIMATE: 28 CPR * 3.61 * 5.23 UltraPlanetary stages / (90 mm wheel circumference). */
    public static final double DRIVE_TICKS_PER_IN = 47.5;
    public static final double STICK_DEADBAND = 0.05;
    public static final double DRIVER_TURN_SCALE = 0.8;

    // Self-driving (DriveController). Powers are 0 to 1.
    public static final double AUTO_MAX_DRIVE_POWER = 0.8;
    public static final double AUTO_MAX_TURN_POWER = 0.6;
    public static final double AUTO_MIN_DRIVE_POWER = 0.08;     // Just enough to overcome friction.
    public static final double AUTO_MIN_TURN_POWER = 0.12;
    public static final double AUTO_DRIVE_KP = 0.04;            // Power per inch of error.
    public static final double AUTO_TURN_KP = 1.2;              // Power per radian of error.
    public static final double AUTO_MAX_ACCEL = 2.5;            // Power per second while speeding up.
    public static final double POSITION_TOLERANCE_IN = 2.5;
    public static final double HEADING_TOLERANCE_DEG = 2.5;
    public static final double WAYPOINT_TOLERANCE_IN = 6;
    /** Stop driving and turn on the spot when pointed this far off the path. */
    public static final double MAX_DRIVE_HEADING_ERROR_DEG = 50;
    /** Space kept between robot center and field obstacles when planning a route. */
    public static final double ROUTE_CLEARANCE_IN = 11;

    // Launcher. 1160 ticks/s and the +/-20 and +/-50 windows are REV's own values.
    public static final double DEFAULT_FLYWHEEL_VELOCITY = 1200;
    public static final double TELEOP_VELOCITY_TOLERANCE = 20;
    public static final double AUTO_VELOCITY_TOLERANCE = 50;
    public static final double FLYWHEEL_REVERSE_POWER = -0.55;
    /** Time the feeder must run per Pollen in auto. ESTIMATE */
    public static final double FEED_SECONDS_PER_POLLEN = 1.0;
    /** Once feeding starts it keeps going at least this long, even through the flywheel dip. */
    public static final double MIN_FEED_SECONDS = 2.5;

    /** Distance from robot center to the Cell opening, measured along the floor -> flywheel ticks/s. */
    public static final double[][] SHOT_TABLE = {
            {30, 1160},
            {38, 1160},
            {50, 1160},
    };

    // Where the robot likes to shoot from.
    /** Measured: best shots came with the launcher end of the robot this far from the wall behind the Cell. */
    public static final double SHOT_FRONT_TO_WALL_IN = 23;
    /** Robot center to the Cell opening at that spot. Follows ROBOT_LENGTH_IN, so measure that too. */
    public static final double SHOT_DISTANCE_IN =
            Field.HALF - SHOT_FRONT_TO_WALL_IN + ROBOT_LENGTH_IN / 2 - Field.OPENING_X;
    /** Allowed shot positions fan out this far either side of straight-on. */
    public static final double SHOT_SECTOR_DEG = 30;
    /** Already this close to the ideal distance: turn to aim without driving. */
    public static final double SHOT_BAND_IN = 5;
    /** How many inches of driving one radian of extra turning is worth. */
    public static final double TURN_COST_IN_PER_RAD = 8;
    /** Straight-on shots are more reliable; extra inches charged per radian off-axis. */
    public static final double OFF_AXIS_COST_IN_PER_RAD = 12;

    // Poker. Positions are REV's: up = 1, down = 0.
    public static final double POKER_UP = 1;
    public static final double POKER_DOWN = 0;
    public static final double FLOWER_STANDOFF_IN = 13;         // ESTIMATE: robot center to Flower center.

    // Camera. Faces the launcher, tilted up, because the tags are on the underside of each Cell.
    public static final double CAMERA_FORWARD_IN = -4;          // Measured: behind robot center. Positive means ahead of center.
    public static final double CAMERA_LEFT_IN = 3;              // ESTIMATE
    public static final double CAMERA_HEIGHT_IN = 11;           // ESTIMATE: above the tiles.
    public static final double CAMERA_PITCH_UP_DEG = 37;        // this is using the gheto mount and may change
    public static final double CAMERA_YAW_DEG = LAUNCHER_FACES_REAR ? 180 : 0;
    public static final int CAMERA_EXPOSURE_MS = 15;            // Short avoids motion blur; too short is black. Tune under field light.
    public static final int CAMERA_GAIN = 250;
    public static final float TAG_DECIMATION = 2;

    // Trusting the camera.
    public static final double FIX_GAIN = 0.25;                 // Fraction of each correction applied.
    public static final double FIX_GATE_IN = 18;                // Larger jumps are treated as suspect...
    public static final int FIX_REJECTS_TO_ACCEPT = 5;          // ...until this many arrive in a row.
    public static final double FIX_MAX_RANGE_IN = 110;
    public static final double FIX_MAX_TURN_DEG_PER_SEC = 120;  // Blurry frames while spinning.
    public static final double FIX_HEIGHT_TOLERANCE_IN = 8;     // Sanity check on the solved tag height.
    public static final int FIX_MIN_PERCENT_VISIBLE = 50;
    /** Consecutive frames that must agree before believing a Hive tipped. */
    public static final int HIVE_FLIP_FRAMES = 5;

    // Match timing.
    public static final double TELEOP_SECONDS = 120;
    public static final double AUTO_PARK_BY_SECONDS = 24;       // Stop scoring and go park.
    public static final double RESUME_POSE_WINDOW_SECONDS = 30; // Auto's pose expires after this.
}
