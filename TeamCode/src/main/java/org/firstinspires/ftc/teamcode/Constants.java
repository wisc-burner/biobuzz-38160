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
    public static final boolean LAUNCHER_FACES_REAR = true;
    /** REV's docs say "side", one code comment says "front". Check your robot. */
    public static final boolean POKER_ON_LEFT = false;
    public static final double POKER_FORWARD_IN = -4;           // ESTIMATE: poker ahead of robot center.
    /** The lowest point of a Hive is 30.6 in above the tiles (manual Figure 9-10). */
    public static final boolean CAN_DRIVE_UNDER_HIVE = true;

    // Drivetrain.
    /** ESTIMATE: 28 CPR * 3.61 * 5.23 UltraPlanetary stages / (90 mm wheel circumference). */
    public static final double DRIVE_TICKS_PER_IN = 47.5;
    public static final double STICK_DEADBAND = 0.05;
    public static final double DRIVER_TURN_SCALE = 0.8;
    /**
     * Fastest the driver's forward power may change, per second, either way. 2.5 means stopped to
     * full in 0.4 s, and full forward to full reverse in 0.8 s. Lower it if the robot still tips.
     */
    public static final double DRIVER_MAX_ACCEL = 2.5;

    // Self-driving (DriveController). Powers are 0 to 1.
    public static final double AUTO_MAX_DRIVE_POWER = 0.7;
    public static final double AUTO_MAX_TURN_POWER = 0.6;
    public static final double AUTO_MIN_DRIVE_POWER = 0.08;     // Just enough to overcome friction.
    public static final double AUTO_MIN_TURN_POWER = 0.12;
    public static final double AUTO_DRIVE_KP = 0.04;            // Power per inch of error.
    public static final double AUTO_TURN_KP = 1.2;              // Power per radian of error.
    public static final double AUTO_MAX_ACCEL = 2.5;            // Power per second while speeding up.
    public static final double POSITION_TOLERANCE_IN = 2.5;
    public static final double HEADING_TOLERANCE_DEG = 2.5;
    public static final double WAYPOINT_TOLERANCE_IN = 6;
    /**
     * Closer than this to the end, stop pointing at it: just drive straight forward or back, then
     * turn to the final heading. Turning to face a point this near chased camera noise in circles.
     */
    public static final double FINAL_APPROACH_IN = 6;
    /** Stop driving and turn on the spot when pointed this far off the path. */
    public static final double MAX_DRIVE_HEADING_ERROR_DEG = 50;
    /** Space kept between robot center and field obstacles when planning a route. */
    public static final double ROUTE_CLEARANCE_IN = 11;

    // Launcher. The +/-20 and +/-50 windows are REV's own values.
    public static final double TELEOP_VELOCITY_TOLERANCE = 20;
    public static final double AUTO_VELOCITY_TOLERANCE = 50;
    public static final double FLYWHEEL_REVERSE_POWER = -0.55;
    /** A drop this big from steady speed counts as a shot. ESTIMATE: check the shot log against real shots. */
    public static final double SHOT_DIP_TICKS_PER_SEC = 60;
    /** Time the feeder must run per Pollen in auto. ESTIMATE */
    public static final double FEED_SECONDS_PER_POLLEN = 1.0;

    /**
     * {shot distance, flywheel ticks/s}, shortest distance first. Shot distance is robot center to
     * the Cell opening along the floor, as Diagnostics shows it. Speeds between rows are blended and
     * speeds past either end are held. Every shot uses this table, including when the robot does not
     * know where it is: then it assumes it is at SHOT_DISTANCE_IN.
     * ESTIMATE: only the middle row (1200, the speed drivers were scoring with by hand) is from a
     * real robot. Re-measure each row in Diagnostics.
     */
    public static final double[][] SHOT_TABLE = {
            {30, 1120},
            {44, 1200},
            {56, 1280},
    };

    // Where the robot likes to shoot from.
    /**
     * Launcher end of the robot to the wall behind the Cell. Smaller backs the robot away from the
     * Cell, which sends the ball higher at the opening. Was 23, and auto-aligned shots came in low.
     * Below about 17 the back bumper hits the wall.
     */
    public static final double SHOT_FRONT_TO_WALL_IN = 20;
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

    // Poker. The arm is mounted so the end of the servo's travel (0) is just below down; moving
    // toward 1 swings it back up. Past about 0.5 it heads for the drive motor.
    public static final double POKER_DOWN = 0.06;               // Just off the end of travel so it holds itself up off the floor.
    public static final double POKER_UP = 0.4;                  // Tested: stands straight up.
    public static final double FLOWER_STANDOFF_IN = 13;         // ESTIMATE: robot center to Flower center.

    // Camera. Faces the launcher, tilted up, because the tags are on the underside of each Cell.
    public static final double CAMERA_FORWARD_IN = 3;           // Measured: behind robot center. Positive means ahead of center.
    public static final double CAMERA_LEFT_IN = 0;              // ESTIMATE
    public static final double CAMERA_HEIGHT_IN = 12;           // ESTIMATE: above the tiles.
    public static final double CAMERA_PITCH_UP_DEG = 45;        // this is using the gheto mount and may change
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
