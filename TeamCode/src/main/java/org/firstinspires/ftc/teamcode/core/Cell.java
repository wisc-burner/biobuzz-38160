package org.firstinspires.ftc.teamcode.core;

/**
 * The four Hive Cells. Names match the SDK's AprilTag cluster names.
 *
 * Each Hive is a seesaw with a Cell at each end. A Cell's opening faces away from the Hive, toward
 * the audience wall or the rear wall, so you shoot at it from that side of the field. Every Tip
 * swaps which end is raised, which sends the robot to the other side of the Hive.
 */
public enum Cell {
    RED_SCORING("RED SCORING", Alliance.RED, -1),
    RED_AUDIENCE("RED AUDIENCE", Alliance.RED, 1),
    BLUE_AUDIENCE("BLUE AUDIENCE", Alliance.BLUE, 1),
    BLUE_SCORING("BLUE SCORING", Alliance.BLUE, -1);

    public final String clusterName;
    public final Alliance alliance;
    /** +1 if the opening faces the audience wall, -1 if it faces the rear wall. */
    public final int outward;

    Cell(String clusterName, Alliance alliance, int outward) {
        this.clusterName = clusterName;
        this.alliance = alliance;
        this.outward = outward;
    }

    /** Field X of the center of the opening while this Cell is raised. */
    public double x() {
        return outward * Field.OPENING_X;
    }

    public double y() {
        return alliance == Alliance.RED ? -Field.HIVE_Y : Field.HIVE_Y;
    }

    public Pose2d position() {
        return new Pose2d(x(), y(), 0);
    }

    /** The Cell on the other end of the same Hive. */
    public Cell other() {
        switch (this) {
            case RED_SCORING: return RED_AUDIENCE;
            case RED_AUDIENCE: return RED_SCORING;
            case BLUE_AUDIENCE: return BLUE_SCORING;
            default: return BLUE_AUDIENCE;
        }
    }

    /** Manual 10.3.1: red starts with its audience Cell raised, blue with its rear Cell. */
    public static Cell startingRaised(Alliance alliance) {
        return alliance == Alliance.RED ? RED_AUDIENCE : BLUE_SCORING;
    }

    /** Returns null for anything that is not a BIOBUZZ cluster. */
    public static Cell fromClusterName(String name) {
        for (Cell cell : values()) {
            if (cell.clusterName.equals(name)) {
                return cell;
            }
        }
        return null;
    }
}
