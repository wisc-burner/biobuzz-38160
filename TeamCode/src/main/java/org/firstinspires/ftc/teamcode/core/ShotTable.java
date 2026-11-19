package org.firstinspires.ftc.teamcode.core;

import org.firstinspires.ftc.teamcode.Constants;

/** Flywheel speed for a given shot distance, interpolated from {@link Constants#SHOT_TABLE}. */
public final class ShotTable {
    private ShotTable() {}

    public static double velocityFor(double distance) {
        double[][] table = Constants.SHOT_TABLE;
        if (distance <= table[0][0]) {
            return table[0][1];
        }
        for (int i = 1; i < table.length; i++) {
            if (distance <= table[i][0]) {
                double fraction = (distance - table[i - 1][0]) / (table[i][0] - table[i - 1][0]);
                return table[i - 1][1] + fraction * (table[i][1] - table[i - 1][1]);
            }
        }
        return table[table.length - 1][1];
    }

    /** Speed for the spot the robot drives to. Used when the robot does not know where it is. */
    public static double atShotSpot() {
        return velocityFor(Constants.SHOT_DISTANCE_IN);
    }
}
