package org.firstinspires.ftc.teamcode.core;

import org.firstinspires.ftc.teamcode.Constants;

/** Remembers which Cell of each Hive is raised, using the camera and a manual override. */
public final class HiveTracker {
    private final Cell[] raised = {Cell.startingRaised(Alliance.RED), Cell.startingRaised(Alliance.BLUE)};
    private final Cell[] candidate = new Cell[2];
    private final int[] frames = new int[2];

    public Cell raised(Alliance alliance) {
        return raised[alliance.ordinal()];
    }

    public void setRaised(Cell cell) {
        raised[cell.alliance.ordinal()] = cell;
    }

    /** Driver override, for when the camera has not seen the Tip. */
    public void flip(Alliance alliance) {
        setRaised(raised(alliance).other());
    }

    public void observe(Sighting sighting) {
        int hive = sighting.cell.alliance.ordinal();
        Cell seenRaised = sighting.isRaised() ? sighting.cell : sighting.cell.other();
        if (seenRaised == raised[hive]) {
            frames[hive] = 0;
            return;
        }
        // A Hive swings through odd angles mid-Tip, so wait for several frames to agree.
        frames[hive] = seenRaised == candidate[hive] ? frames[hive] + 1 : 1;
        candidate[hive] = seenRaised;
        if (frames[hive] >= Constants.HIVE_FLIP_FRAMES) {
            raised[hive] = seenRaised;
            frames[hive] = 0;
        }
    }
}
