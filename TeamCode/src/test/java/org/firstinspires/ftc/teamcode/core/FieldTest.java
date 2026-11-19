package org.firstinspires.ftc.teamcode.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.Constants;
import org.junit.Test;

public class FieldTest {
    @Test
    public void blueIsRedSpunHalfWayRound() {
        Pose2d blue = Alliance.BLUE.fromRed(new Pose2d(10, -20, 0));
        assertEquals(-10, blue.x, 1e-9);
        assertEquals(20, blue.y, 1e-9);
        assertEquals(Math.PI, Math.abs(blue.heading), 1e-9);
    }

    @Test
    public void startingRaisedCellsAreMirrorImages() {
        Pose2d red = Cell.startingRaised(Alliance.RED).position();
        Pose2d blue = Cell.startingRaised(Alliance.BLUE).position();
        assertEquals(-red.x, blue.x, 1e-9);
        assertEquals(-red.y, blue.y, 1e-9);
    }

    @Test
    public void redIsOnTheNegativeYSide() {
        assertTrue(Cell.RED_AUDIENCE.y() < 0);
        assertTrue(Cell.BLUE_AUDIENCE.y() > 0);
        assertTrue(Cell.RED_AUDIENCE.x() > 0);
        assertTrue(Cell.RED_SCORING.x() < 0);
    }

    @Test
    public void clusterNamesMatchTheSdk() {
        assertEquals(Cell.RED_SCORING, Cell.fromClusterName("RED SCORING"));
        assertEquals(Cell.RED_AUDIENCE, Cell.fromClusterName("RED AUDIENCE"));
        assertEquals(Cell.BLUE_AUDIENCE, Cell.fromClusterName("BLUE AUDIENCE"));
        assertEquals(Cell.BLUE_SCORING, Cell.fromClusterName("BLUE SCORING"));
        assertNull(Cell.fromClusterName("Center Goal"));
    }

    @Test
    public void otherCellIsOnTheSameHive() {
        for (Cell cell : Cell.values()) {
            assertEquals(cell.alliance, cell.other().alliance);
            assertEquals(cell, cell.other().other());
            assertEquals(cell.y(), cell.other().y(), 1e-9);
            assertEquals(-cell.x(), cell.other().x(), 1e-9);
        }
    }

    /** G304: on our half, touching the wall, and not touching a Flower. */
    @Test
    public void startPosesAreLegal() {
        for (Alliance alliance : Alliance.values()) {
            for (boolean near : new boolean[] {true, false}) {
                Pose2d start = Field.startPose(alliance, near);
                double halfWidth = Constants.ROBOT_WIDTH_IN / 2;
                double halfLength = Constants.ROBOT_LENGTH_IN / 2;
                assertEquals("touching wall", Field.HALF, Math.abs(start.x) + halfLength, 1e-9);
                assertTrue("own half", alliance == Alliance.RED ? start.y < -halfWidth : start.y > halfWidth);
                for (Pose2d flower : Field.flowers()) {
                    boolean overlapsX = Math.abs(flower.x - start.x) < halfLength + 3.5;
                    boolean overlapsY = Math.abs(flower.y - start.y) < halfWidth + 3.5;
                    assertFalse("clear of Flower", overlapsX && overlapsY);
                }
            }
        }
    }

    @Test
    public void parkReachesTheLoadingZone() {
        Pose2d park = Field.parkPosition(Alliance.RED);
        assertTrue(park.x > -48 && park.x < -24);
        double nearestEdge = park.y - Constants.ROBOT_LENGTH_IN / 2;
        assertTrue("robot must overlap the zone", nearestEdge < -(Field.HALF - 11));
        assertTrue("robot must stay on the field", nearestEdge > -Field.HALF);
    }

    @Test
    public void shotTableInterpolatesAndClamps() {
        double[][] table = Constants.SHOT_TABLE;
        assertEquals(table[0][1], ShotTable.velocityFor(0), 1e-9);
        assertEquals(table[table.length - 1][1], ShotTable.velocityFor(500), 1e-9);
        double midpoint = (table[0][0] + table[1][0]) / 2;
        assertEquals((table[0][1] + table[1][1]) / 2, ShotTable.velocityFor(midpoint), 1e-9);
    }

    @Test
    public void shotTableCoversTheShotSpot() {
        double[][] table = Constants.SHOT_TABLE;
        for (int i = 1; i < table.length; i++) {
            assertTrue("rows must be shortest distance first", table[i][0] > table[i - 1][0]);
        }
        assertTrue("measure a row closer than the shot spot", table[0][0] < Constants.SHOT_DISTANCE_IN);
        assertTrue("measure a row farther than the shot spot",
                table[table.length - 1][0] > Constants.SHOT_DISTANCE_IN);
    }
}
