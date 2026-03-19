// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.systems.field;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.subsystems.Dashboard;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.function.Supplier;

import lombok.Getter;
import lombok.RequiredArgsConstructor;

/**
 * Contains information for location of field element and other useful reference
 * points.
 *
 * <p>
 * NOTE: All constants are defined relative to the field coordinate system, and
 * from the
 * perspective of the blue alliance station
 */
public class FieldConstants {
    public static final AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    // AprilTag related constants
    public static final double aprilTagWidth = Units.inchesToMeters(6.5);

    // Field dimensions
    public static final double fieldLength = field.getFieldLength();
    public static final double fieldWidth = field.getFieldWidth();

    /**
     * Officially defined and relevant vertical lines found on the field (defined by
     * X-axis offset)
     */
    public static class LinesVertical {
        public static final double center = fieldLength / 2.0;
        public static final double starting = field.getTagPose(26).get().getX();
        public static final double allianceZone = starting;
        public static final double hubCenter = field.getTagPose(26).get().getX()
                + Hub.width / 2.0;
        public static final double neutralZoneNear = center - Units.inchesToMeters(120);
        public static final double neutralZoneFar = center + Units.inchesToMeters(120);
        public static final double oppHubCenter = field.getTagPose(4).get().getX()
                + Hub.width / 2.0;
        public static final double oppAllianceZone = field.getTagPose(10).get()
                .getX();
    }

    /**
     * Officially defined and relevant horizontal lines found on the field (defined
     * by Y-axis offset)
     *
     * <p>
     * NOTE: The field element start and end are always left to right from the
     * perspective of the
     * alliance station
     */
    public static class LinesHorizontal {

        public static final double center = fieldWidth / 2.0;

        // Right of hub
        public static final double rightBumpStart = Hub.nearRightCorner.getY();
        public static final double rightBumpEnd = rightBumpStart - RightBump.width;
        public static final double rightTrenchOpenStart = rightBumpEnd - Units.inchesToMeters(12.0);
        public static final double rightTrenchOpenEnd = 0;

        // Left of hub
        public static final double leftBumpEnd = Hub.nearLeftCorner.getY();
        public static final double leftBumpStart = leftBumpEnd + LeftBump.width;
        public static final double leftTrenchOpenEnd = leftBumpStart + Units.inchesToMeters(12.0);
        public static final double leftTrenchOpenStart = fieldWidth;
    }

    /** Hub related constants */
    public static class Hub {

        // Dimensions
        public static final double width = Units.inchesToMeters(47.0);
        public static final double height = Units.inchesToMeters(72.0); // includes the catcher at the top
        public static final double innerWidth = Units.inchesToMeters(41.7);
        public static final double innerHeight = Units.inchesToMeters(56.5);

        // Relevant reference points on alliance side
        public static final Pose2d HubPoseBlue = new Pose2d(
                new Translation2d(field.getTagPose(26).get().getX() + width / 2.0,
                        fieldWidth / 2.0),
                new Rotation2d());

        /** Blue alliance hub/speaker position (x, y) in inches. */
        public static final Translation2d BLUE_HUB_POSITION = new Translation2d(Inches.of(182.11).in(Meters),
                Inches.of(158.84).in(Meters));
        /** Red alliance hub/speaker position (x, y) in inches. */
        public static final Translation2d RED_HUB_POSITION = new Translation2d(Inches.of(469.11).in(Meters),
                Inches.of(158.84).in(Meters));

        public static Pose2d getHubPose() {
            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

            if (alliance == Alliance.Blue) {
                return new Pose2d(BLUE_HUB_POSITION, new Rotation2d());
            } else {
                return new Pose2d(RED_HUB_POSITION, new Rotation2d());
            }

        }

        public static Translation2d getHubTranslation2d() {
            return Dashboard.getAlliance() == Alliance.Blue
                    ? (BLUE_HUB_POSITION)
                    : (RED_HUB_POSITION);
        }

        public static final Translation3d topCenterPoint = new Translation3d(
                field.getTagPose(26).get().getX() + width / 2.0,
                fieldWidth / 2.0,
                height);
        public static final Translation3d innerCenterPoint = new Translation3d(
                field.getTagPose(26).get().getX() + width / 2.0,
                fieldWidth / 2.0,
                innerHeight);

        public static final Translation2d nearLeftCorner = new Translation2d(topCenterPoint.getX() - width / 2.0,
                fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d nearRightCorner = new Translation2d(topCenterPoint.getX() - width / 2.0,
                fieldWidth / 2.0 - width / 2.0);
        public static final Translation2d farLeftCorner = new Translation2d(topCenterPoint.getX() + width / 2.0,
                fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d farRightCorner = new Translation2d(topCenterPoint.getX() + width / 2.0,
                fieldWidth / 2.0 - width / 2.0);

        // Relevant reference points on the opposite side
        public static final Translation3d oppTopCenterPoint = new Translation3d(
                field.getTagPose(4).get().getX() + width / 2.0,
                fieldWidth / 2.0,
                height);
        public static final Translation2d oppNearLeftCorner = new Translation2d(oppTopCenterPoint.getX() - width / 2.0,
                fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d oppNearRightCorner = new Translation2d(oppTopCenterPoint.getX() - width / 2.0,
                fieldWidth / 2.0 - width / 2.0);
        public static final Translation2d oppFarLeftCorner = new Translation2d(oppTopCenterPoint.getX() + width / 2.0,
                fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d oppFarRightCorner = new Translation2d(oppTopCenterPoint.getX() + width / 2.0,
                fieldWidth / 2.0 - width / 2.0);

        // Hub faces
        public static final Pose2d nearFace = field.getTagPose(26).get().toPose2d();
        public static final Pose2d farFace = field.getTagPose(20).get().toPose2d();
        public static final Pose2d rightFace = field.getTagPose(18).get().toPose2d();
        public static final Pose2d leftFace = field.getTagPose(21).get().toPose2d();
    }

    /** Left Bump related constants */
    public static class LeftBump {

        // Dimensions
        public static final double width = Units.inchesToMeters(73.0);
        public static final double height = Units.inchesToMeters(6.513);
        public static final double depth = Units.inchesToMeters(44.4);

        // Relevant reference points on alliance side
        public static final Translation2d nearLeftCorner = new Translation2d(LinesVertical.hubCenter - width / 2,
                Units.inchesToMeters(255));
        public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
        public static final Translation2d farLeftCorner = new Translation2d(LinesVertical.hubCenter + width / 2,
                Units.inchesToMeters(255));
        public static final Translation2d farRightCorner = Hub.farLeftCorner;

        // Relevant reference points on opposing side
        public static final Translation2d oppNearLeftCorner = new Translation2d(LinesVertical.hubCenter - width / 2,
                Units.inchesToMeters(255));
        public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
        public static final Translation2d oppFarLeftCorner = new Translation2d(LinesVertical.hubCenter + width / 2,
                Units.inchesToMeters(255));
        public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
    }

    /** Right Bump related constants */
    public static class RightBump {
        // Dimensions
        public static final double width = Units.inchesToMeters(73.0);
        public static final double height = Units.inchesToMeters(6.513);
        public static final double depth = Units.inchesToMeters(44.4);

        // Relevant reference points on alliance side
        public static final Translation2d nearLeftCorner = new Translation2d(LinesVertical.hubCenter + width / 2,
                Units.inchesToMeters(255));
        public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
        public static final Translation2d farLeftCorner = new Translation2d(LinesVertical.hubCenter - width / 2,
                Units.inchesToMeters(255));
        public static final Translation2d farRightCorner = Hub.farLeftCorner;

        // Relevant reference points on opposing side
        public static final Translation2d oppNearLeftCorner = new Translation2d(LinesVertical.hubCenter + width / 2,
                Units.inchesToMeters(255));
        public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
        public static final Translation2d oppFarLeftCorner = new Translation2d(LinesVertical.hubCenter - width / 2,
                Units.inchesToMeters(255));
        public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
    }

    /** Left Trench related constants */
    public static class LeftTrench {
        // Dimensions
        public static final double width = Units.inchesToMeters(65.65);
        public static final double depth = Units.inchesToMeters(47.0);
        public static final double height = Units.inchesToMeters(40.25);
        public static final double openingWidth = Units.inchesToMeters(50.34);
        public static final double openingHeight = Units.inchesToMeters(22.25);

        // Relevant reference points on alliance side
        public static final Translation3d openingTopLeft = new Translation3d(LinesVertical.hubCenter, fieldWidth,
                openingHeight);
        public static final Translation3d openingTopRight = new Translation3d(LinesVertical.hubCenter,
                fieldWidth - openingWidth, openingHeight);

        // Relevant reference points on opposing side
        public static final Translation3d oppOpeningTopLeft = new Translation3d(LinesVertical.oppHubCenter, fieldWidth,
                openingHeight);
        public static final Translation3d oppOpeningTopRight = new Translation3d(LinesVertical.oppHubCenter,
                fieldWidth - openingWidth, openingHeight);
    }

    public static class RightTrench {

        // Dimensions
        public static final double width = Units.inchesToMeters(65.65);
        public static final double depth = Units.inchesToMeters(47.0);
        public static final double height = Units.inchesToMeters(40.25);
        public static final double openingWidth = Units.inchesToMeters(50.34);
        public static final double openingHeight = Units.inchesToMeters(22.25);

        // Relevant reference points on alliance side
        public static final Translation3d openingTopLeft = new Translation3d(LinesVertical.hubCenter, openingWidth,
                openingHeight);
        public static final Translation3d openingTopRight = new Translation3d(LinesVertical.hubCenter, 0,
                openingHeight);

        // Relevant reference points on opposing side
        public static final Translation3d oppOpeningTopLeft = new Translation3d(LinesVertical.oppHubCenter,
                openingWidth, openingHeight);
        public static final Translation3d oppOpeningTopRight = new Translation3d(LinesVertical.oppHubCenter, 0,
                openingHeight);
    }

    public static final ArrayList<Pose3d> TrenchPose = new ArrayList<>(4); // Starting at the right branch facing the
                                                                           // driver station in clockwise

    /** Tower related constants */
    public static class Tower {
        // Dimensions
        public static final double width = Units.inchesToMeters(49.25);
        public static final double depth = Units.inchesToMeters(45.0);
        public static final double height = Units.inchesToMeters(78.25);
        public static final double innerOpeningWidth = Units.inchesToMeters(32.250);
        public static final double frontFaceX = Units.inchesToMeters(43.51);

        public static final double uprightHeight = Units.inchesToMeters(72.1);

        // Rung heights from the floor
        public static final double lowRungHeight = Units.inchesToMeters(27.0);
        public static final double midRungHeight = Units.inchesToMeters(45.0);
        public static final double highRungHeight = Units.inchesToMeters(63.0);

        // Relevant reference points on alliance side
        public static final Translation2d centerPoint = new Translation2d(
                frontFaceX, field.getTagPose(31).get().getY());
        public static final Translation2d leftUpright = new Translation2d(
                frontFaceX,
                (field.getTagPose(31).get().getY())
                        + innerOpeningWidth / 2
                        + Units.inchesToMeters(0.75));
        public static final Translation2d rightUpright = new Translation2d(
                frontFaceX,
                (field.getTagPose(31).get().getY())
                        - innerOpeningWidth / 2
                        - Units.inchesToMeters(0.75));

        // Relevant reference points on opposing side
        public static final Translation2d oppCenterPoint = new Translation2d(
                fieldLength - frontFaceX,
                field.getTagPose(15).get().getY());
        public static final Translation2d oppLeftUpright = new Translation2d(
                fieldLength - frontFaceX,
                (field.getTagPose(15).get().getY())
                        + innerOpeningWidth / 2
                        + Units.inchesToMeters(0.75));
        public static final Translation2d oppRightUpright = new Translation2d(
                fieldLength - frontFaceX,
                (field.getTagPose(15).get().getY())
                        - innerOpeningWidth / 2
                        - Units.inchesToMeters(0.75));
    }

    public static class Depot {
        // Dimensions
        public static final double width = Units.inchesToMeters(42.0);
        public static final double depth = Units.inchesToMeters(27.0);
        public static final double height = Units.inchesToMeters(1.125);
        public static final double distanceFromCenterY = Units.inchesToMeters(75.93);

        // Relevant reference points on alliance side
        public static final Translation3d depotCenter = new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY,
                height);
        public static final Translation3d leftCorner = new Translation3d(depth,
                (fieldWidth / 2) + distanceFromCenterY + (width / 2), height);
        public static final Translation3d rightCorner = new Translation3d(depth,
                (fieldWidth / 2) + distanceFromCenterY - (width / 2), height);
    }

    public static class Outpost {
        // Dimensions
        public static final double width = Units.inchesToMeters(31.8);
        public static final double openingDistanceFromFloor = Units.inchesToMeters(28.1);
        public static final double height = Units.inchesToMeters(7.0);

        // Relevant reference points on alliance side
        public static final Translation2d centerPoint = new Translation2d(0,
                field.getTagPose(29).get().getY());
    }

   
}