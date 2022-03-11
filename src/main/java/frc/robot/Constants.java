// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.SimVisionTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class Vision{
        // public static final double targetWidth =

        //     Units.inchesToMeters(91.0) - Units.inchesToMeters(6.70); // meters

        //     // See

        //     // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf

        //     // page 197

            public static final double targetHeight =

            Units.inchesToMeters(91.0); // meters

            public static final double targetHeightAboveGround = Units.inchesToMeters(81.19); // meters

            // See
            // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/LayoutandMarkingDiagram.pdf

            // pages 4 and 5

            public static final double kFarTgtXPos = 1.0;

            public static final String kCamName = "targetCam";
            public static final Translation2d translation = new Translation2d(.5, 0);
            public static final Rotation2d rotation = new Rotation2d(0.0);
            public static final Transform2d kCameraToRobot = new Transform2d(translation, rotation);
    }

    public static final double kTurnRateToleranceDegPerS = 10;
    public static final double kTurnToleranceDeg = 10;
    public static final double kTurnP = 0.01;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0.0001;
}
