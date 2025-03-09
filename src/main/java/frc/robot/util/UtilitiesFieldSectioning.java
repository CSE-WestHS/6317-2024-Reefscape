// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;


import static edu.wpi.first.units.Units.FeetPerSecond;

import java.util.List;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.GoToPositionElevator;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.vision.Vision;

/** Add your docs here. */
public class UtilitiesFieldSectioning {
    //scoring positions
    public static final Pose2d NearCenterLeft = new Pose2d(2.915,4.190,Rotation2d.fromDegrees(1.169));
    public static final Pose2d NearCenterRight = new Pose2d(2.870,3.845,Rotation2d.fromDegrees(1.169));
    public static final Pose2d FarCenterLeft = new Pose2d(5.965,4.19,Rotation2d.fromDegrees(180));
    public static final Pose2d FarCenterRight = new Pose2d(5.920,3.860,Rotation2d.fromDegrees(180));
    public static final Pose2d NearLeftNear = new Pose2d(3.576,5.167,  Rotation2d.fromDegrees(-56.023));
    public static final Pose2d NearLeftFar = new Pose2d(3.877,5.302, Rotation2d.fromDegrees(-56.023));
    public static final Pose2d NearRightNear = new Pose2d(3.546, 2.778, Rotation2d.fromDegrees(58.325));
    public static final Pose2d NearRightFar = new Pose2d(3.862, 2.643, Rotation2d.fromDegrees(58.325));
    public static final Pose2d FarLeftNear = new Pose2d(5.109, 5.377, Rotation2d.fromDegrees(-119.249));
    public static final Pose2d FarLeftFar = new Pose2d(5.364, 5.242, Rotation2d.fromDegrees(-119.249));
    public static final Pose2d FarRightNear = new Pose2d(5.094, 2.703, Rotation2d.fromDegrees(123.024));
    public static final Pose2d FarRightFar = new Pose2d(5.409, 2.808, Rotation2d.fromDegrees(123.024));

    //sections
    public static final Pose2d S1 = new Pose2d(5.359,5.559,Rotation2d.fromDegrees(-114.228)); //section 1
    public static final Pose2d S2 = new Pose2d(6.533,4.169,Rotation2d.fromDegrees(-180.000)); //section 2
    public static final Pose2d S3 = new Pose2d(5.454,2.539,Rotation2d.fromDegrees(125.395)); //section 3
    public static final Pose2d S4 = new Pose2d(3.333,2.563,Rotation2d.fromDegrees(62.904)); //section 4
    public static final Pose2d S5 = new Pose2d(2.493,4.049,Rotation2d.fromDegrees(7.883)); //section 5
    public static final Pose2d S6 = new Pose2d(3.488,5.428,Rotation2d.fromDegrees(-55.886)); //section 6
    public static final Pose2d F1 = new Pose2d(1.858, 6.590,Rotation2d.fromDegrees(-48.832) ); //feed station
    //pid
    public static final ProfiledPIDController angleController = new ProfiledPIDController(0.5,0, 0, new Constraints(DriveCommands.ANGLE_MAX_VELOCITY, DriveCommands.ANGLE_MAX_ACCELERATION));
    
    //auto tolerances
    private static double tolerance = 0.2; 

    
        //array of positions
        public static final Pose2d[] sectionsArr = {FarCenterLeft,FarCenterRight,NearCenterLeft,NearCenterRight,FarLeftFar,FarLeftNear,FarRightFar,FarRightNear,NearRightFar,NearRightNear,NearLeftFar,NearRightNear,F1};
        public static final Pose2d[] poseArr = {FarCenterLeft,FarCenterRight,NearCenterLeft,NearCenterRight,FarLeftFar,FarLeftNear,FarRightFar,FarRightNear,NearRightFar,NearRightNear,NearLeftFar,NearRightNear,F1};
        
        /***
         * 
         * @param currentPose
         * @return closest section's pose
         */
        public static Pose2d getClosestSection(Pose2d currentPose) {
            Pose2d currentClosest = new Pose2d();
            double currentDistanceFromPoint = 999999; //set high so that no element is auto selected - will probably delete later
            double minDistance = currentDistanceFromPoint;
            for (int i = 0; i < sectionsArr.length; ++i) {
                // d = √(x2 - x1)2 + (y2 - y1)2
                currentDistanceFromPoint = Math.sqrt(Math.pow(sectionsArr[i].getX() - currentPose.getX(),2) + Math.pow(sectionsArr[i].getY() - currentPose.getY(), 2));
                if (currentDistanceFromPoint < minDistance) {
                    minDistance = currentDistanceFromPoint;
                    currentClosest = sectionsArr[i];
                }
            }
            return currentClosest;
        }

        /**
         * 
         * @param currentPose robots current pose
         * @param reefPose pose of the specific reef section
         * @param drive drive subsystem
         */
        public static void faceSpecificReef(Pose2d currentPose, Pose2d reefPose, Drive drive) {
            angleController.enableContinuousInput(-Math.PI, Math.PI);
            angleController.setTolerance(Units.degreesToRadians(10));
            double omega = angleController.calculate(currentPose.getRotation().getRadians(), reefPose.getRotation().getRadians());
            ChassisSpeeds speeds = new ChassisSpeeds(0, 0, -omega);
            boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                isFlipped
                    ? drive.getRotation().plus(new Rotation2d(Math.PI))
                    : drive.getRotation());
            drive.runVelocity(speeds);
        }

        /**
         * 
         * @param drive
         * @param elevator
         * @see .. This function takes current pose and based on pose changes elevator height
         */
        public static boolean changeElevatorHeightBasedOnPose(Drive drive, Elevator elevator) {
            if (drive.getPose().nearest(List.of(sectionsArr)) == F1) {
                return true;
            }
            else {
                return false;
            }
        }

        public static boolean shouldShoot(Drive drive) {
            if (getClosestSectionDistance(drive.getPose()) < 100) {
                return true;
            }
            return false;
        }


        /**
         * 
         * @param currentPose current pose of robot
         * @return name of reef section
         */
        public static String getClosestSectionName(Pose2d currentPose) {
            Pose2d closest = getClosestSection(currentPose);
            angleController.enableContinuousInput(-Math.PI, Math.PI);
            int value = 0;
            for (int i = 0; i < sectionsArr.length;++i) {
                if (sectionsArr[i] == closest) {
                    value = i;
                }
            }
            if (value <= 5) {
                return "S" + (value+1);
            }
            else {
                return "F" + (value - 5);
            }
        }

        /**
         * 
         * @param currentPose current robot pose
         * @param drive drive subsystem
         */
        public static void faceClosestReef(Pose2d currentPose, Drive drive) {
            Pose2d closest = currentPose.nearest(List.of(sectionsArr));
            // angleController.enableContinuousInput(-Math.PI, Math.PI);
            // angleController.setTolerance(0.349066);
            // double omega = angleController.calculate(currentPose.getRotation().getRadians(), closest.getRotation().getRadians());
            // ChassisSpeeds speeds = new ChassisSpeeds(0, 0, -omega);
            // boolean isFlipped =
            //       DriverStation.getAlliance().isPresent()
            //           && DriverStation.getAlliance().get() == Alliance.Red;
            // speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            //     speeds,
            //     isFlipped
            //         ? drive.getRotation().plus(new Rotation2d(Math.PI))
            //         : drive.getRotation());
            // drive.runVelocity(speeds);
            Commands.run(()->DriveCommands.joystickDriveAtAngle(drive, ()->0, ()->0, ()->closest.getRotation() ));

    }
    /**
     * 
     * @param currentPose
     * @return distance from the closest april tag pose
     */
    public static double getClosestSectionDistance(Pose2d currentPose) {
        double currentDistanceFromPoint = 999999; //set high so that no element is auto selected - will probably delete later
        double minDistance = currentDistanceFromPoint;
        for (int i = 0; i < poseArr.length; ++i) {
            // d = √(x2 - x1)2 + (y2 - y1)2
            currentDistanceFromPoint = Math.sqrt(Math.pow(poseArr[i].getX() - currentPose.getX(),2) + Math.pow(poseArr[i].getY() - currentPose.getY(), 2));
            if (currentDistanceFromPoint < minDistance) {
                minDistance = currentDistanceFromPoint;
            }
        }
        return minDistance;
    }
    /**
     * 
     * @param currentPose
     * @purpose If close to reef, makes max speed slow, otherwise normal speed
     */
    public static void isCloseToReef(Pose2d currentPose) {
        if (getClosestSectionDistance(currentPose) <= 0.305 && RobotContainer.hasShotCoral == false) {
            DriveConstants.maxSpeedAt12Volts = FeetPerSecond.of(2);
        }
        else {
            DriveConstants.maxSpeedAt12Volts = FeetPerSecond.of(8);
        }
        System.out.println(DriveConstants.maxSpeedAt12Volts);
    }
}
