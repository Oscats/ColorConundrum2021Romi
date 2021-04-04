// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;

/** This is a PoJo for holding our waypoint lists. 
 * There must be a better way to organize this, but I have not figured it out yet :)
 */
public class Waypoints {
    private double[] pointDefault = {0,0};
    public List<Translation2d> getPathAWaypoints(){
        List<Translation2d> aWaypoints = List.of(
            /**Put your Waypoints here. 
             * Units should be in Meters, but you can use the 
             * Units.inchesToMeters() method to convert them.
             */			
            new Translation2d(Units.inchesToMeters(37.5), Units.inchesToMeters(21)),  //1 <-- First Loop
            new Translation2d(Units.inchesToMeters(43.5), Units.inchesToMeters(15)), //2
            new Translation2d(Units.inchesToMeters(37.5), Units.inchesToMeters(9)), //3
            new Translation2d(Units.inchesToMeters(31.5), Units.inchesToMeters(15)), //4
            new Translation2d(Units.inchesToMeters(37.5), Units.inchesToMeters(21)), //5 <-- First Loop
            new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(24)), //6 <-- Second Loop
            new Translation2d(Units.inchesToMeters(66.5), Units.inchesToMeters(30)), //7
            new Translation2d(Units.inchesToMeters(59.5), Units.inchesToMeters(36.5)), //8
            new Translation2d(Units.inchesToMeters(53), Units.inchesToMeters(30)), //9 <-- Second Loop
            new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(20)), //10 <-- Third Loop
            new Translation2d(Units.inchesToMeters(75), Units.inchesToMeters(7.5)), //11
            new Translation2d(Units.inchesToMeters(80), Units.inchesToMeters(15.5)), //12 <-- Third Loop
            new Translation2d(Units.inchesToMeters(75), Units.inchesToMeters(22.5)), //13 <-- Return
            new Translation2d(Units.inchesToMeters(40), Units.inchesToMeters(21.5)), //13.5
            new Translation2d(Units.inchesToMeters(15), Units.inchesToMeters(20)) //14						        	
            //)
        );
        return aWaypoints;
    }
    public List <Pose2d> getPathAPoses(){
        /**TThis Method Contains the Poses.
         * The first one is the starting pose.
         * The second one is the ending pose.
         */
        List <Pose2d> aPoses = List.of(
            new Pose2d(Units.inchesToMeters(7.5), Units.inchesToMeters(22),new Rotation2d(0)),
            new Pose2d(Units.inchesToMeters(7.5), Units.inchesToMeters(22), new Rotation2d(180))
        );
        return aPoses;
    }
    public List<Translation2d> getWaypoints(){

        double[] waypoint1 = SmartDashboard.getNumberArray("Waypoint 1", pointDefault);
        double[] waypoint2 = SmartDashboard.getNumberArray("Waypoint 2", pointDefault);
        double[] waypoint3 = SmartDashboard.getNumberArray("Waypoint 3", pointDefault);
        double[] waypoint4 = SmartDashboard.getNumberArray("Waypoint 4", pointDefault);
        double[] waypoint5 = SmartDashboard.getNumberArray("Waypoint 5", pointDefault);
        double[] waypoint6 = SmartDashboard.getNumberArray("Waypoint 6", pointDefault);
        double[] waypoint7 = SmartDashboard.getNumberArray("Waypoint 7", pointDefault);
        double[] waypoint8 = SmartDashboard.getNumberArray("Waypoint 8", pointDefault);
        double[] waypoint9 = SmartDashboard.getNumberArray("Waypoint 9", pointDefault);
        double[] waypoint10 = SmartDashboard.getNumberArray("Waypoint 910", pointDefault);
        double[] waypoint11 = SmartDashboard.getNumberArray("Waypoint 911", pointDefault);
        double[] waypoint12 = SmartDashboard.getNumberArray("Waypoint 912", pointDefault);
        double[] waypoint13 = SmartDashboard.getNumberArray("Waypoint 913", pointDefault);
        double[] waypoint14 = SmartDashboard.getNumberArray("Waypoint 914", pointDefault);
        double[] waypoint15 = SmartDashboard.getNumberArray("Waypoint 915", pointDefault);

        List <Translation2d> myWaypoints = new ArrayList<Translation2d>();
          myWaypoints.add (
                        new Translation2d(Units.inchesToMeters(waypoint1[0]),Units.inchesToMeters( waypoint1[1])
            ));
            if (waypoint2[0] != 0) myWaypoints.add(         new Translation2d(Units.inchesToMeters(waypoint2[0]),
            Units.inchesToMeters(waypoint2[1])));
            if (waypoint3[0] != 0) myWaypoints.add(         new Translation2d(Units.inchesToMeters(waypoint3[0]),
            Units.inchesToMeters(waypoint3[1])));
            if (waypoint4[0] != 0) myWaypoints.add(         new Translation2d(Units.inchesToMeters(waypoint4[0]),
            Units.inchesToMeters(waypoint4[1])));
            if (waypoint5[0] != 0) myWaypoints.add(         new Translation2d(Units.inchesToMeters(waypoint5[0]),
            Units.inchesToMeters(waypoint5[1])));
            if (waypoint6[0] != 0) myWaypoints.add(         new Translation2d(Units.inchesToMeters(waypoint6[0]),
            Units.inchesToMeters(waypoint6[1])));
            if (waypoint7[0] != 0) myWaypoints.add(         new Translation2d(Units.inchesToMeters(waypoint7[0]),
            Units.inchesToMeters(waypoint7[1])));
            if (waypoint8[0] != 0) myWaypoints.add(         new Translation2d(Units.inchesToMeters(waypoint8[0]),
            Units.inchesToMeters(waypoint8[1])));
            if (waypoint9[0] != 0) myWaypoints.add(         new Translation2d(Units.inchesToMeters(waypoint9[0]),
            Units.inchesToMeters(waypoint9[1])));
            if (waypoint10[0] != 0) myWaypoints.add(            new Translation2d(Units.inchesToMeters(waypoint10[0]),
            Units.inchesToMeters(waypoint10[1])));
            if (waypoint11[0] != 0) myWaypoints.add(            new Translation2d(Units.inchesToMeters(waypoint11[0]),
            Units.inchesToMeters(waypoint11[1])));
            if (waypoint12[0] != 0) myWaypoints.add(            new Translation2d(Units.inchesToMeters(waypoint12[0]),
            Units.inchesToMeters(waypoint12[1])));
            if (waypoint13[0] != 0) myWaypoints.add(            new Translation2d(Units.inchesToMeters(waypoint13[0]),
            Units.inchesToMeters(waypoint13[1])));
            if (waypoint14[0] != 0) myWaypoints.add(            new Translation2d(Units.inchesToMeters(waypoint14[0]),
            Units.inchesToMeters(waypoint14[1])));
            if (waypoint15[0] != 0) myWaypoints.add(            new Translation2d(Units.inchesToMeters(waypoint15[0]),
            Units.inchesToMeters(waypoint15[1])));
    return myWaypoints;
}
}

