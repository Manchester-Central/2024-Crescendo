// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.MathUtil;

/** Add your docs here. */
public class ControllerUtil {
    public static double getAdjustedRotationValue(double x, double y){
        var numirator = x + y;
        var denominator = Math.sqrt(2);
        var result = numirator / denominator;
        return -MathUtil.clamp(result, -1, 1);
    }
}
