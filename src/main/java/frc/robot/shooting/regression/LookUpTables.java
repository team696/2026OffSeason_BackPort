// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooting.regression;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Add your docs here. */
public class LookUpTables {
    public class ScoringVelocityLUT{
        public static final InterpolatingDoubleTreeMap scoringvelocityLUT = new InterpolatingDoubleTreeMap();

        static{
            scoringvelocityLUT.put(0.0, 0.0); //meters ,RPS
            scoringvelocityLUT.put(1.0,0.0);
            scoringvelocityLUT.put(2.0, 0.0);
        }
    }

    public class ScoringHoodLUT{
        public static final InterpolatingDoubleTreeMap scoringhoodLUT = new InterpolatingDoubleTreeMap();

        static{
            scoringhoodLUT.put(0.0, 0.0); //meter, Degrees
            scoringhoodLUT.put(1.0,0.0);
            scoringhoodLUT.put(2.0, 0.0);
        }
    }

    public class NetrualZoneVelocityLUT{
        public static final InterpolatingDoubleTreeMap neturalzonevelocityLUT = new InterpolatingDoubleTreeMap();

        static{
            neturalzonevelocityLUT.put(0.0, 0.0); //meters ,RPS
            neturalzonevelocityLUT.put(1.0,0.0);
            neturalzonevelocityLUT.put(2.0, 0.0);
        }
    }

    public class NeturalZoneHoodLUT{
        public static final InterpolatingDoubleTreeMap neturalzonehoodLUT = new InterpolatingDoubleTreeMap();

        static{
            neturalzonehoodLUT.put(0.0, 0.0); //meters ,Degrees
            neturalzonehoodLUT.put(1.0,0.0);
            neturalzonehoodLUT.put(2.0, 0.0);
        }

    }

    public class OppositeSideVelocityLUT{
        public static final InterpolatingDoubleTreeMap oppositesidevelocityLUT = new InterpolatingDoubleTreeMap();

        static{
            oppositesidevelocityLUT.put(0.0, 0.0); //meters ,RPS
            oppositesidevelocityLUT.put(1.0,0.0);
            oppositesidevelocityLUT.put(2.0, 0.0);
        }

    }

    public class OppositeSideHoodLUT{
        public static final InterpolatingDoubleTreeMap oppositesidehoodLUT = new InterpolatingDoubleTreeMap();

        static{
            oppositesidehoodLUT.put(0.0, 0.0); //meters ,Degrees
            oppositesidehoodLUT.put(1.0,0.0);
            oppositesidehoodLUT.put(2.0, 0.0);
        
    }

    }
}
