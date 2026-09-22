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
            scoringvelocityLUT.put(1.5, 30.0); // meters, RPS
            scoringvelocityLUT.put(2.5, 30.0);
            scoringvelocityLUT.put(3.5, 30.0);
            scoringvelocityLUT.put(4.5, 30.0);
            scoringvelocityLUT.put(5.5, 30.0);
        }
    }

    public class ScoringHoodLUT{
        public static final InterpolatingDoubleTreeMap scoringhoodLUT = new InterpolatingDoubleTreeMap();

        static{
            scoringhoodLUT.put(1.5, 28.0); // meters, Degrees
            scoringhoodLUT.put(2.5, 34.0);
            scoringhoodLUT.put(3.5, 42.0);
            scoringhoodLUT.put(4.5, 50.0);
            scoringhoodLUT.put(5.5, 56.0);
        }
    }

    public class NetrualZoneVelocityLUT{
        public static final InterpolatingDoubleTreeMap neturalzonevelocityLUT = new InterpolatingDoubleTreeMap();

        static{
            neturalzonevelocityLUT.put(3.0, 30.0); // meters, RPS
            neturalzonevelocityLUT.put(4.5, 30.0);
            neturalzonevelocityLUT.put(6.0, 30.0);
            neturalzonevelocityLUT.put(7.5, 30.0);
        }
    }

    public class NeturalZoneHoodLUT{
        public static final InterpolatingDoubleTreeMap neturalzonehoodLUT = new InterpolatingDoubleTreeMap();

        static{
            neturalzonehoodLUT.put(3.0, 20.0); // meters, Degrees
            neturalzonehoodLUT.put(4.5, 22.0);
            neturalzonehoodLUT.put(6.0, 24.0);
            neturalzonehoodLUT.put(7.5, 26.0);
        }

    }

    public class OppositeSideVelocityLUT{
        public static final InterpolatingDoubleTreeMap oppositesidevelocityLUT = new InterpolatingDoubleTreeMap();

        static{
            oppositesidevelocityLUT.put(7.0, 30.0); // meters, RPS
            oppositesidevelocityLUT.put(9.0, 30.0);
            oppositesidevelocityLUT.put(11.0, 30.0);
            oppositesidevelocityLUT.put(13.0, 30.0);
        }

    }

    public class OppositeSideHoodLUT{
        public static final InterpolatingDoubleTreeMap oppositesidehoodLUT = new InterpolatingDoubleTreeMap();

        static{
            oppositesidehoodLUT.put(7.0, 25.0); // meters, Degrees
            oppositesidehoodLUT.put(9.0, 27.5);
            oppositesidehoodLUT.put(11.0, 30.0);
            oppositesidehoodLUT.put(13.0, 32.0);
        
    }

    }
}
