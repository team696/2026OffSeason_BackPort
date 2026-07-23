-lib:
    -Check FieldLayout.Java

-Subsystems:
    -Swerve:
        -Rename it to drive[]
        -Merge the two code bases[]
    -Intake:
        -Add Pivot[x]
        -Add Rollers[x]
        -Tune motors
            -PID[]
            -Voltage[]
    -Hopper:
        -Add Rollers[x]
        -Add Rack and Pinion[x]
        -~~Add tilt for the Rollers[]~~
        -Add Upward Extending Hopper[x]
        -Tune motors
            -PID[]
            -Voltage[]
    -Shooter:
        -Add Shooter Rollers[x]
        -Add Feeding Rollers[x]
        -Add Hood[x]
        -Add 1678's Shooting logic[]
            ~~-Shooting verification(Hub or ferry)[]~~
            -Shooting state(Enum)[]
            -Regression[x]:
                -Less than 6m = Interpolating LUTs[x]
                    -3 Tables, Shooting velocity, hood angle, and air time
                -Greater than 6m = Line of best fit[x]
            -Motion Compentation[]
                -Launch Delay(Current x,y position + velocity*launch delay)[]
                -Shooter offset(velocity(x,y) + distance between center and shooter * rotational velocity * cos(phi)/sin(phi), where phi = theta + the positve or negative sign of the rotational velocity * pi/2);Can be used to with a twist2d[]
                -Look Ahead value(x and y lookahead = Launch Delay + Shooter Offset Velocity * time of flight(movement))[]
                -Rejection conditions(Yaw stability,error tolerances, and static shots)[]
                -atan2(dx,dy), where dx/dy = Look ahead - current pose[]
                -PID to rotation(either pid or call a swerve request)[]
            -Fitlering[]
                -WIP
        -Tune motors
            -PID[]
            -Voltage[]
            -Look Up/Interpolation Table[]

-SuperStructure:
    -Research Superstructure and how 1678's structures their class[]
    -Implement it(again will elaborate once I figure this out)[]

-wpilibj2.commands:
    -3 Axis PID[]
        -Calculate PID 3 PID logic(X,Y,Theta)[]

-Testing Setup:
    -Controls on Elastic[]
        -Hood Angle[]
        -Velocity[]
        -Display Distance from the Hub[]


-Autos(As of right now things are unsure):
    -Try intergrating Pathplanner and Choreo(If avaliable)[]
        -If not, figure out something else[]

-Misc:
    -Add Telemetary[]
    -Add Battery Checker[]
    -Add a Match Timer[]
        -Add rumble to the controller[]
    -Bind everything[]
    

