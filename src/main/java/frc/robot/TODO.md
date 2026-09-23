-lib:
    -Check FieldLayout.Java

-Subsystems:
    -Swerve:
        -Rename it to drive[]
        -Merge the two code bases[]
    -Intake:
        ~~-Add Pivot[x]~~
        -Move Rack and Pinion to Intake[]
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
        -Add 1678's Shooting logic[x]
            ~~-Shooting verification(Hub or ferry)[]~~
            -Shooting state(Enum)[x]
            -Regression[x]:
                -Less than 6m = Interpolating LUTs[x]
                    -3 Tables, Shooting velocity, hood angle, and air time
                -Greater than 6m = Line of best fit[x]
            -Motion Compentation[x]
                -Launch Delay(Current x,y position + velocity*launch delay)[x]
                -Shooter offset(velocity(x,y) + distance between center and shooter * rotational velocity * cos(phi)/sin(phi), where phi = theta + the positve or negative sign of the rotational velocity * pi/2);Can be used to with a twist2d[x]
                -Look Ahead value(x and y lookahead = Launch Delay + Shooter Offset Velocity * time of flight(movement))[x]
                -Rejection conditions(Yaw stability,error tolerances, and static shots)[x]
                -atan2(dx,dy), where dx/dy = Look ahead - current pose[x]
                -PID to rotation(either pid or call a swerve request)[x]
            -Fitlering[x]
                -Tilt[x]
                -Roll[x]
        -Tune motors
            -PID[]
            -Voltage[]
            -Look Up/Interpolation Table[]

-SuperStructure:
    -Research Superstructure and how 1678's structures their class[]
    -Implement it(again will elaborate once I figure this out)[]
    - Run intake P  X
        - stick out rack in pintoin && rollers
    - Take in balls P
        - Run intake && Run Hopper rolls
    - Shooting B
        - Motsoin comp shooting && Hopper rollers running && intake rack and pinton osolation && Lower hopper
    - Outtake P X
        - Review run intake
    - Upward Exting P X
        - Raise hopper
    - Auto Alline B
        - Mostion comp rotate (only drivebase) 
    - Farry Shooting (passing) B
        - 
    - Lower hopper P X
        - sets hopper to low
    - User actions 
        - Standard drivebase
        - Button: Shoots 
            - Shooting
        - Button: AutoAline
            - Auto Alline
        - Button: Farry Shooting Right
            - Farry Shooting Right
        - Button: Farry Shooting Left
            - Farry Shooting Left
        - Button: Autoline to the hub
            - Auto Aline
        - Button: Intake
            - Run intake
        - Button: Outtake
            - Run Outtake
        - Button: Flip state
            - IF up
                - Lower
            - IF down
                - Upper

-wpilibj2.commands:
    -3 Axis PID[]
        -Calculate PID 3 PID logic(X,Y,Theta)[]

-Testing Setup:
    -Controls on Elastic[x]
        -Hood Angle[x]
        -Velocity[x]
        -Display Distance from the Hub[x]


Testing:
    -Get the shooter delay[]
        -Get the amperage of the feed rollers, read until it dips. 
            Once it dips you start a timer until the amperage of the shooter roller dips.
    -Test LUTs[]
    -Test Hopper[]
    -Double check intake[]

-Autos(As of right now things are unsure):
    -Try intergrating Pathplanner and Choreo(If avaliable)[]
        -If not, figure out something else[]

-Misc:
    -Add Telemetary[x]
    -Add Battery Checker[]
    -Add a Match Timer[]
        -Add rumble to the controller[]
    -Bind everything[]
    

