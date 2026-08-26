# Driver Game Pad:
- Left Joy Stick - Forward / Backwards Right / Left
- Right Joy Stick - Turn Robot Clockwise or Counter Clockwise
- Right Trigger - Spin In - Pickup balls??
- Left Trigger - Shoot??
- A Button - Zero Gyro
- Back -- Home All devices

# Limelight Coordinate Definition 
- +X points forward
- +Y Points to the right
- +Z Points upward 

# Camera Parameters for rear facing camera from 8-23-26
- Forward:-.3285 Right:0 Up:.42 Roll:0 Pitch:25 Yaw:180

# Camera Parameters for right facing camera from 8-23-26
- Forward:-.0745 Right:-.3255 Up:.2 Roll:0 Pitch:25 Yaw:-90

# Camera Parameters for left facing camera from 8-23-26
- Since camera was broken these need to be validated
- Forward:-.0745 Right:.3255 Up:.2 Roll:0 Pitch:25 Yaw:90

# You can run spotless in many ways:
- In the command line you can type   ./gradlew spotlessApply
- You can also type shift-crtl p and then find Run Task hit enter and then click on "Format with Spotless"

# Help with java items
- In VSCode Settings: {"[java]": {"spotlessGradle.diagnostics.enable": false}}
- In java to convert degrees to an angle use -> Degrees.of(90)
- In java to convert an RPM value to an angular velocity use -> RPM.of(-500)
- Avoid the unused warning by adding to the line above the warning -- @SuppressWarnings("unused")
