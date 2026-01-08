classDiagram
    %% The Brain
    class RaspberryPi4 {
        +I2C Bus 1 (GPIO 2/3)
        +I2C Bus 3 (GPIO 4/5)
        +GPIO Bank A (Left Motor)
        +GPIO Bank B (Right Motor)
        +CSI Port
    }

    %% Axis 1: Left
    class Motor_Left_System {
        +TMC2208 Driver
        +NEMA 17 Motor
        +AS5600 Encoder (Addr 0x36)
    }

    %% Axis 2: Right
    class Motor_Right_System {
        +TMC2208 Driver
        +NEMA 17 Motor
        +AS5600 Encoder (Addr 0x36)
    }

    %% Vision
    class GlobalShutterCam {
        +Sony IMX296
        +60 FPS
    }

    %% Connections
    RaspberryPi4 "I2C Bus 1" <..> "AS5600" Motor_Left_System : Feedback Loop A
    RaspberryPi4 "I2C Bus 3" <..> "AS5600" Motor_Right_System : Feedback Loop B (Isolated)
    
    RaspberryPi4 "GPIO 17/27" --> "STEP/DIR" Motor_Left_System : Motion A
    RaspberryPi4 "GPIO 10/9" --> "STEP/DIR" Motor_Right_System : Motion B
    
    RaspberryPi4 "CSI Port" <..> GlobalShutterCam : Image Data