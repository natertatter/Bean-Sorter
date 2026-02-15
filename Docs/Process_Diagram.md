# Sorter MVP — Process Flow Diagram

Process flow derived from the hardware architecture in `Diagram.md.txt`.

## Data Flow Process

```mermaid
flowchart TB
    subgraph RPi["Raspberry Pi 4"]
        Start([Start])
        ReadPos[Read Position Data]
        Process[Process / Decide Movement]
        SendCmd[Send Motor Commands]
        ReadReply[Read UART Reply]
        End([Continue Loop])
    end

    subgraph LeftAxis["Left Axis (I2C Bus 1, UART0)"]
        L_AS5600[AS5600 Encoder<br/>Addr 0x36]
        L_TMC[TMC2208 Driver<br/>Addr 0]
    end

    subgraph RightAxis["Right Axis (I2C Bus 3, UART5)"]
        R_AS5600[AS5600 Encoder<br/>Addr 0x36]
        R_TMC[TMC2208 Driver<br/>Addr 0]
    end

    subgraph Wiring["UART Wiring"]
        UART_Node[UART Wiring Node<br/>1k Resistor · Merge TX/RX]
    end

    Start --> ReadPos
    ReadPos --> |"I2C Bus 1"| L_AS5600
    ReadPos --> |"I2C Bus 3"| R_AS5600
    L_AS5600 --> Process
    R_AS5600 --> Process
    Process --> SendCmd
    SendCmd --> |"GPIO 14 TX"| UART_Node
    SendCmd --> |"GPIO 12 TX"| UART_Node
    UART_Node --> |"PDN_UART"| L_TMC
    UART_Node --> |"PDN_UART"| R_TMC
    L_TMC --> |"GPIO 15 RX"| ReadReply
    R_TMC --> |"GPIO 13 RX"| ReadReply
    ReadReply --> End
    End --> ReadPos
```

## Sequence Diagram (Communication Flow)

```mermaid
sequenceDiagram
    participant RPi as Raspberry Pi 4
    participant I2C_L as I2C Bus 1
    participant I2C_R as I2C Bus 3
    participant UART as UART Wiring Node
    participant Left as Left Motor System
    participant Right as Right Motor System

    loop Control Loop
        RPi->>I2C_L: Read position (AS5600 @ 0x36)
        RPi->>I2C_R: Read position (AS5600 @ 0x36)
        I2C_L->>RPi: Left axis position
        I2C_R->>RPi: Right axis position
        RPi->>RPi: Compute movement
        RPi->>UART: TX motor command (GPIO 14/12)
        UART->>Left: PDN_UART → TMC2208
        UART->>Right: PDN_UART → TMC2208
        Left->>RPi: Reply (GPIO 15 RX)
        Right->>RPi: Reply (GPIO 13 RX)
    end
```

## Simplified Process Overview

```mermaid
flowchart LR
    A[Raspberry Pi] -->|I2C| B[Read Position<br/>AS5600 x2]
    A -->|UART TX| C[UART Wiring Node]
    C -->|PDN_UART| D[Left TMC2208]
    C -->|PDN_UART| E[Right TMC2208]
    D -->|UART RX| A
    E -->|UART RX| A
    B --> A
```
