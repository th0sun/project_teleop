# Dobot MG400 Teleoperation System Architecture & Data Flow

This document details the data flow and latency analysis pipeline for the Dobot MG400 Teleoperation System, running on Ubuntu 24.04 and ROS 2 Jazzy.

## System Architecture & Data Flow Pipeline

```mermaid
graph TD
    %% Styling
    classDef userLayer fill:#f0f9ff,stroke:#0284c7,stroke-width:2px;
    classDef rosLayer fill:#f0fdf4,stroke:#16a34a,stroke-width:2px;
    classDef commLayer fill:#fffbeb,stroke:#eab308,stroke-width:2px;
    classDef robotLayer fill:#fef2f2,stroke:#dc2626,stroke-width:2px;
    classDef feedbackLayer fill:#fdf4ff,stroke:#c026d3,stroke-width:2px;
    classDef vizLayer fill:#f8fafc,stroke:#475569,stroke-width:2px,stroke-dasharray: 5 5;

    %% 1. User Input Layer
    subgraph User Input Layer
        UI_KB[Keyboard Teleop]:::userLayer
        UI_VR[Controller Input / Unity]:::userLayer
    end

    %% 2. ROS 2 Layer
    subgraph ROS 2 Layer
        ROS_TELEOP[Teleop Node <br> Generates JointTrajectory / Motion]:::rosLayer
        ROS_TOPIC_CMD[[Topic: /unity/joint_cmd / /joint_trajectory]]:::rosLayer
        ROS_CTRL[Controller Node]:::rosLayer
        ROS_DRV[MG400 ROS 2 Driver]:::rosLayer
    end

    %% 3. Communication Layer
    subgraph Communication Layer
        COMM_TCP((TCP Socket <br> Port 30003/29999)):::commLayer
        COMM_UDP((UDP Socket <br> Port 30004)):::commLayer
    end

    %% 4. Robot Layer
    subgraph Robot Layer
        ROB_CTRL[MG400 Robot Controller]:::robotLayer
        ROB_Q[Firmware Command Queue]:::robotLayer
        ROB_EXEC[Motion Execution]:::robotLayer
        ROB_MOT((Robot Joint Movement)):::robotLayer
    end

    %% 5. Feedback Layer
    subgraph Feedback Layer
        FB_ENC[Robot Joint Encoders]:::feedbackLayer
        FB_TOPIC[[Topic: /joint_states]]:::feedbackLayer
    end

    %% 6. Visualization & Monitoring
    subgraph Visualization Tools
        VIZ_FOX[Foxglove Studio <br> Data Flow & Plotting]:::vizLayer
        VIZ_RQT[rqt_graph <br> Node Topology]:::vizLayer
        VIZ_TRC[ros2_tracing <br> Latency Analysis]:::vizLayer
    end

    %% Flow Connections (Command)
    UI_KB -->|Input Data| ROS_TELEOP
    UI_VR -->|Input Data| ROS_TELEOP
    
    ROS_TELEOP -->|Publish| ROS_TOPIC_CMD
    ROS_TOPIC_CMD -->|Subscribe| ROS_CTRL
    ROS_CTRL -->|Process Target| ROS_DRV
    
    ROS_DRV -->|TCP Command| COMM_TCP
    COMM_TCP -->|Packet Receive| ROB_CTRL
    
    ROB_CTRL -->|Push to Queue| ROB_Q
    ROB_Q -->|Pop & Execute| ROB_EXEC
    ROB_EXEC -->|Motor Actuation| ROB_MOT

    %% Flow Connections (Feedback)
    ROB_MOT -->|Position Read| FB_ENC
    FB_ENC -->|Feedback Packet| COMM_UDP
    COMM_UDP -->|Parse Bytes| ROS_DRV
    ROS_DRV -->|Publish| FB_TOPIC
    
    %% Visualization connections
    ROS_TOPIC_CMD -.->|Inspect| VIZ_FOX
    FB_TOPIC -.->|Realtime Plot| VIZ_FOX
    ROS_TELEOP -.->|Timing| VIZ_TRC
    ROS_DRV -.->|Topology| VIZ_RQT
```

## Latency Analysis Timeline

To debug teleoperation lag, the system latency is broken down into three critical segments:

1. **ROS 2 Processing Latency:** Time from when `Teleop Node` receives the user input to when the `MG400 Driver` is ready to send it over network.
2. **Communication Latency:** Network overhead of transmitting the command via TCP to the actual Robot hardware.
3. **Robot Execution Latency:** Time spent sitting in the hardware firmware queue before the actual physical motion begins.

```mermaid
sequenceDiagram
    participant User as User / Teleop
    participant ROS as ROS 2 (Driver)
    participant Net as Network (TCP)
    participant Q as Robot Queue
    participant Mech as Robot Hardware

    Note over User, ROS: 1. ROS Processing Latency
    User->>ROS: Command Sent (Input)
    activate ROS
    ROS-->>ROS: Process kinematics & predict
    
    Note over ROS, Net: 2. Communication Latency
    ROS->>Net: Driver Send
    deactivate ROS
    activate Net
    Net->>Q: TCP Comm received
    deactivate Net
    
    Note over Q, Mech: 3. Robot Execution Latency
    activate Q
    Q-->>Q: Sitting in firmware queue
    Q->>Mech: Queue Pop (Execution)
    deactivate Q
    activate Mech
    Mech-->>Mech: Physical Motion
    Mech->>ROS: Feedback Published (/joint_states)
    deactivate Mech
```

## Summary for Debugging Lag

- **If ROS 2 Processing is slow:** Check `ros2_tracing` for blocked callbacks, high CPU utilization, or single-threaded executor deadlocks.
- **If Network Communication is slow:** Check for Nagle's algorithm being enabled (`TCP_NODELAY` off), packet loss, or high latency over Wi-Fi vs Ethernet.
- **If Robot Execution is slow:** Check if the Queue depth is too large (stale commands piling up). Implement dynamic dropping (gatekeeper) before sending commands to bound the queue size.
