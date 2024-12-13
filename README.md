# RSP Fuzzer and Visualizer

A containerized system for testing and visualizing the behavior of the `robot_state_publisher` ROS package in isolation. This system generates test motion patterns to feed into the `robot_state_publisher` and visualizes the resulting joint states and transform data in real-time.

## Containers

- **Robot State Publisher**: [`robot_state_publisher`](https://wiki.ros.org/robot_state_publisher) uses the URDF specified by the parameter robot_description and the joint positions from the topic joint_states to calculate the forward kinematics of the robot and publish the results via [`tf`](https://wiki.ros.org/tf)
- **Fuzzer**: Generates test motion patterns to feed into the robot state publisher
- **Recorder**: Captures joint states and transform data from the `robot_state_publisher` and stores it in `JSON` format.
- **Visualizer**: Web-based real-time visualization dashboard

## Architecture

### High-Level Architecture

This architecture diagram shows the intended functionality of the system. Note that, the low level data flow between the components is abstracted in this diagram.

```mermaid
%%{init: {
  'theme': 'base',
  'themeVariables': {
    'primaryColor': '#e3f2fd',
    'secondaryColor': '#f3f0ff',
    'tertiaryColor': '#fff5f5',
    'primaryBorderColor': '#1e88e5',
    'lineColor': '#424242'
  },
  'flowchart': {
    'curve': 'basis',
    'nodeSpacing': 50,
    'rankSpacing': 50
  }
}}%%

graph LR
    subgraph Core["Core System"]
        F["Fuzzer Node<br/>(Input Generator)"]-->|"JointState"|R["Robot State<br/>Publisher"]
        R-->|"JointState +<br/>TF Messages"|V["WebSocket<br/>Server"]
        R-->|"JointState +<br/>TF Messages"|C["Data Logger"]
    end

    subgraph Config["Configuration"]
        CF["Config<br/>Files"]-.->F
        UR["URDF<br/>Files"]-->|"Robot Description"|R
    end

    subgraph Web["Interface"]
        V-->|"WebSocket"|W["Web UI"]
    end

    C-->|"JSON Export"|D[(Data<br/>Directory)]

    %% Style Definitions
    classDef default fill:#e3f2fd,stroke:#1e88e5,stroke-width:1.5px,color:#1565c0
    classDef storage fill:#f3f0ff,stroke:#7c4dff,stroke-width:1.5px,color:#4527a0
    classDef visual fill:#fff5f5,stroke:#FF9800,stroke-width:1.5px,color:#FF9800
    classDef record fill:#f1f8e9,stroke:#7cb342,stroke-width:1.5px,color:#33691e

    %% Apply Styles
    class F,R default
    class CF,UR storage
    class V,W visual
    class C,D record

    %% Style subgraphs
    style Core fill:#e3f2fd,stroke:#1e88e5,stroke-width:2px
    style Config fill:#f3f0ff,stroke:#7c4dff,stroke-width:2px
    style Web fill:#fff5f5,stroke:#FF9800,stroke-width:2px
```

### Detailed Architecture

Even though the sysem is composed on four containers, the following diagram shows ther abstracted low level data flow between the components.

```mermaid
%%{
  init: {
    'theme': 'base',
    'themeVariables': {
      'fontFamily': 'Helvetica',
      'fontSize': '14px',
      'primaryColor': '#e3f2fd',
      'secondaryColor': '#f3f0ff',
      'tertiaryColor': '#fff5f5',
      'primaryBorderColor': '#1e88e5',
      'lineColor': '#424242',
      'textColor': '#212529'
    },
    'flowchart': {
      'htmlLabels': true,
      'curve': 'basis',
      'nodeSpacing': 50,
      'rankSpacing': 50
    }
  }
}%%

graph TB
    %% Node styles
    classDef default fill:#e3f2fd,stroke:#1e88e5,stroke-width:1.5px,color:#1565c0,font-family:Helvetica
    classDef storage fill:#f3f0ff,stroke:#7c4dff,stroke-width:1.5px,color:#4527a0
    classDef visual fill:#fff5f5,stroke:#FF9800,stroke-width:1.5px,color:#FF9800
    classDef record fill:#f1f8e9,stroke:#7cb342,stroke-width:1.5px,color:#33691e
    
    subgraph DataStorage["Data Storage"]
        direction TB
        DS[Data Directory]:::storage
        URDF[URDF Files]:::storage
        Config[Config Files]:::storage
    end

    subgraph ROS2["ROS2 Domain"]
        direction TB
        RSP[Robot State Publisher]
        FUZ[Fuzzer Node]
        FUZ -->|JointState| RSP
    end

    subgraph VIS["Visualizer"]
        direction TB
        WebServer[WebSocket Server]:::visual
        webUI[Web UI]:::visual
        WebServer -->|WebSocket| webUI
    end

    subgraph REC["Recorder"]
        direction TB
        DataLogger[Data Logger]:::record
    end
    
    %% External connections
    URDF -->|Robot Description| RSP
    Config -->|Parameters| FUZ
    FUZ -->|JointState| WebServer
    RSP -->|TF Messages| WebServer
    RSP -->|TF Messages| DataLogger
    FUZ -->|JointState| DataLogger
    DataLogger -->|JSON Export| DS

    %% Style all subgraphs
    style DataStorage fill:#f3f0ff,stroke:#7c4dff,stroke-width:2px
    style ROS2 fill:#e3f2fd,stroke:#1e88e5,stroke-width:2px
    style VIS fill:#fff5f5,stroke:#FF9800,stroke-width:2px
    style REC fill:#f1f8e9,stroke:#7cb342,stroke-width:2px
```

## Requirements

- Docker
- Docker Compose

## Quick Start

1. Clone the repository

2. Create a `data` directory for storing results:

    ```sh
    mkdir data
    ```

3. Start the system:

    ```sh
    docker-compose up
    ```

4. Open the visualizer in your browser:

    ```text
    http://localhost:8080
    ```

    The visualizer should display graphs containing the robot's joint states and transform data as follows:

    ![Visualizer Screenshot](docs/images/web-console.png)

5. Recorded data will be stored in the `data` directory. `data/results` directory will contain json files with the recorded data. File names starting with `joint_states` contain joint states data, and file names starting with `transforms` contain transform data. The files are suffixed with the timestamp when the recording started.
