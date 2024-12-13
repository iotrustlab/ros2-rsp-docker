# ROS2 Robot Visualizer

A real-time visualization and data recording system for ROS2-based robots, specifically designed for the Turtlebot4 platform.

## Components

- **Robot State Publisher**: Publishes URDF-based robot state
- **Fuzzer**: Generates test motion patterns
- **Recorder**: Captures joint states and transform data
- **Visualizer**: Web-based real-time visualization dashboard

## Architecture

```mermaid
%%{init: {
  'theme': 'base',
  'themeVariables': {
    'primaryColor': '#E3F2FD',
    'primaryTextColor': '#1A237E',
    'primaryBorderColor': '#42A5F5',
    'lineColor': '#78909C',
    'secondaryColor': '#F3E5F5',
    'tertiaryColor': '#E8F5E9',
    'fontSize': '14px',
    'fontFamily': 'Roboto, Arial, sans-serif'
  }
}}%%

graph LR
    subgraph Core["Core System"]
        F["Fuzzer<br/>(Input Generator)"]-->|"Joint States"|R["Robot State<br/>Publisher"]
        R-->|"Joint States +<br/>Transform Data"|V["Visualizer"]
        R-->|"Joint States +<br/>Transform Data"|C["Recorder"]
    end

    subgraph Config["Configuration Layer"]
        CF["Config<br/>Files"]-.->F
        UR["URDF<br/>Models"]-.->R
    end

    subgraph Web["Web Interface Layer"]
        V-->|"WebSocket<br/>Connection"|W["Web UI"]
    end

    C-->|"JSON Data"|D[(Data<br/>Storage)]

    %% Style Definitions
    classDef default fill:#E3F2FD,stroke:#42A5F5,color:#1A237E,font-weight:500
    classDef config fill:#E8F5E9,stroke:#66BB6A,color:#1B5E20,font-weight:500
    classDef web fill:#F3E5F5,stroke:#AB47BC,color:#4A148C,font-weight:500
    classDef storage fill:#FFF3E0,stroke:#FF9800,color:#E65100,font-weight:500

    %% Apply Styles
    class F,R,V,C default
    class CF,UR config
    class W web
    class D storage
```

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
