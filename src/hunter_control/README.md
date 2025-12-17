# 🧠 Hunter Control

**Hunter Control** is the decision-making core of the Hunter Robot. It implements a reactive **Behavior Tree (BT)** using the `py_trees` library to orchestrate the robot's autonomous behaviors, ranging from safety maneuvers to active target pursuit.

## 📋 Overview

This package replaces traditional nested state machines with a modular Behavior Tree. This approach allows for:
- **Modularity**: Behaviors (Actions/Conditions) are reusable classes.
- **Reactivity**: The tree is ticked at 10Hz, allowing the robot to switch tasks instantly (e.g., aborting a chase to avoid an obstacle).
- **Maintainability**: The logic is hierarchical and easier to visualize.

The control loop follows a **Sense-Plan-Act** paradigm:
1.  **Sense**: The `ToBlackboard` node ingests ROS 2 topics (`/scan`, `/vision/target`, etc.) and writes them to the **Blackboard** (Shared Memory).
2.  **Plan**: The Behavior Tree evaluates **Conditions** (Leaf Nodes) against the Blackboard state to select the appropriate branch.
3.  **Act**: The active **Action** node executes its logic (e.g., PID control) and publishes to `/cmd_vel`.

---

## 🌳 Behavior Tree Structure

The tree is organized by **Priority** (Selector). The robot always attempts to execute the highest-priority valid branch.

1.  **🔋 Battery Check** *(Highest Priority)*
    - **Condition**: Is battery < 20%?
    - **Action**: `EmergencyStop` (Halt all movement).
2.  **🛡️ Safety Layer**
    - **Condition**: Is an obstacle detected within 0.6m?
    - **Action**: `EvasiveManeuver` (Stop -> Back up -> Turn 180°).
3.  **🎯 Real Tracking**
    - **Condition**: Is the Red Ball visible and NOT a prediction?
    - **Action**: `TrackReal` (Dual-PID pursuit: Turn-then-Move).
4.  **🔄 Proximity Recovery**
    - **Condition**: Was the target just seen but now lost (likely too close/blind spot)?
    - **Action**: `BlindBackUp` (Reverse blindly to regain visual contact).
5.  **👻 Ghost Tracking**
    - **Condition**: Is the Kalman Filter predicting the target path?
    - **Action**: `TrackGhost` (Rotate to face the predicted location).
6.  **🔍 Search** *(Lowest Priority)*
    - **Action**: `SpinSearch` (Rotate in place to scan the environment).

```mermaid 
graph TD

%% --- STILI ---
%% Ho aggiunto 'color:black' per forzare il testo nero
classDef seq fill:#e1f5fe,stroke:#01579b,stroke-width:2px,color:black;
classDef sel fill:#e8f5e9,stroke:#2e7d32,stroke-width:2px,color:black;
classDef act fill:#fff9c4,stroke:#fbc02d,stroke-width:2px,color:black;
classDef cond fill:#fce4ec,stroke:#c2185b,stroke-width:2px,stroke-dasharray: 5 5,color:black;
classDef data fill:#e0e0e0,stroke:#616161,stroke-width:2px,color:black;

%% Stile specifico per la legenda (più piccolo)
classDef legend font-size:10pt,fill:#fff,stroke:none,color:black;

%% --- ROOT ---
ROOT("<b>-></b>



Hunter_Brain"):::seq --> DATA_LAYER("ACTION



<b>Update_BB</b>"):::data
ROOT --> BRAIN_SEL("<b>?</b>



Brain_Selector"):::sel

%% --- RAMO 0: BATTERY CHECK (Priorità Assoluta) ---
BRAIN_SEL --> BATTERY_SEQ("<b>-></b>



Battery_Check"):::seq
BATTERY_SEQ --> COND_BAT("CONDITION



<b>Battery_Low?</b>"):::cond
BATTERY_SEQ --> ACT_EMG_STOP("ACTION



<b>Emergency_Stop</b>"):::act

%% --- RAMO PRIORITÀ OPERATIVE ---
BRAIN_SEL --> PRIORITIES("<b>?</b>



Priorities"):::sel

%% --- RAMO 1: SAFETY ---
PRIORITIES --> SAFETY_SEQ("<b>-></b>



Safety"):::seq
SAFETY_SEQ --> COND_OBS("CONDITION



<b>Obstacle?</b>"):::cond
SAFETY_SEQ --> ACT_EVADE("ACTION



<b>Evasive_Maneuver</b>"):::act

%% --- RAMO 2: REAL TRACKING ---
PRIORITIES --> REAL_SEQ("<b>-></b>



Real_Tracking"):::seq
REAL_SEQ --> COND_REAL("CONDITION



<b>Ball_Detected?</b>"):::cond
REAL_SEQ --> ACT_CHASE("ACTION



<b>Chase_Ball</b>"):::act

%% --- RAMO 3: RECOVERY ---
PRIORITIES --> REC_SEQ("<b>-></b>



Proximity_Recovery"):::seq
REC_SEQ --> COND_CLOSE("CONDITION



<b>Was_Ball_Close?</b>"):::cond
REC_SEQ --> ACT_BACKUP("ACTION



<b>Blind_Backup</b>"):::act

%% --- RAMO 4: GHOST TRACKING ---
PRIORITIES --> GHOST_SEQ("<b>-></b>



Ghost_Tracking"):::seq
GHOST_SEQ --> COND_PRED("CONDITION



<b>Prediction_Active?</b>"):::cond
GHOST_SEQ --> ACT_GHOST("ACTION



<b>Follow_Ghost</b>"):::act

%% --- RAMO 5: SEARCH ---
PRIORITIES --> ACT_SEARCH("ACTION



<b>Search_Spin</b>"):::act

 ```
```mermaid
%%{init: {
  'theme': 'base',
  'flowchart': { 'curve': 'basis', 'nodeSpacing': 40, 'rankSpacing': 80 }, 
  'themeVariables': { 'primaryColor': '#ffffff', 'edgeLabelBackground':'#ffffff', 'tertiaryColor': '#ffffff'}
}}%%
flowchart LR
    %% --- STILI ---
    classDef topic fill:#E8F5E9,stroke:#2e7d32,stroke-width:1.5px,color:#000000;
    classDef input fill:#FFFFFF,stroke:#33691e,stroke-width:2px,color:#000000;
    classDef logic fill:#FFFFFF,stroke:#bf360c,stroke-width:2px,color:#000000;
    classDef memory fill:#f3e5f5,stroke:#7b1fa2,stroke-width:3px,color:#000000;

    %% === SFONDO ===
    subgraph FoglioBianco [ ]
        direction LR

        %% --- 1. INPUT (SCRITTORI) ---
        subgraph Writers [Ingressi ROS]
            direction TB
            ScanTopic[/`/scan`<br/>/]:::topic
            VisibleTopic[/`/vision/is_visible`/]:::topic
            TargetTopic[/`/vision/target`/]:::topic
            ToBlackboard["topicsToBB.py"]:::input
        end
        style Writers fill:#f1f8e9,stroke:#33691e,stroke-width:1px,color:#000000

        %% --- 2. MEMORIA ---
        %% Aggiunte le nuove variabili chiave gestite
        Blackboard[("🧠<br/>BLACKBOARD<br/><br/><i>- obstacle_distance<br/>- target_visible<br/>- target_error_x<br/>- target_area<br/>- is_prediction<br/>- last_valid_area<br/>- is_occluded</i>")]:::memory

        %% --- 3. LOGICA (LETTORI) ---
        subgraph Readers [LOGIC & ACTIONS]
            direction TB
            
            %% Condizioni
            subgraph Conditions [Conditions]
                direction TB
                CondObs{IsObstacleClose?}:::logic
                CondReal{IsTargetReal?}:::logic
                CondPred{IsTargetPredicted?}:::logic
                CondClose{WasTargetClose?}:::logic
            end

            %% Azioni
            subgraph Actions [Actions]
                direction TB
                ActStop[EmergencyStop]:::logic
                ActEvade[EvasiveManeuver]:::logic
                ActTrack[TrackReal]:::logic
                ActGhost[TrackGhost]:::logic
                ActRecover[BlindBackUp]:::logic
                ActSearch[SpinSearch]:::logic
            end
        end
        style Readers fill:#fff3e0,stroke:#e65100,stroke-width:1px,color:#000000
        style Conditions fill:none,stroke:none
        style Actions fill:none,stroke:none

        %% --- CONNESSIONI DI SCRITTURA ---
        ScanTopic --> ToBlackboard
        VisibleTopic --> ToBlackboard
        TargetTopic --> ToBlackboard

        ToBlackboard ==>|Aggiorna Chiavi| Blackboard

        %% --- CONNESSIONI DI LETTURA ---
        %% Le frecce indicano quali dati attivano quali comportamenti
        Blackboard -.->|obstacle_distance| CondObs
        Blackboard -.->|target_visible<br/>is_prediction| CondReal
        Blackboard -.->|is_prediction| CondPred
        Blackboard -.->|last_valid_area| CondClose
        
        %% Collegamenti logici (semplificati per il diagramma)
        CondObs --> ActStop
        CondObs --> ActEvade
        CondReal --> ActTrack
        CondPred --> ActGhost
        CondClose --> ActRecover
        Blackboard -.->|Fallback| ActSearch

    end
    
    style FoglioBianco fill:#FFFFFF,stroke:none,color:#000000

    %% --- STILI FRECCE ---
    linkStyle 0,1,2 stroke-width:3px,stroke:#33691e,fill:none;
    linkStyle 3 stroke-width:3px,stroke:#43a047,fill:none;
    linkStyle 4,5,6,7,13 stroke-width:2px,stroke:#7b1fa2,fill:none,stroke-dasharray: 5 5;
    linkStyle 8,9,10,11,12 stroke-width:1px,stroke:#bf360c,fill:none;
 ```

---

## 🧩 Nodes & Scripts

### 1. `control_node` (Main Brain)
- **Entry Point**: `bt_main.py`
- **Function**: Initializes the ROS node, builds the tree defined in `create_root()`, and ticks it at 10Hz.
- **Subscribes**: `/scan`, `/vision/target`, `/vision/is_visible`, `/battery/status`.
- **Publishes**: `/cmd_vel`.

### 2. `battery_node` (Simulation)
- **Entry Point**: `simulations/battery_node.py`
- **Function**: Simulates battery discharge based on motor usage.
- **Logic**: `SoC = SoC - (Idle_Drain + |Velocity| * Load_Factor)`.
- **Publishes**: `/battery/status`.

### 3. `ball_teleop` (Testing Tool)
- **Entry Point**: `ball_teleop.py`
- **Function**: Allows a human operator to drive the Red Ball in Gazebo to stress-test the robot's tracking.
- **Controls**:
    - **W/S**: Continuous linear movement (Deadman switch).
    - **A/D**: Discrete 90° turns (Press once).

### 4. `metrics_logger` (Analysis Tool)
- **Entry Point**: `analysis/metrics_logger.py`
- **Function**: Records telemetry data during a test session and generates a performance report (CSV + PNG plots) upon exit (CTRL+C).
- **Metrics**:
    - **RMSE**: Root Mean Square Error of the tracking (pixels).
    - **Response**: Correlates visual error with actuator commands.
- **Subscribes**: `/vision/target`, `/cmd_vel`.
- **Output**: `test_report_YYYYMMDD_HHMMSS.png`, `test_data_YYYYMMDD_HHMMSS.csv`.

---

## 🕹️ Usage

### Run the Control Brain
Usually launched via `hunter_bringup`, but can be run standalone:
```bash
ros2 run hunter_control control_node
```

### Run the Ball Teleop
To manually move the target:
```bash
ros2 run hunter_control ball_teleop
```

### Run the Battery Simulator
Usually launched via `hunter_bringup`, but can be run standalone:
```bash
ros2 run hunter_control battery_node
```

### Run the Metrics Logger
Start this before your test run, then stop it with CTRL+C to generate the report:
```bash
ros2 run hunter_control metrics_logger
```

---

## 📂 Code Structure

```plaintext
hunter_control/
├── hunter_control/
│   ├── analysis/
│   │   └── metrics_logger.py # Performance analysis tool
│   ├── behaviors/
│   │   ├── actions.py      # Action Nodes (Track, Search, Evade...)
│   │   ├── conditions.py   # Condition Nodes (IsObstacle?, IsVisible?...)
│   │   └── topicsToBB.py   # ROS->Blackboard Data Ingestion
│   ├── simulations/
│   │   └── battery_node.py # Battery logic
│   ├── ball_teleop.py      # Teleop script
│   └── bt_main.py          # Tree definition and Node entry point
├── package.xml
└── setup.py
```
