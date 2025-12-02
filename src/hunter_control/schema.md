```mermaid 
graph TD
    RootSeq("-> <br/> (Main Sequence)") --> DataGathering("|| <br/> (Data Gathering)")
    RootSeq --> LogicSelector("? <br/> (Logic Root)")

    %% RAMO 1: SENSE (Acquisizione Dati)
    %% Eseguiti in parallelo per aggiornare la lavagna
    DataGathering --> Lidar2BB("[ToBlackboard: <br/> Write 'obstacle_dist']")
    DataGathering --> VisionVis2BB("[ToBlackboard: <br/> Write 'target_visible']")
    DataGathering --> VisionPos2BB("[ToBlackboard: <br/> Write 'target_pose']")

    %% RAMO 2: PLAN & ACT (Logica)
    LogicSelector --> SafetySeq("-> <br/> (Safety)")
    LogicSelector --> ChaseSeq("-> <br/> (Chase)")
    LogicSelector --> SearchAction("[Action: Spin / Search]")

    %% Dettaglio Safety
    SafetySeq --> CheckObstacle("(Condition) <br/> Obstacle < 0.6m?")
    SafetySeq --> StopAction("[Action: Emergency Stop]")

    %% Dettaglio Chase
    ChaseSeq --> CheckTarget("(Condition) <br/> Target Visible?")
    ChaseSeq --> TrackAction("[Action: Follow Target]")

    classDef sequence fill:#f9f,stroke:#333,stroke-width:2px,color:#000;
    classDef selector fill:#f9f,stroke:#333,stroke-width:2px,color:#000;
    classDef parallel fill:#ccf,stroke:#333,stroke-width:2px,color:#000;
    classDef input fill:#ddd,stroke:#333,stroke-width:2px,color:#000;
    classDef condition fill:#ff9,stroke:#333,stroke-width:2px,color:#000;
    classDef action fill:#9f9,stroke:#333,stroke-width:2px,color:#000;

    class RootSeq,SafetySeq,ChaseSeq sequence;
    class LogicSelector selector;
    class DataGathering parallel;
    class Lidar2BB,VisionVis2BB,VisionPos2BB input;
    class CheckObstacle,CheckTarget condition;
    class StopAction,TrackAction,SearchAction action;
 ```