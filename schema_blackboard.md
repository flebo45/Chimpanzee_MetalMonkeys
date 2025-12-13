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