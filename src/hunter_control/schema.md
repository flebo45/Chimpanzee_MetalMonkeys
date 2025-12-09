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
%% Sequence = ->
ROOT("<b>-></b><br/>Hunter_Brain"):::seq --> DATA_LAYER("ACTION<br/><b>Update_BB</b>"):::data
ROOT --> PRIORITIES("<b>?</b><br/>Priorities"):::sel

%% --- RAMO 1: SAFETY ---
PRIORITIES --> SAFETY_SEQ("<b>-></b><br/>Safety"):::seq
SAFETY_SEQ --> COND_OBS("CONDITION<br/><b>Obstacle?</b>"):::cond
SAFETY_SEQ --> ACT_EVADE("ACTION<br/><b>Evasive_Maneuver</b>"):::act

%% --- RAMO 2: REAL TRACKING ---
PRIORITIES --> REAL_SEQ("<b>-></b><br/>Real_Tracking"):::seq
REAL_SEQ --> COND_REAL("CONDITION<br/><b>Ball_Detected?</b>"):::cond
REAL_SEQ --> ACT_CHASE("ACTION<br/><b>Chase_Ball</b>"):::act

%% --- RAMO 3: RECOVERY ---
PRIORITIES --> REC_SEQ("<b>-></b><br/>Proximity_Recovery"):::seq
REC_SEQ --> COND_CLOSE("CONDITION<br/><b>Was_Ball_Close?</b>"):::cond
REC_SEQ --> ACT_BACKUP("ACTION<br/><b>Blind_Backup</b>"):::act

%% --- RAMO 4: GHOST TRACKING ---
PRIORITIES --> GHOST_SEQ("<b>-></b><br/>Ghost_Tracking"):::seq
GHOST_SEQ --> COND_PRED("CONDITION<br/><b>Prediction_Active?</b>"):::cond
GHOST_SEQ --> ACT_GHOST("ACTION<br/><b>Follow_Ghost</b>"):::act

%% --- RAMO 5: SEARCH ---
PRIORITIES --> ACT_SEARCH("ACTION<br/><b>Search_Spin</b>"):::act

 ```