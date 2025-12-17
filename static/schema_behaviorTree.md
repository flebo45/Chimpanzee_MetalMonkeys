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