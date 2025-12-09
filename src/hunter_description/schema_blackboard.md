```mermaid
%%{init: {
  'theme': 'base',
  'flowchart': { 'curve': 'basis' }, 
  'themeVariables': { 'primaryColor': '#ffffff', 'edgeLabelBackground':'#ffffff', 'tertiaryColor': '#ffffff'}
}}%%
flowchart LR
    %% --- STILI ---
    classDef input fill:#FFFFFF,stroke:#33691e,stroke-width:2px,color:#000000;
    classDef logic fill:#FFFFFF,stroke:#bf360c,stroke-width:2px,color:#000000;
    
    %% Stile speciale per la Blackboard (riempimento lilla chiaro)
    classDef memory fill:#f3e5f5,stroke:#7b1fa2,stroke-width:3px,color:#000000;

    %% === IL TRUCCO DEL FOGLIO BIANCO ===
    subgraph FoglioBianco [ ]
        direction LR

        %% --- 1. INPUT ---
        subgraph Writers [INPUT]
            direction TB
            Lidar2BB(Lidar2BB):::input
            VisPos2BB(VisionPose2BB):::input
            VisStat2BB(VisStatus2BB):::input
        end
        style Writers fill:#f1f8e9,stroke:#33691e,stroke-width:1px,color:#000000

        %% --- 2. MEMORIA ---
        %% Nota: Ho aggiunto <br/> per mandare a capo il testo e ingrandire il cilindro
        Blackboard[("🧠<br/>BLACKBOARD")]:::memory

        %% --- 3. LOGICA ---
        subgraph Readers [LOGIC & ACTIONS]
            direction TB
            CondObs{Ostacolo?}:::logic
            CondVis{Visibile?}:::logic
            ActTrack[ActionTrack]:::logic
            ActStop[ActionStop]:::logic
            ActSearch[ActionSearch]:::logic
        end
        style Readers fill:#fff3e0,stroke:#e65100,stroke-width:1px,color:#000000

        %% --- CONNESSIONI ---
        %% Gruppo 1: SCRITTURA (Linee solide e spesse)
        Lidar2BB ==>|Scrive| Blackboard
        VisPos2BB ==>|Scrive| Blackboard
        VisStat2BB ==>|Scrive| Blackboard

        %% Gruppo 2: LETTURA (Linee tratteggiate e colorate)
        %% Usiamo -.-> per il tratteggio
        Blackboard -.->|Legge| CondObs
        Blackboard -.->|Legge| CondVis
        Blackboard -.->|Legge| ActTrack
        Blackboard -.->|Legge| ActStop
        Blackboard -.->|Legge| ActSearch

    end
    %% Stile del foglio di sfondo
    style FoglioBianco fill:#FFFFFF,stroke:none,color:#000000

    %% --- PERSONALIZZAZIONE FINE DELLE FRECCE (LinkStyle) ---
    %% Link 0,1,2 sono gli INPUT (Neri spessi)
    linkStyle 0,1,2 stroke-width:3px,stroke:#000000,fill:none;
    
    %% Link 3,4,5,6,7 sono gli OUTPUT (Viola scuro, tratteggiati)
    linkStyle 3,4,5,6,7 stroke-width:2px,stroke:#7b1fa2,fill:none,stroke-dasharray: 5 5;
 ```