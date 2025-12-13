```mermaid
%%{init: {
  'theme': 'base',
  'flowchart': { 'curve': 'basis', 'nodeSpacing': 30, 'rankSpacing': 100 },
  'themeVariables': { 'primaryColor': '#ffffff', 'edgeLabelBackground':'#ffffff', 'tertiaryColor': '#ffffff'}
}}%%
flowchart TB
    %% --- STILI ---
    classDef sim fill:#e1f5fe,stroke:#0277bd,stroke-width:2px,color:#000;
    classDef ros fill:#e8f5e9,stroke:#2e7d32,stroke-width:2px,color:#000;
    classDef server fill:#fff3e0,stroke:#ef6c00,stroke-width:2px,stroke-dasharray: 5 5,color:#000;
    
    %% Stile per rendere invisibili i box di raggruppamento (pulizia visiva)
    classDef container fill:none,stroke:none;

    %% === SFONDO BIANCO ===
    subgraph FoglioBianco [ ]
        direction TB

        %% --- BLOCCO 1: SIMULAZIONE (SINISTRA) ---
        subgraph SimGroup [ ]
            direction LR
            Gazebo[("Gazebo Sim")]:::sim
            Bridge["ROS Bridge"]:::sim
        end

        %% --- BLOCCO 2: CERVELLO (CENTRO-ALTO) ---
        subgraph BrainGroup [ ]
            direction TB
            Vision[["vision_node"]]:::ros
            Control[["control_node"]]:::ros
        end

        %% --- BLOCCO 3: TELEMETRIA (BASSO - IL "PAVIMENTO") ---
        %% Mettendolo in un subgraph separato che spingiamo in basso
        subgraph LoggerGroup [ ]
            direction LR
            Telem[["telemetry_node"]]:::ros
            Server[("Web Server")]:::server
        end

        %% === CONNESSIONI ===

        %% 1. SIMULAZIONE INTERNA
        Gazebo <==> Bridge

        %% 2. FLUSSO VISIONE (ALTO)
        Bridge ===>|/camera/image_raw| Vision
        Vision -->|/vision/target, /vision/is_visible| Control

        %% 3. FLUSSO CONTROLLO (CENTRO)
        Bridge -->|/scan| Control
        Control -->|/cmd_vel| Bridge

        %% 4. FLUSSO TELEMETRIA (BASSO)
        Bridge -->|/scan| Telem
        Vision -->|/vision/target| Telem
        Control -->|/cmd_vel| Telem
        
        Telem -.-> Server

        %% === TRUCCHI DI LAYOUT (INVISIBLE LINKS) ===
        %% Questo è il segreto: forza LoggerGroup a stare SOTTO BrainGroup
        %% e forza BrainGroup a stare A DESTRA di SimGroup
        
        Bridge ~~~ Vision
        Vision ~~~ Telem
        Bridge ~~~ Telem

    end
    
    %% Applicazione stili ai container per nascondere i bordi
    class SimGroup,BrainGroup,LoggerGroup container;
    
    style FoglioBianco fill:#FFFFFF,stroke:none,color:#000000

    %% --- STILI FRECCE ---
    %% Camera (Blu, Spessa)
    linkStyle 1 stroke-width:3px,stroke:blue;
    
    %% Cmd_Vel (Rosso) - La freccia che torna indietro
    linkStyle 4 stroke-width:2px,stroke:red;
    
    %% Frecce che vanno alla telemetria (Grigio chiaro per non disturbare)
    linkStyle 5,6,7 stroke-width:1px,stroke:gray;

    %% Server link (Arancio)
    linkStyle 8 stroke-width:2px,stroke:orange,stroke-dasharray: 5 5;
 ```