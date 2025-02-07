# Superstructure

## Subsystems

-   **Coral**
    - 1 TalonFX: connected to wheels that move the CORAL through the subsystem
    - 2 DigitalInputs: one sensor at the top of the subsystem that checks when a CORAL enters the system and one sensor at the bottom ofthe subsystem that checks when a CORAL is fully intook and when it exits the system
-   **Elevator**
    -   2 Krakens: moves elevator up and down

## States

### State Diagram

This diagram is very basic, and I'm sleepy.

```mermaid
stateDiagram
    state "Idle" as s1
    state "Intake Ready" as s2
    state "Intaking" as s3
    state "Intook" as s4
    state "Score Ready" as s5
    state "Scoring" as s6

    s1 --> s2: EleAtIntakeHeight -> TRUE
    s2 --> s3: TopSensor -> TRUE
    s3 --> s4: BotSensor -> TRUE
    s4 --> s5: EleAtScoreHeight -> TRUE
    s5 --> s6: ScoreReq -> TRUE
    s6 --> s1: BotSensor -> FALSE

```

### Output Truth Table

|    **State**     | **Coral** |**Elevator** |
| :--------------: | :-------: | :--------:  |
|     **Idle**     | Unrunning |  Home       |
|**Intake Ready**  | Unrunning |  HP Level   |
|   **Intaking**   | Running   |  HP Level   |
|  **Intook**      | Unrunning |Moving -> Lvl|
|  **Score Ready** | Unrunning | Score Level |
|  **Scoring**     |   Running | Score Level |


## Inputs

### Hardware

#### Digital Inputs

- TopSensor: watching for CORAL entering system
- BotSensor: watching for:
    - CORAL being fully intook
    - CORAL exiting the system

#### Numeric Signals (Motors/Encoders)

-   Elevator At Height
    - it's a MMEV thingy

### Software

#### Control Signals (Human/Autonomous Requests)

- Auton
    - Preload Request: goes straight from Idle -> Intook
        - Only exists during the first scoring iteration in Auton
    - Auton Elevator Requests:
        - Two (HP, Reef)
    Override Requests:
        - Intake Override: goes to Intaking anyway if robot path reaches the HP station without being Intake Ready
        - Score Override: scores anyway if robot has been at the reef for more than a second without being Score Ready
- Teleop (Button Binds)
    - Driver Requests:
        - Elevator Request
        - Score Request
    - Override Requests:
        - Intake Override (intake is usually automatic)
        - Score Override: scores anyway regardless of state