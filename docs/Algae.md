# Algae

## Motors

-   **Wrist**
    - 1 TalonFX: rotates vertically. It's technically a shoulder.
-   **Intake**
    -   1 TalonFX:intakes/scores algae.

## States

### State Diagram

Algae has its own state machine this year cuz its soooooo special like that.

```mermaid
stateDiagram
    state "IDLE" as s1
    state "TO_GROUND" as s2
    state "GROUND" as s3
    state "INTAKING" as s4
    state "INTOOK" as s5
    state "HOME" as s6
    state "TO_PROCESSOR" as s7
    state "PROCESSOR" as s8
    state "SHOOTING" as s9
    state "SHOT" as s10

    s1 --> s2: GroundReq->T
    s2 --> s3: AtGround->T
    s3 --> s4: IntakeReq->T
    s4 --> s5: CurrentSpike->T
    s5 --> s6: AtHome->T
    s6 --> s7: ProcessorReq->T
    s7 --> s8: AtProcessor->T
    s8 --> s9: ShootReq->T
    s9 --> s10: CurrentSpike->F
    s10 --> s1: Automatic

```

### Output Truth Table

|    **State**     | **Intake**     |**Wrist**    | **Open Requests**  |
| :--------------: | :------------: | :--------:  | :----------------: |
|     **IDLE**     | Unrunning      |  HOME       | Ground             |
|**TO_GROUND**     | Unrunning      |Move->GROUND |                    |
|  **GROUND**      | Unrunning      | GROUND      | Intake             |
|  **INTAKING**    | Intaking       | GROUND      |                    |
|  **INTOOK**      | KeepIn         |Move->HOME   |                    |
| **HOME**         | KeepIn         | HOME        | Processor          |
| **TO_PROCESSOR** | KeepIn         |Move->PROC   | n/a                |
| **PROCESSOR**    | KeepIn         |PROC         | Shoot              |
| **SHOOTING**     | Shooting       | PROC        | n/a                |
| **SHOT**         | Unrunning      | Move->HOME  | n/a                |