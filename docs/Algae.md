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
    state "INTAKING" as s2
    state "HOME" as s3
    state "TO_PROCESSOR" as s4
    state "PROCESSOR" as s5
    state "SHOOTING" as s6
    state "SHOT" as s7

    s1 --> s2: GroundReq->T
    s2 --> s3: CurrentSpike->T
    s3 --> s4: ProcessorReq->T
    s4 --> s5: AtProcessor->T
    s5 --> s6: ShootReq->T
    s6 --> s7: CurrentSpike->F
    s7 --> s8: Automatic

```

### Output Truth Table

|    **State**     | **Intake**     |**Wrist**    | **Open Requests**  |
| :--------------: | :------------: | :--------:  | :----------------: |
|     **IDLE**     | Unrunning      |  HOME       | Ground             |
|**INTAKING**      | Intaking       |Move->GROUND |                    |
| **HOME**         | KeepIn         | Move->HOME  | Processor          |
| **TO_PROCESSOR** | KeepIn         |Move->PROC   | n/a                |
| **PROCESSOR**    | KeepIn         |PROC         | Shoot              |
| **SHOOTING**     | Shooting       | PROC        | n/a                |
| **SHOT**         | Unrunning      | Move->HOME  | n/a                |