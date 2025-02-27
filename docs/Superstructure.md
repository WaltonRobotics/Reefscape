# Superstructure

## Subsystems

-   **Coral**
    - 1 TalonFX: connected to wheels that move the CORAL through the subsystem
    - 2 DigitalInputs: one sensor at the top of the subsystem that checks when a CORAL enters the system and one sensor at the bottom ofthe subsystem that checks when a CORAL is fully intook and when it exits the system
-   **Elevator**
    -   2 Krakens: moves elevator up and down

## States

### State Diagram

This diagram is very basic, and I'm sleepy. NOTE: there are technically different states for each height in ELE_TO_SCORE

```mermaid
stateDiagram
    state "IDLE" as s1
    state "ELE_TO_INTAKE" as s2
    state "INTAKING" as s3
    state "SLOW-INTAKE" as s4
    state "INTOOK" as s5
    state "ELE_TO_SCORE" as s6
    state "SCORE_READY" as s6
    state "SCORING" as s7
    state "SCORED" as s8
    state "ELE_TO_CLIMB" as s9
    state "CLIMB_READY" as s10
    state "CLIMBING" as s11
    state "CLIMBED" as s12

    s1 --> s2: IntakeReq->T
    s1 --> s4: BotSensor->T
    s2 --> s3: EleAtIntake->T
    s3 --> s4: TopSensor->T
    s4 --> s5: BotSensor->T
    s5 --> s6: ScoreEleReq->Ta
    s6 --> s7: EleAtScore->T
    s7 --> s8: ScoreReq->T
    s8 --> s9: BotSensor->F & ScoreReq->F
    s9 --> s2: Automatic
    s9 --> s1: ToHomeReq->T
    s1 --> s9: ClimbUpReq->T
    s9 --> s10: EleAtClimb->T
    s10 --> s11: ClimbDownReq->T
    s11 --> s12: EleAtHome->T

```

### Output Truth Table

|    **State**     | **Coral** |**Elevator**   | **Open Requests**  |
| :--------------: | :-------: | :-----------: | :----------------: |
|     **IDLE**     | Unrunning |    HOME       | IntakeReq, ClimbReq|
|**ELE_TO_INTAKE** | Unrunning |Move->INTAKE   | n/a                |
|  **INTAKING**    | Running   | INTAKE        | n/a                |
|  **SLOW-INTAKE** |Slowrunning| INTAKE        | n/a                |
| **INTOOK**       | Unrunning | INTAKE        | ScoreEleReq        |
|  **ELE_TO_SCORE**| Unrunning |Move->SCORE    | n/a                |
| **SCORE_READY**  | Unrunning | SCORE         | ScoreReq           |
| **SCORING**      | Running   |  SCORE        | n/a                |
| **SCORED**       | Unrunning | SCORE         | n/a                |
| **ELE_TO_CLIMB** | Unrunning |Move->CLIMBUP  | n/a                |
| **CLIMB_READY**  | Unrunning | CLIMBUP       | ClimbDownReq       |
| **CLIMBING**     | Unrunning |Move->CLIMBDOWN| n/a                |
| **CLIMBED**      | Unrunning | HOME          | n/a                |     