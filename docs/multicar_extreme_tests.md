# Multi-Car Extreme-Scenario Tests

These tests exercise traffic-specific behavior that ordinary laps may not trigger reliably. They use the `with_other_cars` profile and a controlled Webots arrangement.

## Test principles

- Change one scenario variable at a time.
- Record the controller revision, world, vehicle slots, initial placement, and timeout.
- Keep the ego vehicle's controller unchanged across a comparison.
- Use telemetry, contact logs, and frames together; a single average speed is not enough.
- Treat jump-start scenarios as diagnostic approximations because they do not restore the full physical history of a prior run.

## 1. Stationary blockage

Place an opponent ahead of the ego vehicle with one side visibly more open.

Expected behavior:

1. obstacle detection becomes active;
2. target speed decreases as the obstacle becomes centred and larger;
3. avoidance bias points toward the side with more road margin;
4. the vehicle passes without sustained barrier contact or prolonged oscillation.

Repeat with the open side reversed to verify that the decision depends on observed geometry rather than a fixed steering preference.

## 2. Full-width blockage

Place vehicles so neither side provides enough clearance. This case tests controlled stopping and recovery priority rather than successful passing.

Expected behavior:

- forward speed falls before contact;
- blockage state accumulates only after the visual scene remains stationary;
- recovery runs for a bounded duration;
- normal driving does not immediately cancel the recovery command;
- the controller exits recovery or stops safely if the scene remains infeasible.

## 3. Side impact and displacement

Start an opponent beside the ego vehicle or allow a faster vehicle to contact it laterally. Observe whether temporary road-centre loss causes an uncontrolled steering reversal.

Expected behavior:

- temporal state remains bounded;
- confidence loss reduces speed;
- the estimator resets cleanly after a timestamp restart;
- recovery returns to the visible road without repeated contact.

## 4. Barrier pinning

Place the ego vehicle at an angle against a barrier. This isolates reverse recovery and open-side selection.

Expected behavior:

- contact and low-motion evidence agree on the blockage;
- reverse speed is requested only in the traffic profile and only for the configured interval;
- steering during reverse increases clearance rather than tightening the pin;
- the controller resumes forward motion after the recovery cooldown.

## 5. Overtaking

Use `webots_auto_multicar.sh` with a reduced opponent speed:

```bash
bash scripts/webots_auto_multicar.sh complex 300 30 6 0.55
```

Measure time spent behind the opponent, minimum separation, speed reduction, pass completion, and contact episodes. A pass is not successful if it gains position by accumulating a serious collision.

## Evaluation

For each scenario, report:

- completion or timeout;
- collision episodes filtered to the ego slot;
- maximum blockage duration;
- percentage of frames in avoidance or recovery;
- representative approach, decision, and outcome frames;
- any regression in the solo profile.

The multi-car tests complement, but do not replace, a complete solo and traffic-world run.
