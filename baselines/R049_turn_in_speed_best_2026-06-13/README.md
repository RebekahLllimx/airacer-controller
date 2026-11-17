# R049 Corner-Arrival and Speed Baseline

- MD5: `79ffbdbfe1259cc41824123e296bd49b`.
- Scenario: solo `complex` Webots run from `car_1`.
- Observed result: stable corner completion, mean command speed `0.85`, median command speed `0.90`, no lost-road frames, and no high-severity static contact in the structured log.

## Design changes represented by this baseline

1. Corner arrival is derived from near-field lateral displacement rather than distant heading alone.
2. Speed compensation opens the look-ahead gate earlier at higher speed.
3. A latch holds steering authority through the corner and decays after exit.
4. The speed profile raises medium-corner speed without removing sharp-corner limits.

The snapshot is retained as the principal solo-driving rollback point. Traffic avoidance is intentionally outside its scope.
