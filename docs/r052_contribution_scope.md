# Team Context and Contribution

AI Racer was developed by a five-person team. The repository preserves the integrated controller as well as the experiment history needed to understand how it evolved.

## Primary contribution

My work focused on the multi-car control path and its integration with the shared controller:

- temporal road-centre tracking across frames;
- obstacle-aware steering and speed control;
- stateful multi-car recovery and controller-simulator contract debugging;
- integration across perception, state-estimation and policy modules;
- multi-car launch procedures, regression tests, and Webots diagnosis.

The corresponding development sequence is recorded from `41201d1` through milestone commit `9fc9d89` (R052). Later integration work improved profile isolation, recovery behaviour, tests, and public documentation across the team.

## Repository structure

- `controller/` is the maintainable modular source.
- `submissions/` contains generated single-file platform artifacts.
- `tests/` covers deterministic module and build contracts.
- `experiments/` records simulator evidence and design conclusions.

Commit history remains available for reviewing the exact boundary between the R052 contribution and subsequent team integration.

