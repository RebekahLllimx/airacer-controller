# R052 contribution scope and later upstream work

This repository includes the reviewed R052 handoff and later team development. The two should not be treated as one personal contribution history.

## Reviewed R052 line

The R052 work is recorded in the sequence from `41201d1` through `e53f12b`, `f13354c`, `d0b7ab4`, and the handoff commit `9fc9d89`. The reviewed contribution focuses on:

- temporal road-centre tracking across frames;
- obstacle-aware steering and speed control;
- stateful multi-car recovery and controller-simulator contract debugging;
- integration across perception, state-estimation and policy modules;
- multi-car launch procedures, regression tests and Webots diagnosis.

Generated files under `submissions/` are build artifacts. The maintainable source of truth is `controller/`.

## Later team development

Commits after the R052 handoff include additional recovery behavior, profile isolation, report work and team integration. They are part of the repository's current system, but are not automatically personal R052 contributions. Use the commit history and experiment records when attributing a specific mechanism or result.

Useful checks:

```bash
git log --oneline --decorate --graph --all --max-count=40
git diff 9fc9d89..main -- controller tests docs
```

This note documents scope; it does not rewrite commit history or reassign later team work.

