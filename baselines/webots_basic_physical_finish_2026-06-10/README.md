# Basic-World Physical Completion Baseline

This note records an early controller that physically completed one circuit of the official Webots `basic` world on 10 June 2026.

- `params.json` records the matching perception, estimation, and control parameters.
- Telemetry crossed the start region again at approximately `t=288.187 s` with no major collision.
- The SDK metadata at the time did not count the lap because its configured checkpoint coordinates did not match the world; the evidence is therefore a physical trajectory observation rather than an official lap record.
- Generated-controller SHA256: `0c7c74b0c390ceadbe2938c2c475308f9441048c8252c57ea1bcd5d97688b0d7`.

The evidence is useful for historical comparison; current validation should use the SDK workflow documented in `docs/official_testing.md`. The generated snapshot remains recoverable from Git history but is not duplicated in the public source tree.
