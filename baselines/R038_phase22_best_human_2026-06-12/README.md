# R038 Phase 2.2 Baseline

- Source commit: `ab58b745b48a3cdac2c6c71371c8420823ca4ca4`.
- MD5: `f4b79c09f6811580817ecfe04d1fb11a`.
- SHA256: `7c1ab7106c83781f7f7b4a9487ca831ba0c7721f4a305c12aab58c50e544ce79`.

The R038 controller removed most of the earlier inside-barrier stalls and completed a substantially longer, cleaner run. Visual review still found an overly tight turning radius in some corners, intermittent lane-marking loss, and one recoverable barrier contact. It is retained as the conservative reference preceding the corner-arrival redesign.

The controller source is recoverable from the listed commit. It is not duplicated here because generated snapshots preserve obsolete implementation details and add thousands of lines beside the maintained modular controller.
