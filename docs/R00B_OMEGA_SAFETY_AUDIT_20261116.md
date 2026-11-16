# R00B Omega software-safety audit (2026-11-16)

Baseline: `cdde0029b3806343e03ba88ac40e39c5971343ee`. Hardware testing and robot motion were **not** authorized or performed.

## Confirmed defects at baseline

- **O01:** Constructor calls `drd.open()` then `initialize()` without identity verification; initialization can move. `autoInit`, `precisionInit` and `start` results are not checked; cleanup can leave object state stale.
- **O02:** `OmegaRobot.stop()` ignores `dhd.stop()` status. A requested BRAKE is not verified physical braking.
- **O03:** Recorder uses unchecked convenience reads; concurrent SDK operations are not coordinated; recorder exceptions can terminate a daemon thread without supervisor-visible failure. `time.time()` is not calibrated sample time.

## Proposed review sequence

1. [PR #3](https://github.com/event-driven-robotics/forcedimension-omega-control/pull/3): additive staged construction, explicit device selection, checked serial identity before authorization to initialize, checked init results, best-effort BRAKE/close and explicit uncertainty on cleanup failure. Existing default constructor behavior preserved for compatibility. **No safety-rated guarantees.**
2. [PR #4](https://github.com/event-driven-robotics/forcedimension-omega-control/pull/4): checked DHD stop acknowledgment, propagate SDK failures. Distinguish command request, SDK acknowledgment and unverified physical braking.
3. [PR #5](https://github.com/event-driven-robotics/forcedimension-omega-control/pull/5): use checked reads for native recorder, coordinate checked calls using an instance lock, retain thread errors, fail incomplete log saves, preserve HDF5 `data` shape and columns and add timestamp provenance.

Each PR independently targets `main` and remains unmerged pending review. When integrating, review interactions and retest the combined result, especially cleanup/stop locking and the Falange adapter.

## SDK and physical uncertainties

- Validate installed `forcedimension_core` bindings and vendor SDK version against exact DHD/DRD function signatures and return semantics, particularly `dhd.openSerial` followed by DRD operations, enumeration stability, and initialization state.
- Verify all relevant SDK operations are safe under the chosen locking model; external direct SDK calls and legacy convenience methods are not fully coordinated by PR #5.
- Software tests cannot prove physical BRAKE engagement, watchdog latency, robot stop distance, force output, device identity stability across hotplug, or motion-free SDK open behavior.
- The native HDF5 `time` column is **wall-clock-derived elapsed time**; it is neither calibrated MCU time nor host-monotonic sample time. Sampling is not deterministic.
- No physical qualification may be inferred from fake-SDK tests or CI success.

## Qualification preparation gates

Independent code review; passing hardware-free CI on combined changes; inspect the installed SDK API; Falange adapter update to explicitly stage opening, serial verification and initialization; supervised non-contact physical tests for identity mismatch, init failure, BRAKE behavior, recorder failure, concurrent reads/commands and cleanup. Keep Falange dependency pinned until those reviews are complete.
