# Falange R00B Omega.3 hardware evidence — initialization and TimeGuard

**Date:** 2026-11-17. **Evidence source:** operator-provided Falange console results and physical observations; the raw JSONL journals are private lab files, not uploaded here or independently inspected. **Qualification remains open.** Do not interpret this report as approval for contact, force control, full R00B movement, or physical stop verification. This is a focused software/API finding.

## Environment

- Hardware: Force Dimension omega.3, physical label matched to SDK-reported individual serial (value retained in private lab evidence).
- Native Force Dimension SDK `3.17.6`, Python `forcedimension_core==1.0.0`.
- Owner `forcedimension-omega-control` installed from verified Git commit `78913dfb941f6eefd348df1e7866fabd97025e55`.
- Falange experiment branch `fix/mcu-e06-lossless-event-batches-20261112`, software-only comparison PR #33 merged at `9feae7fed3c3d6902fce286b2f7dbf138d3e0cc8`.
- Rig is operator-reported free-standing on a tabletop with three-screw PLA fingertip adapter. Independent e-stop, clearance, signed limits and F06 physical approval are outstanding.

## Observed trials

### Identity (2026-11-17): successful DRD-index open

- The supervised `serial --device-index 0` read and displayed the SDK serial, which the operator later confirmed against the physical label. The existing owner uses `drd.openID(index)` and checked `dhd.getSerialNumber(device_id)`.
- No initialize or trajectory was requested. Private identity journal: `$HOME/falange-r00b-private/R00B_IDENTITY_20261117T060139589861Z_1841241.jsonl`.

### First observation: DHD-serial open, DRD initialization failure

- `observe --expected-serial <private>` used `dhd.openSerial(serial)` and verified identity, then called `drd.autoInit(device_id)`.
- SDK reported `drd.autoInit() == -1`. Owner's failure cleanup `drd.close() == -1`. A separate Falange `dhd.stop()` request was SDK-acknowledged, but a subsequent `drd.close() == -1` again.
- **Operator saw no visible motion** during that attempt. No post-init samples. Device/SDK connection was not verifiably closed; the operator later performed a separate DRD comparison, but we have no independent hardware recovery attestations.
- Journal: `$HOME/falange-r00b-private/R00B_OBSERVATION_20261117T061440601157Z_1860786.jsonl`.

### Second observation: DRD-index open, initialization behavior successful, position read falsely rejected

- `observe --device-index 0 --expected-serial <private>` used `drd.openID(index)`, checked individual serial before separate moving-initialization authorization, and called the pinned owner's checked `initialize()`.
- Operator **saw the robot physically move and the LED become steady**, indicating initialization to the operator. The program advanced past the `initialize()` call, past its checked DRD tracking-parameter readback, into the state observation loop.
- **SDK tracking parameters reported** `vmax=1.0 m/s, amax=1.0 m/s², jerk=1.0 m/s³`; informational only, **not approved motion/safety limits**.
- First vector read `dhd.getPosition()` returned **status `1`**, which owner `_read_vector_checked()` incorrectly rejects unless status is exactly `0`. Falange printed `RuntimeError: DHD position read failed with status 1`, `0` recorded samples, `FAILED`. No end-of-trial close or BRAKE evidence was supplied in the displayed summary (request the full local event list if necessary).
- Journal: `$HOME/falange-r00b-private/R00B_OBSERVATION_20261117T063058896498Z_1886003.jsonl`.

## Confirmed software defect: DHD_TIMEGUARD = 1 for position is a success

Force Dimension's **vendor SDK** docs explicitly state that `dhdGetPosition()` returns `0` **or `DHD_TIMEGUARD`** on success and `-1` on failure. `DHD_TIMEGUARD = 1` means the TimeGuard cache avoided a redundant device communication. The **Python binding** `forcedimension_core.constants.TIMEGUARD` is also `1`; `dhd.getPosition()` likewise documents `0` or `TIMEGUARD` success. Documentation:

- Vendor: https://downloads.forcedimension.com/sdk/doc/fdsdk-3.14.0/dhd/dhdc_8h.html
- Python DHD: https://forcedimension-core-python-docs.readthedocs.io/en/stable/forcedimension_core.dhd.html
- TimeGuard behavior: https://forcedimension-core-python-docs.readthedocs.io/en/stable/forcedimension_core.dhd-doc.html

Owner code prior to this fix treated every nonzero `dhd.getPosition()` result as failure even with finite returned data. This is the directly observed cause of the **second observation's read failure**, not evidence of an unsuccessful initialization. The vendor Python binding documents `getLinearVelocity()` and `getForce()` as **zero-only** success, so the fix must accept TimeGuard **only for position**. Invalid/non-finite samples and genuine errors remain fail-closed.

**TimeGuard caveat:** a status-1 position can be recently cached, not a freshly acquired hardware sample. Do not equate host receipt time with actual device acquisition or infer a safety-rated measurement freshness guarantee. This revision preserves the original three-vector return type, does not disable TimeGuard, and does not relax monitoring timeouts or operator-authorized limits.

## Open owner / integration questions

1. The **first trial** differed in device selection (`dhd.openSerial` instead of `drd.openID`). Its `autoInit=-1` and failed DRD closes remain **unexplained**. The second trial supports the hypothesis that DRD opening matters, but **does not prove** a DHD/DRD compatibility failure; hardware state/recovery also changed between trials. Do not silently fall back from a requested identity, assume default-device indexing is stable, or erase failure evidence.
2. The **second trial** passed initial physical motion/checked parameter readback but **did not obtain a full five-second vector sequence**. Velocity and force readouts, recorder behavior, physical BRAKE and DRD close still require supervised evidence after an independently reviewed fix.
3. Keep existing safety checks, staged constructor, serial verification and BRAKE/close error reporting. Fix the **documented TimeGuard success status** with device-free tests and independent review, then update Falange's exact owner pin only in a separately reviewed integration change.
4. Confirm local rig workspace, free-standing base stability and independent physical stop before any further potentially moving SDK operation. **F06 remains blocked / not verified**, as do full robot/MCU qualifications.
