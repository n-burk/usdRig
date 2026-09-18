# Esf/Exec compatibility qualification

`probeEsfCompatibility` qualifies the non-public request and change-delivery
contacts required by spec section 11.2. It is an explicit CMake target, excluded
from the normal build because a missing platform export is a qualification
failure rather than a reason to modify OpenUSD.

```text
cmake --build build --target probeEsfCompatibility
build\probeEsfCompatibility.exe
```

On Windows, run from the configured MSVC environment with the OpenUSD `bin`
and `lib` directories on `PATH`.

The September 5, 2026 Windows x64 shared-library run passed against the installed
OpenUSD 26.08 package. Its local source checkout is
`4b753359d308d7d0416dcee464e1c7d130bc39dc`. No OpenUSD source, visibility setting,
or exported symbol was changed.

The probe derives directly from `ExecSystem` and `Exec_RequestImpl`. It verifies:

- Generic system construction and request registration/tracker iteration.
- Request compilation, scheduling, ordinary computation and extraction.
- Repeated authored-value invalidation and renewed request interest.
- Temporary value overrides and the lifetime of their cache view.
- Numeric and PreTime evaluation identities and time notifications.
- Resync, incoming-connection notification, re-preparation, index expiration,
  and request removal outside tracker locks.
- A clean USD diagnostic error mark; coding errors fail the probe.

Fixture storage uses the shipped EsfUsd adapter with its required stage-data
lifetime. No `ExecUsdSystem` performs the tested requests or forwards their
changes. This proves that the tested generic contacts compile, link, load, and
execute in this stock package. It does **not** qualify a custom Esf scene
database, pack loader, full backend parity, or another operating system. Those
are qualified separately: see the implemented
[provider runtime](standalone-runtime.md) and [rigpack slice](standalone-pack.md)
for their supported boundary and regression coverage. Full backend and platform
qualification remain separate work under section 11.
