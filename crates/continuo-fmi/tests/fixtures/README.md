# Reference FMUs

Test fixtures for the FMU importer: FMI 3.0 Co-Simulation FMUs from the
Modelica Association's [Reference-FMUs][repo], which is the FMI project's own
set of models for exercising an implementation.

- **Version**: v0.0.40, released 2026-07-10, from the `Reference-FMUs.zip`
  release asset's `3.0/` directory.
- **License**: BSD-2-Clause, reproduced in full below.
- **Upstream**: <https://github.com/modelica/Reference-FMUs>

## Why these are checked in

They are somebody else's artifacts, which is the point. Our own FMU is built
by `fmi-export`, from the same upstream workspace as the importer, so testing
only against it would be close to circular: a shared misreading of the
standard would agree with itself and pass.

There is a ready-made crate for fetching them instead,
[`fmi-test-data`][test-data], from the same `rust-fmi` workspace as the
`fmi` crate this one imports through, and it downloads this very archive
from GitHub. It is not used, for three reasons in increasing order of how
much they matter.

A test that reaches the network fails on a flight, in a firewalled runner,
and the day upstream reorganizes its releases. Caching the download would
cover the warm path only, since a cache entry unused for a week is evicted
and a first run has nothing to hit.

That crate pins the version rather than taking one: `REF_FMU_VERSION` is a
hardcoded constant, currently a release behind these files, and it builds
an archive name that the current release no longer uses. Which reference
FMUs this repository tests against would become a property of that crate's
release cadence.

And it depends on `fmi` 0.5 while this crate uses 0.8. Those are
semver-incompatible, so both would be built, including a second `fmi-sys`
and therefore a second bindgen run on every CI agent. The fixtures would
also reach the tests through a different importer version than the one
under test. That costs far more than the megabyte it saves.

Under a megabyte for the four, written once and never edited, so plain git
carries them. Git LFS pays off per revision of a file and these have one,
while costing a setup step on every clone and every CI agent.

Each is built for x86_64 and aarch64 on Windows, Linux and macOS, covering
every agent in the CI matrix.

## What each one is for

- **BouncingBall** is the smoke test: real internal state, a height that
  falls, and a bounce that arrives as a state event. Nothing else is a
  simpler thing to have working first.
- **Feedthrough** carries every FMI 3.0 variable type except Clock, two of
  each: Float32/64, the eight sized integers, Boolean, String and Binary.
  That makes it the type-dispatch fixture, where a value going in has to come
  back out unchanged, and an Int64 above 2^53 has to survive without being
  routed through a float on the way.
- **StateSpace** is the array fixture, and it earns its place three times
  over. Its matrices are sized by structural parameters `m`, `n` and `r`
  rather than by constants, so it exercises configuration mode and dimensions
  resolved from the mapping rather than read from the XML. Its `A` is n by n,
  so a transposed matrix still runs, which is what makes a row-major test
  worth writing. And it declares `hasEventMode`, so it is the obvious fixture
  the day event mode is taken on.
- **Resource** reads `resources/y.txt` during initialization, which is the
  only thing that checks the `resourcePath` handed to
  `fmi3InstantiateCoSimulation`. FMI 3.0 changed that argument from 2.0's
  file URI to a plain path, our own FMU ships no resources, and real vendor
  FMUs routinely do, so without this the first FMU that needs its own files
  would be the one that finds the bug. It fails legibly too, with `y` simply
  wrong.

Dahlquist and VanDerPol are deliberately absent, being smooth ODEs with no
events, arrays or resources, so the adapter would treat them exactly as it
treats BouncingBall. Stair and Clocks wait for event mode, which the adapter
switches off.

## License

Reproduced verbatim from `LICENSE.txt` at tag v0.0.40.

```
Copyright (c) 2026, Modelica Association Project "FMI".
All rights reserved.

The Reference FMUs are released under the 2-Clause BSD license:

--------------------------------------------------------------------------------
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIEDi
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--------------------------------------------------------------------------------
```

[repo]: https://github.com/modelica/Reference-FMUs
[test-data]: https://crates.io/crates/fmi-test-data
