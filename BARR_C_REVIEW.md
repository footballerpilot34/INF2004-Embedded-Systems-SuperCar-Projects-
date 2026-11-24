> **Final recursive audit:** see [review/BARR_RULE_MATRIX.md](review/BARR_RULE_MATRIX.md)
> and review/final-summary.json. They supersede the earlier counts and snapshots.

# Measurement reporting update

This section supersedes earlier width-classification and reporting notes.
The scanner has no nominal obstacle dimensions, width acceptance window or
150/250 mm classification. Continuous width is derived from measured edge
geometry. Reliable confirmed geometry is published; insufficient edge evidence
produces UNKNOWN/null. A missing centre echo remains unavailable and does not
borrow the trigger range or a neighbouring ray. Raw samples retain their actual
range and quality, including weak returns and no-echo observations.

The full 31-angle, three-attempt sweep and 200 mm threshold remain. Timeout or
STOP can interrupt a sweep. MQTT and both dashboard APIs publish null for
unavailable numeric measurements. The retired calibration/corrected-width JSON
keys are removed; consumers must use width and edge status. Low-level profile
compatibility slots remain zero and are never published as measurements.

Bypass planning exclusively supplies BYPASS_FIXED_WIDTH_MM (150 mm), the
unchanged effective planning width with the previously unpopulated calibration
table. Nominal depth and movement constants remain planning-only. No scan width
is constrained to this nominal size. The return, post-contact advance, lateral
cap, final correction, state handoff and bounded IR recovery are unchanged.

Limitations: geometric width is an estimate from angular sampling and the
ultrasonic beam, not a calibrated precision measurement. No continuous physical
calibration data exists, so none is fabricated. Unreliable data may now yield
UNKNOWN more often. Strict BARR-C compliance is still not claimed; existing
C11 SDK, naming/type and long-function deviations remain in BARR_C_REVIEW.md.
Numeric Teleplot channels retain explicit invalid sentinels; JSON uses null.

Current verification: review/measurement-tests.txt, measurement-build.txt and
measurement-barr.json. Previous review patches, reports and binary comparisons
are snapshots of the earlier submission refactor, not this updated firmware.
No firmware has been flashed or pushed.

---

# Submission review — BARR-C:2018

This review uses the supplied 2018 Michael Barr PDF, read in full (87 PDF
pages, including appendices). This is **not a full compliance certification**.
The listed deviations are open for project-owner review, not claimed approved.
No copyrighted standard text is reproduced here.

## Scope and reproducibility

All 50 project-owned C source/header files were inventoried: `src/*.[ch]`,
`robocar.c`, `lwipopts.h`, and the three C bench programs under `tests/`.
`dhcpserver/` is MicroPython MIT-licensed third-party code and was excluded.
Pico SDK, lwIP, generated build sources and import scripts were not refactored.
Local Wi-Fi credentials are not included in the review patches.

`review/01-ultrasonic.patch` contains the functional source changes relative to
1eb62b8. `review/02-coding-standard.patch` contains the subsequent source/style
changes relative to that functional baseline. They are ordered patches, not two
alternative firmwares. Test and documentation updates accompany the working
changes; no commit, push or flash is performed by this review.

Install `tools/barr-requirements.txt` in a disposable Python environment, then:

```
python3 tools/check_barr.py
python3 tools/check_barr.py --syntax --report review/barr-findings.json
clang-format --dry-run --Werror src/*.[ch] robocar.c lwipopts.h tests/*.c
cmake --build build --target supercar -j4
```

The basic layout check runs as a dependency of `supercar`. The syntax audit is
an additional parser-based review, not a complete C type or BARR-C checker.
The six parser findings are initializer-fragment macros in config.h; the target
compiler accepts them. `review/static-analysis.txt` records the target GCC
`-fanalyzer` compilation of all 24 project-owned C translation units with
`-Wall -Wextra -Wformat=2 -Wconversion -Wsign-conversion -Wshadow
-Wstrict-prototypes -Wmissing-prototypes`, at `-O0`, to temporary objects.
These analysis objects are not linked into the firmware.

## Rules addressed

- 1.2.a, 3.4–3.6: 80-character lines, four-space block indentation, no tabs,
  LF line endings. Continuation indentation follows the expression structure.
- 1.3: mandatory control-flow braces and Allman block brace layout.
- 1.4: explicit expression grouping and logical operand parentheses, preserving
  the original syntax-tree grouping and short-circuit order.
- 2.2.h: Doxygen file and function briefs, including previously undocumented
  scan, servo, planner and recovery helpers. Existing calibration/geometry and
  hardware explanations remain alongside the implementation.
- 3.1–3.3: consistent spacing/alignment, expanded single-line bodies, separated
  executable statements, and end-of-file markers.
- 4.2.b, 4.3.c: include guards and module-header checks retained. Compiler and
  host builds check public declarations against their implementations.
- 4.4: project-owned source/header templates under tools/templates/.
- 8.1.a: split multiple object and structure-field declarations.
- 8.3.b: explicit no-action defaults for previously unmatched switch values.
- 1.8, 5.2, 6.5, 7.2: reviewed existing static/const/volatile use, fixed-width
  hardware counters and ISR-shared state. No blanket qualifier/type changes
  were made to timing-sensitive drivers. GCC checks initialization and types.

The refactoring-only binary comparison with the functional-only baseline is
recorded in `review/binary-comparison.txt`. Test fixtures remain synthetic;
they are not presented as physical sensor calibration or floor validation.

## Remaining deviations and review limits

| Rules | Location / deviation | Reason and disposition |
| --- | --- | --- |
| 1.1.a,c | CMake C11 target, `_Static_assert`, Pico SDK extensions | Forcing C99 fails on SDK `static_assert` declarations. Keep the working toolchain; SDK adaptation requires separate review. Do not claim strict C99 compliance. |
| 1.5 | Existing short identifiers and abbreviations | Glossary below; full symbol renaming remains open. |
| 1.6; 5.2.b; 5.3 | Native `long`/`unsigned long` formatting casts, SDK ABI types and mixed conversions | Preserve printf/SDK contracts and timer wraparound. Audit locations are in the JSON and compiler report. Converting these requires range-specific review; warnings are not dismissed as proof of correctness. |
| 1.7.c,d; 8.5.a | Two retained continue statements | Preserve established scan/decoder iteration flow. No forbidden auto/register or nonlocal library jumps were introduced. |
| 1.8.b,c; 7.2 | Existing mutable locals and shared state | Const coverage and all concurrent interleavings are not formally proven. ISR counters/echo timestamps remain volatile; existing synchronization was retained. Adding locks/interrupt masking could change pulse timing and is deferred. |
| 2.1.c.ii | Detailed scan diagnostics remain enabled in Release | Explicit requirement to restore physical scan logs; hiding them with NDEBUG would change the requested output and timing. |
| 2.2; 4.3.e | Documentation quality and include minimality | Added briefs do not prove every precondition, parameter, range cast and unused transitive include is documented. Further manual review remains necessary. |
| 3.1.j; 3.2; 3.3.b | Formatter function-name spacing and natural-block spacing | Formatter cannot express every BARR-specific declaration/alignment convention. Remaining fine-grained style needs manual review. |
| 4.1.b,d; 4.2.a,c; 4.3.a,b | Legacy file/module layout | Bench main filenames and shared/header-only assets remain; scan_plot.h owns private string storage, bypass_geometry.h contains inline implementation. Changing module/API layout is deferred. |
| 5.1.c; 6.1.d,i; 7.1 | Legacy type/function/variable names and prefixes | Some short locals, pointer/Boolean prefixes and long identifiers remain. Retain API/link contracts; mechanical layout checks do not certify naming. |
| 5.4 | Native float/double geometry, direct comparisons, finite checks | Keep the tested numerical model. Renaming types and adding finite/error paths require separate numerical and target validation. |
| 5.5 | SDK/network structure layout | SDK types remain vendor-owned; no project wire format changed. Layout is not independently verified for every alternative platform. |
| 5.6.b | Existing SDK bitmask/Boolean idioms | Preserve logic; complete essential-type analysis remains open. |
| 6.2.a,c; 8.2.b | 16 functions exceed 100 lines; multiple exits and deep branches remain | Splitting state machines or changing guard order risks altered timing and error behavior. Locations in JSON; no redesign performed. |
| 6.3 | Existing macros and conditional configuration | Preserve compile-time APIs; macro side-effect safety requires call-site review. |
| 6.5.a,c,d | GPIO callback architecture and public edge handlers | Pico SDK installs the actual IRQ trampoline; driver callbacks are ordinary C functions. Do not add ISR attributes to callbacks. Default vector handling remains SDK-owned. |
| 8.2.c,d; 8.4; 8.6 | Existing control expressions, loop literals and equality operand order | Remaining rule coverage is incomplete. Reordering side effects or refactoring control flow is deferred; no full-compliance claim. |

This is a broad behavior-preserving remediation, with explicit remaining work,
not approval of the listed deviations or a substitute for target testing.

## Abbreviations (Rule 1.5.b)

ADC: analog-to-digital converter; AO: analog output; GPIO: general-purpose I/O;
IMU: inertial/magnetic sensor; IR: infrared; ISR: interrupt service routine;
MQTT: Message Queuing Telemetry Transport; PWM: pulse-width modulation;
UF2: firmware block format; USB: Universal Serial Bus; US: ultrasonic;
HDG: heading; ODO: odometry; RX/TX: receive/transmit; msec/usec: milliseconds/
microseconds; mm: millimetres; deg: degrees; gb/gp: global Boolean/pointer.

## Final verification results

- All seven host suites pass; full output: `review/host-tests.txt`.
- Added 200 mm measured-range dense footprint validation; existing 150 mm
  regression cases remain. No new floor test or sensor calibration is claimed.
- Clean Release `supercar` build succeeds. No project-owned compiler warnings;
  eight GCC stringop-overflow warnings originate in vendor lwIP dhcp.c.
  Vendor files were not changed. Log: `review/firmware-build.txt`.
- Expanded compiler analysis completes for 24 translation units with no errors:
  24 conversion warnings remain for numerical/SDK contracts, plus one analyzer
  warning on the intentionally infinite firmware tick loop. These are recorded,
  not suppressed. Review them before approving deviations.
- Syntax audit records 139 findings: 115 native-width integer uses (primarily
  formatting casts), 16 long functions, two continue statements, and six macro
  parser limitations. This list does not enumerate every semantic deviation.
- Layout/mandatory-brace/logical-parenthesis/header checks, clang-format dry run,
  and `git diff --check` pass. Source/test patches reverse-check successfully.
- The final binary matches the functional-only build except for the one-byte SDK
  build-date change. This does not establish identical real-time floor behavior.
- Final firmware is `build/supercar.uf2`. It is not flashed or pushed.

Additional layout review: Rule 8.3.a break/case alignment is not fully enforced by
this formatter configuration and remains an open fine-grained style deviation.
