# Full-project BARR-C:2018 audit

This review uses the supplied 87-page PDF, including all appendices, read during
this session. The recursive inventory and all-rule disposition below supersede
the earlier fixed-directory audit. This is not a compliance certificate.

## Inventory and exclusions

`tools/check_barr.py` now discovers C/header files recursively, rather than
limiting inspection to selected modules. There are 50 project-owned files:
24 C translation units and 26 headers. The inventory includes src/, robocar.c,
lwipopts.h and every C bench test. MicroPython dhcpserver/, Pico SDK, dependency
and generated build directories are excluded. Local credentials were restored
to their original tracked bytes; they are not part of the intended changes.

## Rule-by-rule disposition

| Section | Review and disposition |
| --- | --- |
| 1.1 | Deviation: Pico SDK 2.3.1 requires C11 static assertions/extensions. A forced C99 build fails in vendor headers; retain C11 rather than modify vendor code. |
| 1.2 | Pass automated 80-column checks on all inventoried files. |
| 1.3 | Mandatory braces added; Allman layout checked. Initializer braces are not executable control blocks. |
| 1.4 | Parenthesized operator grouping and logical operands checked with syntax trees; original evaluation order retained. |
| 1.5 | Project glossary in BARR_C_REVIEW.md. Legacy API abbreviations retained as documented deviations. |
| 1.6 | Cast locations inventoried. Full range-proof comments remain incomplete, particularly printf and geometry conversions. These are manual-review items, not automatically proven safe. |
| 1.7 | No new auto/register, goto or continue constructs introduced. Existing iteration behavior preserved. |
| 1.8 | Private driver/state functions remain static, readonly API inputs const, IRQ-updated counters/echo state volatile. Universal const coverage and concurrency correctness are not formally proven. |
| 2.1 | No new disabled/commented executable blocks. Release scan diagnostics remain enabled by explicit user requirement, deviating from NDEBUG-only diagnostics. |
| 2.2 | Added module/function briefs and retained hardware/calibration assumptions. Complete parameter/range/reference documentation remains open; generated briefs alone are not certification. |
| 3.1 | Spacing normalized. Function-definition spacing differs from BARR's special declaration convention; documented formatter limitation. |
| 3.2 | Adjacent declarations/assignments aligned. Exact alignment in every mixed block requires further manual review. |
| 3.3 | Executable statements separated and EOF markers present; natural-block blank-line compliance is not exhaustively certified. |
| 3.4 | Four-space block indentation; expression continuations follow readable grouping. |
| 3.5 | No literal tabs in the inventoried files. |
| 3.6 | LF/control-character checks pass. |
| 4.1 | Lowercase filenames checked. Legacy bench filenames without main, and first-eight-character module collisions, remain deviations. |
| 4.2 | Guards checked. Header-only private geometry/assets and missing one-to-one main headers remain documented architecture deviations. No new extern data introduced. |
| 4.3 | Own-module headers retained; no absolute/source-file includes introduced. Public/private definition ordering and transitive include minimality remain incomplete. |
| 4.4 | Source/header templates provided under tools/templates/. |
| 5.1 | Existing typedef conventions retained. Module prefixes and SDK anonymous/tagged structures are not fully normalized; API/layout changes deferred. |
| 5.2 | Hardware counters use fixed-width types. Native long printf casts are retained for current varargs ABI; individually inventoried deviations. |
| 5.3 | Timer wraparound and SDK bit masks retained. GCC conversion diagnostics recorded; mixed-type elimination requires range-specific changes and separate numerical review. |
| 5.4 | Deviation: retain native float/double, existing comparisons and error paths. Arithmetic/finite-check changes could change timing, failure behavior and footprint planning. |
| 5.5 | No struct layout, bitfield or serialization order changed. SDK wire types remain vendor-owned; cross-platform layout not independently proven. |
| 5.6 | Existing bool declarations retained. SDK integer/Boolean idioms remain a manual-review item. |
| 6.1 | Function naming scanned; public interfaces retained. Historical API/module-prefix deviations remain documented. |
| 6.2 | Private static/public prototypes retained. Long state-machine functions and early guard returns are retained deviations to preserve behavior and timing. |
| 6.3 | Existing SDK/configuration macros retained; no new runtime-control macros. Existing macros still require side-effect review. |
| 6.4 | Cooperative functions use task naming; firmware main loop intentionally has no termination. No RTOS scheduling introduced. |
| 6.5 | SDK GPIO trampoline remains the installed IRQ handler. Driver callbacks timestamp/count only and retain _isr names; no inappropriate ISR attributes added to ordinary callbacks. Default IRQ vectors remain SDK-owned. |
| 7.1 | Short definition-local variables renamed consistently; corresponding prototype parameter updated. Further descriptive-name/pointer-prefix/API conventions remain deviations. |
| 7.2 | Initialization checked by GCC diagnostics/analyzer and host UBSan tests. Existing BSS zero initialization and IRQ snapshot logic retained. Complete alias/interleaving proof remains open. |
| 8.1 | Multiple object/field declarations split; type/layout and initialization order preserved. |
| 8.2 | Guard order and state transitions preserved. Deep nesting and missing terminal else idioms remain documented deviations rather than state-machine redesign. |
| 8.3 | Explicit no-action defaults added. Existing fallthrough behavior preserved. BARR-specific case/break alignment remains a formatter deviation. |
| 8.4 | Iteration bounds, side effects and timeouts preserved. Magic loop limits and loop-condition assignments require further manual review; no semantic rewrite made. |
| 8.5 | No new nonlocal jumps or abort/exit paths introduced. Existing SDK assertion/restart hooks retained. |
| 8.6 | Existing constant-left tests retained; universal equality operand ordering is not certified. Reordering side-effect expressions is deferred. |

## What the findings mean

`full-barr-findings.json` lists concrete syntax findings and candidate manual
review locations. Cast entries are not 467 independently proven bugs. Native
float and long entries are recorded deviations. The config macro parser finding
is checked by the real compiler; it is not a compiler error. Unchecked semantic
requirements are listed above rather than silently treated as passed.

Deviations remain pending project-owner approval. Preserving behavior is the
reason for deferring numerical, locking, API and state-machine rewrites; it is
not a claim that the existing code is fully compliant. Full conformance cannot
honestly be claimed on the evidence available here.

## Verification artifacts

- final-tests.txt: seven host suites, including independent dense footprints.
- final-build.txt: firmware and legacy/bench target build output.
- final-static-analysis.txt: GCC -fanalyzer and expanded diagnostics for 24 C units.
- full-barr-findings.json: recursive rule scan and manual-review locations.
- final-summary.json: exact inventory/change counts and binary hashes.

Earlier 01/02 patches and reports are historical snapshots; the committed diff
is authoritative for the final submission, including the already authorized
ultrasonic measurement reporting fixes from previous turns.

## Remote integration before push

The first push discovered remote commit 5c0b13c (1.0 mm hump-entry height).
That commit is preserved as the parent of this audit commit. Its height-based
entry/leave conditions and diagnostic are retained; the host fixture now mocks
height explicitly. All seven suites, the final UF2 build and analyzer were rerun.
The audit-only binary comparison was identical before this integration; the
final combined binary includes the remote hump behavior, so that comparison is
not a claim that the final binary equals the older pre-remote image.
