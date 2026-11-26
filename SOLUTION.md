# Decentralized robot controller

`src.hpp` implements the permitted Controller extension. It uses only each
robot's private target/speed and the public Monitor observations. There are no
global/static coordination variables and no shared-memory accesses.

The controller combines reciprocal velocity-obstacle constraints with a
conservative separating-plane constraint for every pair. Each robot can consume
at most half of the pair's projected clearance in one time interval. Thus the
pair remains separated throughout the interval, even when the other robot
changes its velocity unexpectedly. The feasible velocity closest to the preferred
velocity is found by enumerating edge projections and boundary intersections of
the speed disk and half-planes. If predictive constraints are infeasible, the
controller retains the separating constraints, for which zero is feasible.

A clockwise steering bias breaks symmetric encounters. Stalled robots use a
visibility graph around observed obstacle disks to find detours. Smaller robots
temporarily yield outward when a larger, nearly stationary neighbor is trapped;
this allows motion in tightly packed configurations where a stationary robot at
its target would otherwise block all exits.

For N robots, ordinary velocity selection uses O(N^3) time and O(N) space per
robot. The occasional visibility graph also uses O(N^3) time and O(N) storage
(20 candidate vertices per obstacle). The problem has at most 12 robots.

Run `python3 tests/run_samples.py` to compile against the unmodified provided
framework and verify completion without collisions or speeding on all eight
distributed samples. This is a heuristic optimization solution; global
completion in every possible configuration is not guaranteed.
