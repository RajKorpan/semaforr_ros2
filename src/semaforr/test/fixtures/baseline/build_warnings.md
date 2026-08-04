# Initial ROS 2 Humble build warning inventory

The baseline build completed successfully on 2026-07-27 in approximately
67 seconds. It emitted repeated warnings from header-defined functions. The
unique warning locations are summarized here:

| Location | Function or issue |
|---|---|
| `FORRGeometry.cpp:240` | `get_perpendicular` can reach the end without returning |
| `Map.cpp:89` | `Map::readMapFromXML` can reach the end without returning |
| `PathPlanner.cpp:1038` | `computeNewEdgeCost` can reach the end without returning |
| `Task.h:127` | `Task::getX` can reach the end without returning |
| `Task.h:161` | `Task::getY` can reach the end without returning |
| `Task.h:189` | `incrementDecisionCount` declares `int` but returns nothing |
| `Task.h:198` | `saveDecision` declares `FORRAction` but returns nothing |
| `Task.h:272` | `getWaypoints` can reach the end without returning |
| `Task.h:393` | `generateWaypoints` declares `bool` but returns nothing |
| `Task.h:426` | `generateOriginalWaypoints` declares `bool` but returns nothing |
| `Task.h:1715` | `generateWaypointsFromInds` declares `bool` but returns nothing |
| `Tier1Advisor.h:49` | `localExplorationStarted` declares `bool` but returns nothing |
| `HighwayExplore.h:1194` | `exploreDecision` can reach the end without returning |
| `FrontierExplore.h:595` | `exploreDecision` can reach the end without returning |
| `Tier3Advisor.cpp:357` | `makeAdvisor` can reach the end without returning |
| `AgentState.h:152` | `getPlansWaypoints` can reach the end without returning |

All listed non-void fall-through defects were resolved during Phases 3-9.

The final ROS 2 Humble build on 2026-07-29 still records 5,025 repeated warning
lines because large legacy headers are compiled into many translation units.
The leading categories are:

| Category | Repeated lines |
|---|---:|
| signed/unsigned comparisons | 4,325 |
| member initialization order | 306 |
| deprecated implicit copy assignment | 153 |
| unused variables | 106 |
| possibly uninitialized values | 45 |
| unused parameters | 44 |

These remain a recorded modernization backlog rather than approved behavior.
The new domain, coordinator, planning, configuration, and ROS-adapter code is
compiled with `-Wall -Wextra -Wpedantic`; no warning suppression was added.
