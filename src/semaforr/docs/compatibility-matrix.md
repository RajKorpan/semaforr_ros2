# Behavioral compatibility matrix

This document defines what implementation claims mean. A component name is
not evidence that a particular algorithm is present. The status and behavior
mode recorded here are the authority for publications, experiments, and
regression review.

## Status vocabulary

- **Dissertation-faithful** reproduces the published algorithm, inputs,
  lifecycle, ordering, and observable behavior closely enough for the
  compatibility acceptance suite.
- **Functionally adapted** intentionally uses a different algorithm while
  preserving the component's purpose. It is supported in modernized mode but
  must not be described as an exact reproduction.
- **Engineering extension** is intentional behavior outside the published
  system.
- **Temporary approximation** is a known simplification with a planned
  faithful or explicitly adapted replacement.
- **Unsupported or incomplete** is exposed by code, configuration, messages,
  or documentation but is not operational end to end.

## Operating modes and claim policy

`experiment.behavior_mode` has two values:

- `modernized` is the supported default. It permits the safety envelope,
  deterministic seeded choices, weighted normalized advisors, incremental
  learners, typed plans, registries, caching, and other documented
  adaptations.
- `compatibility` is reserved for exact experimental reproduction. Startup
  currently rejects it with the unresolved blocker list. This prevents an
  experiment from claiming compatibility merely because historically named
  components were enabled.

The target is to make compatibility mode operational without weakening the
hard command-safety boundary. Safety interventions must be recorded so they
can be separated from cognitive decisions during analysis.

Every reported run must include the behavior mode, ablation profile, random
seed, configuration fingerprint, component manifest, source revision, map,
target sequence, and test-suite revision. Historical profile names such as
`original`, `doors`, or `highway` identify experimental feature sets only;
they are not fidelity claims.

## Representations and learners

| Component | Status | Published algorithm and lifecycle | Current implementation | Intentional? | Behavioral consequence | Exact reproduction? | Planned resolution |
|---|---|---|---|---|---|---|---|
| Decision path/history | Dissertation-faithful | Record the completed decision point, action, pose, and view after execution | Separates decision, command, execution, observation, and successful completed-path histories; terminal feedback carries actual pose and outcome under stable IDs | Yes; IDs and detailed failure states are an engineering extension | Failed and partial actions remain diagnosable but cannot be learned as successful traversal | Yes for successful-path lifecycle | Retain the additional outcome metadata in both modes |
| Trail / `TrailLearner` | Temporary approximation | At target end, work backward and select the earliest point that could sense the current marker | Distance-samples the incremental trace | No | Trails retain detours and are not guaranteed visibility shortcuts | No | Implement visibility-backtracking trail finalization in compatibility mode |
| Conveyor / `ConveyorLearner` | Temporary approximation | Count successful trail occupancy in a footprint grid at target end | Counts and merges similar completed traversal segments | No | Directional segment frequency replaces trail-cell familiarity | No | Add trail-derived conveyor grid; retain segment flows as a modernized representation |
| Region / `RegionLearner` | Temporary approximation | Per-decision minimum-range circle; reconcile overlapping candidate regions after a target | Incrementally clusters nearby poses, averages centers, and grows radius | No | Region identity, size, overlap, and skeleton topology differ | No | Implement target-boundary candidate reconciliation and visibility evidence |
| Exit | Unsupported or incomplete | Traveled location on a region circumference, accumulated at target end | No distinct exit representation | No | Door learning and region transitions lack successful-travel evidence | No | Add first-class exits owned by the region learner |
| Door / `DoorExitLearner` | Temporary approximation | Group consecutive nearby exits into region-relative door arcs | Detects short scan segments at adjacent range discontinuities | No | Openings need not correspond to successful region entry or exit | No | Implement exit-based arcs for compatibility; retain scan openings as a separately named modernized model |
| Hallway / `HallwayLearner` | Functionally adapted | Direction partition, all-pairs similarity, parent/child inference, heatmap, smoothing, connected components, merging at target end | Orientation/spatial bins over sufficiently long traversed centerlines | Yes, for runtime cost | Faster and scalable, but hallway width and inferred aggregates differ | No | Keep modernized learner; add a separate compatibility learner and scenario fixtures |
| Barrier / `BarrierLearner` | Engineering extension | Not a Chapter 3 affordance | Learns barrier segments for modern safety and planning support | Yes | Adds knowledge unavailable to the published controller | No | Retain only in modernized mode and manifest it explicitly |
| Optional static map | Engineering extension | Mapless learning is primary; known-map planners are retained as baselines | Immutable, provenance-tagged wall geometry and occupancy loaded only in `map_enabled` mode and kept separate from learned models | Yes | Enables controlled known-map comparisons without changing mapless learning | Not applicable to mapless reproduction | Retain as an explicitly manifested experimental capability |
| Known grid / `KnownGridLearner` | Dissertation-faithful | Count cells visible along laser rays, at most once per decision | Sparse ray-cell observation counts, projected to a snapshot | Yes, storage optimized | Equivalent familiarity intent with different storage | Pending oracle tests | Add fixed-view cell-count fixtures and recent-window support for Out |
| Inclusion grid / `InclusionGridLearner` | Temporary approximation | Mark cells included in regions and region-connecting subtrails; update during LLE | Primarily marks accepted robot poses | No | LLE coverage and fallback targets differ materially | No | Derive inclusion from regions and skeleton subtrails and retain target-local lifetime |
| Passage grid | Temporary approximation | Free/obstructed/passage-number labels updated locally during HLE | Sparse scan observation counts without passage identity | No | HLE cannot enforce published cue overlap and passage termination tests | No | Add typed cell state and passage IDs |
| Skeleton / `PassageSkeletonLearner` | Temporary approximation | Region nodes, direct region-transition edges, shortest subtrail labels, stable visibility | Spaced path nodes with sequential and inferred branch edges | No | Surrogates, connectivity, plans, LLE, and explanations use a different graph | No | Build the compatibility skeleton from finalized regions, exits, paths, and subtrails |
| Region visibility | Unsupported or incomplete | 360 one-degree maximum visibility with source/end points for every region | No complete `R*` representation; consumers substitute centers/history rays | No | Skeleton surrogates and LLE omit published visibility cues | No | Add immutable per-region visibility snapshots |
| Highway grid / `HighwayLearner` | Functionally adapted | Interpolate exploration path, 3-of-4 smoothing, minimum row/column extents, intersections, spurs, spreading activation, largest intersection component | Incremental touched rows/columns, directional gap fill, extents, intersections, spurs, largest vertex component | Partly, for incremental efficiency | Similar abstractions with different cell membership and component choice | No | Parameterize geometry and add compatibility smoothing/component policies |
| Highway and intersection types | Dissertation-faithful | Long horizontal/vertical extents and their overlap/terminal access points | First-class typed cells, axes, endpoints, intersections, and revisions | Yes | Storage is stronger without changing intent | Pending oracle tests | Add published-map extraction fixtures |
| Highway graph | Functionally adapted | Intersection nodes; highway edges labeled with path-derived subtrails and centroid distance | Typed graph with highway edges, distance, and identifier labels | No | Operationalization lacks the published highway subtrail fallback | No | Store actual path-derived subtrail labels and select by intersection count in compatibility mode |
| Setting normalization | Dissertation-faithful | Robot-centered, east-facing discretized freespace grid | Robot-centered heading-normalized `NormalizedSetting` | Yes | Equivalent purpose with configurable geometry | Pending oracle tests | Freeze compatibility parameters and add canonical setting fixtures |
| Circumstance / `CircumstanceLearner` | Functionally adapted | Offline spectral clustering, neural classifier, confidence rejection, then online assignment and periodic reclustering | Deterministic L1 seed grouping, centroid updates, confidence and evidence gates | Yes, to remove offline model/runtime dependency | Cluster identities and assignments cannot reproduce published cases | No | Provide a compatibility model loader and published clustering/classification pipeline |
| Crowd field / learner | Engineering extension | Social planners predate the Chapter 3 model; no unified learned field in these chapters | Learns versioned density, risk, and flow snapshots | Yes | Adds social learning and new objectives | No | Retain in modernized mode and disable through the social master switch for compatibility |

## Exploration, tiers, rules, and reactive planners

| Component | Status | Published algorithm and lifecycle | Current implementation | Intentional? | Behavioral consequence | Exact reproduction? | Planned resolution |
|---|---|---|---|---|---|---|---|
| Navigation phases | Engineering extension | HLE occurs before target navigation, but phases are implicit | Explicit initial exploration, target navigation, and mission complete phases | Yes | Makes lifecycle testable without changing intended ordering | Yes at lifecycle level | Retain in both modes |
| HLE | Temporary approximation | Left/right fixed ray bundles, length/width and large-room tests, segment-overlap cue similarity, numbered passage grid, rich pursuit termination | Finds contiguous clear bundles, hashes cue endpoints, pursues a short displacement, records observation counts | No | Exploration coverage, path, passages, highways, and time use differ substantially | No | Implement the published cue and pursuit state machine behind compatibility mode |
| LLE | Functionally adapted | Trigger on no/finished plan, use HLE cues, path views, region visibility, inclusion grid, 20 waypoints, random closest-bin fallback, 10% replanning | Stateful deterministic candidates from HLE/history/regions/gaps, waypoint loss and 10% replanning, plus stall trigger | Partly, for determinism and recovery | Trigger frequency and chosen rays differ; missing `R*` and inclusion semantics limit fidelity | No | Restore published trigger and candidate sources in compatibility mode |
| Hard safety filter | Engineering extension | Obstacle avoidance is cognitive Tier 1 | Independent sensor, finite-command, clearance, and action validity filter | Yes | May veto an action the published cognitive system would select | No bit-for-bit; safety interventions separable | Retain and record interventions in both modes |
| Command safety envelope | Engineering extension | Platform execution details not part of tier reasoning | Velocity, acceleration, freshness, bounds, timeout, and final finite checks | Yes | Safer platform behavior and possibly different trajectories | No bit-for-bit | Retain; require intervention-free runs for strict behavioral comparisons |
| Tier ordering | Temporary approximation | Ordered Tier 1; Tier 2 only if no plan; return to Tier 1; Tier 3 only after Tier 1 cannot decide | LLE, plan preparation, hard safety, reactive planners, then Victory/vetoes/Tier 3 | No | Planner work occurs early; reactive planners can precede published priorities; attribution differs | No | Implement a compatibility decision-cycle coordinator with explicit tier returns |
| Victory | Dissertation-faithful | Turn or move toward sensed unobstructed target | Direct visibility, turn, move, or pause within tolerance | Yes | Typed actions and tolerance are platform adaptations | Pending scenario oracle | Add published action-repertoire fixtures |
| AvoidObstacles | Functionally adapted | Veto moves that approach within epsilon of sensed obstacle | Swept-corridor laser clearance plus hard safety | Yes, for footprint safety | More conservative than the published forward-distance check | No | Provide published cognitive rule in compatibility mode while retaining hard safety separately |
| NotOpposite | Dissertation-faithful | Avoid return to the last two orientations | Veto turns whose predicted heading matches recent orientations | Yes | Equivalent with typed tolerance | Pending oracle tests | Freeze compatibility tolerance |
| Enforcer | Temporary approximation | Tier-1 action selector with one-step lookahead; operationalizes typed steps every decision | Converts one hierarchical step into a mission waypoint; local action is usually selected later by Tier 3 | No | Plan-following tier attribution and action sequence differ materially | No | Restore Enforcer as an ordered Tier-1 mandatory rule in compatibility mode |
| Thru | Functionally adapted | After Victory/Enforcer fail, follow clearer side cue in 0.8 m steps until goal, interruption, or budget | Stateful clearer-side pursuit before ordinary Tier-1 arbitration | No for ordering; yes for interface | Can assume control at different times | No | Move trigger behind Victory/Enforcer in compatibility coordinator |
| Behind | Functionally adapted | If nearby region/waypoint/target was unseen twice, mandate available 90-degree right then left turn | Fixed 1.5 m point threshold and nearest right 90-degree turn | No | Region radius and left fallback are absent | No | Implement typed region distance and viable-turn fallback |
| Out | Functionally adapted | Recent `10+n/50` known grid, four 90-degree surveys, then prepend reverse subtrail for Enforcer | Cumulative known grid, direct stateful reverse-history execution | No | Trigger and escape path differ; bypasses Enforcer | No | Add target-local recent grid and subtrail plan injection |
| Forward | Functionally adapted | Visited footprint grid updated by Enforcer; veto projected rotations; clear if all rotations vetoed | Stored visited plan positions and distance checks | Yes, storage simplified | Similar rationale with different cell boundaries and Enforcer coupling | No | Add explicit compatibility visited grid |
| Precedent | Functionally adapted | Circumstance/distance/angle case; published confidence and accuracy thresholds | Same evidence equations plus minimum-evidence gates on adapted circumstances/trails | Yes for evidence safety | More conservative activation and different case identities | No | Use faithful trail/circumstance sources in compatibility mode |

## Global planners

| Planner | Status | Published algorithm | Current implementation | Intentional? | Behavioral consequence | Exact reproduction? | Planned resolution |
|---|---|---|---|---|---|---|---|
| Distance / A* | Temporary approximation | A* on a metric occupancy cost graph | Dijkstra on nonzero known-grid cells, sampled skeleton fallback, or direct fallback | No | Static obstacles and unknown/free semantics differ | No | Load map occupancy and provide the published graph/search policy |
| Crowd density | Functionally adapted | Crowd-sensitive occupancy cost graph | Shared graph with learned density multiplier | Yes | Uses a unified learned crowd model | No | Retain modernized; add legacy objective adapter only if reproduction requires it |
| Crowd risk | Functionally adapted | Penalize risky crowd encounters | Shared graph with learned encounter-risk multiplier | Yes | Risk estimator differs | No | Retain modernized |
| Crowd flow | Functionally adapted | Penalize travel against crowd flow | Shared graph with learned flow-alignment multiplier | Yes | Flow source and normalization differ | No | Retain modernized |
| RegionPlan | Temporary approximation | Published Table 3.1 weights over occupancy graph and faithful regions/doors/exits | Similar weights over known-grid graph and adapted regions/openings | No | Candidate paths and objective costs differ | No | Resolve occupancy, region, door, and exit blockers |
| HallwayPlan | Temporary approximation | Published hallway edge weights over occupancy graph | Similar weights using centerline proximity over known-grid graph | No | Different hallway geometry and base graph | No | Use compatibility hallway aggregates and occupancy graph |
| TrailPlan | Temporary approximation | Published trail-marker weights over occupancy graph | Similar marker proximity over known-grid graph | No | Adapted trails change preferred routes | No | Use faithful trails and occupancy graph |
| ConveyorPlan | Temporary approximation | Published conveyor-cell weights over occupancy graph | Segment proximity and aggregate traversal counts | No | Objective is not the published cell-frequency cost | No | Add conveyor grid objective |
| SkeletonPlan | Temporary approximation | Dijkstra over region graph using region/visibility surrogates and subtrail labels | Plans over sampled skeleton geometry | No | Start/goal attachment and region sequence differ | No | Restore region skeleton, visibility, and subtrail labels |
| HighwayPlan | Functionally adapted | Skeleton connection, shortest highway graph path, skeleton connection; compare with SkeletonPlan | Implements combined highway/skeleton routing and compares alternatives using adapted graphs | Yes at architecture level | Route differs with learned graph and surrogate details | No | Reuse faithful graphs and published surrogate rules in compatibility mode |
| Tier-2 range voting | Dissertation-faithful | Each objective evaluates every plan, normalize costs to `[0,10]`, minimize summed score | Same range-vote structure with deterministic planner-name tie break | Yes, tie adapted | Ties differ; ordinary non-tied selection should match given identical plans/costs | Pending oracle tests | Random tie policy in compatibility mode; retain deterministic option in modernized mode |
| Other selection policies | Engineering extension | Not described | Single, normalized minimum, Pareto-then-vote, shortest-valid | Yes | Enables new experiments | No | Modernized mode only |
| Plan cache and revision invalidation | Engineering extension | Not described | Caches by task and start/target surrogates plus exact declared representation and policy revisions | Yes | Relevant mutations invalidate precisely; unrelated layers remain cached | Potentially | Implemented and covered by exact-dependency regression tests |

## Tier-3 advisors and voting

Unless a row says otherwise, the listed advisor preserves the published
rationale but uses anticipated poses, configurable weights, and per-decision
signed normalization instead of unweighted `[0,10]` comments.

| Advisor | Status | Published rationale | Current consequence / deviation | Exact reproduction? | Planned resolution |
|---|---|---|---|---|---|
| BigStep | Functionally adapted | Prefer a long step | Scores safe forward magnitude; turns receive a partial utility | No | Add published comment oracle |
| ElbowRoom | Functionally adapted | Stay far from obstacles | Scores predicted nearest laser endpoint | No | Freeze published metric and scale |
| Novelty | Functionally adapted | Avoid locations visited on current target | Uses target-tagged navigation history | No | Add published distance/comment mapping |
| GoAround | Functionally adapted | Turn away from nearby obstacle | Maximizes heading separation from nearest scan endpoint | No | Add published lookahead and scale |
| Greedy | Functionally adapted | Approach target or active plan step | Uses mission waypoint correctly, but normalized weighted score differs | No | Compatibility `[0,10]` comment implementation |
| Curiosity | Functionally adapted | Visit locations never visited in experiment | Uses all navigation history | No | Compatibility comment scale |
| Enfilade | Functionally adapted | Return toward recent locations | Uses last ten sufficiently distinct positions | No | Match published history window and score |
| VisualScan | Functionally adapted | Rotate toward orientations with least prior view overlap | Samples unseen angular coverage in nearby history | No | Match published overlap calculation |
| Convey | Functionally adapted | Approach frequent distant conveyors | Uses segment frequency rather than conveyor cells | No | Depends on faithful conveyor grid |
| Enter | Temporary approximation | Enter target/plan-step region | Uses final target region rather than every operationalized plan step | No | Pass typed active plan step to advisor context |
| Exit | Temporary approximation | Leave a region without target/plan step | Uses final target and adapted regions | No | Pass plan step and faithful regions |
| Trailer | Temporary approximation | Follow trail segment that approaches target/plan step | Selects useful adapted trail segment toward final target | No | Use plan step and faithful trails |
| Unlikely | Temporary approximation | Avoid dead-end regions | Infers low-door regions instead of skeleton degree | No | Use region skeleton degree |
| Access | Temporary approximation | Approach regions with many doors | Counts scan-derived openings near adapted regions | No | Use faithful doors/regions |
| Crossroads | Functionally adapted | Approach hallways with many overlaps | Computes centerline overlap dynamically | No | Use faithful hallway aggregate labels |
| Follow | Temporary approximation | Follow target-relevant hallway | Uses final target and adapted centerlines | No | Use active plan step and faithful hallway model |
| LeastAngle | Temporary approximation | Leave a region through skeleton branch best aligned to target/plan step | Uses nearest sampled skeleton node and final target | No | Use current region, adjacent regions, and active plan step |
| SpatialLearner | Functionally adapted | Prefer locations absent from regions and high conveyors | Uses inclusion, adapted regions, and segment flows | No | Rebase on faithful representations |
| Stay | Functionally adapted | Remain in current hallway | Uses distance to nearest centerline | No | Use faithful hallway area membership |
| Weighted signed voting | Engineering extension | Unweighted `[0,10]` range voting with random tie break | Advisor utilities normalize to `[-1,1]`, receive weights, and use seeded tolerance ties | Yes | Changes coalition strength and decisions | No | Add unweighted `[0,10]` compatibility policy |

The remaining registered advisors are modernized extensions and are classified
individually here so a manifest can never silently imply that they belonged to
the published Tier-3 catalog.

| Advisor | Status | Current rationale | Behavioral consequence | Compatibility policy |
|---|---|---|---|---|
| Random | Engineering extension | Leave all candidates tied for seeded baseline selection | Creates a reproducible random-control condition | Disable unless the compatibility experiment explicitly requires a random baseline |
| GoalProgress | Engineering extension | Maximize predicted progress to waypoint or target over every action | Adds a strong generic navigation vote and partly assumes Enforcer's local role | Disable in compatibility mode |
| GoalProgressLinear | Engineering extension | Score only forward progress | Adds a move-specific target vote | Disable in compatibility mode |
| Clearance | Engineering extension | Maximize anticipated clearance over every action | Adds a continuous safety preference beyond AvoidObstacles | Disable in compatibility mode |
| ClearanceRotation | Engineering extension | Score rotation clearance only | Biases turns independently of the published Advisors | Disable in compatibility mode |
| Exploration | Engineering extension | Prefer less-visited anticipated locations | Overlaps Novelty and Curiosity with different scoring | Disable in compatibility mode |
| AvoidRevisit | Engineering extension | Prefer distance from all recorded positions | Adds a global anti-revisit vote | Disable in compatibility mode |
| PreferRegions | Engineering extension | Prefer anticipated poses deeper in learned regions | Adds a generic region vote not listed in Table 4.3 | Disable in compatibility mode |
| PreferHighways | Engineering extension | Prefer anticipated poses near learned highway nodes | Adds local highway preference to Tier 3 | Disable in compatibility mode |
| PreferDoors | Engineering extension | Prefer anticipated poses near learned openings | Adds a direct door preference distinct from Access | Disable in compatibility mode |
| FollowTrails | Engineering extension | Prefer anticipated poses near any trail marker | Adds generic trail attraction distinct from Trailer | Disable in compatibility mode |
| SocialNavigation | Engineering extension | Use live pedestrian prediction for separation and encounter behavior | Changes action selection from current social observations | Disable with the social master switch in compatibility mode |
| CrowdAvoid | Engineering extension | Avoid cells with learned crowd density | Adds a learned social-field vote | Disable with the social master switch in compatibility mode |
| RiskAvoid | Engineering extension | Avoid cells with learned encounter risk | Adds a learned risk-field vote | Disable with the social master switch in compatibility mode |
| FlowFollow | Engineering extension | Prefer alignment with learned pedestrian flow | Adds a learned directional social vote | Disable with the social master switch in compatibility mode |

## Explanations

| Component | Status | Published algorithm | Current implementation | Intentional? | Behavioral consequence | Exact reproduction? | Planned resolution |
|---|---|---|---|---|---|---|---|
| Why action explanations | Unsupported or incomplete | Question-driven templates for Tier 1, Tier 3, HLE, hypothetical pose, confidence, and alternatives; relative support, Gini agreement, and standardized overall support | Automatically emits one generic tier summary and score-margin confidence per decision | No | Does not explain actual rationales or answer published questions | No | Add explanation trace schema and a question/response service implementing the published templates |
| Tier-1 explanation templates | Unsupported or incomplete | Advisor-specific action, confidence, and alternative-action phrases | Only generic tier labels | No | Victory, Enforcer, reactive, and veto reasons are conflated | No | Publish deciding rule/reactive identity and template inputs |
| Tier-3 rationale explanation | Unsupported or incomplete | Support/opposition clauses selected from relative advisor support | Contributions exist in the record, but the node does not apply the procedure | No | Synergy and conflicting rationales are hidden | No | Implement Chapter 5 relative-support and language tables |
| HLE explanation | Unsupported or incomplete | Generic exploration action/confidence/alternative templates | Reports only that an exploration policy selected the action | No | Cannot explain cue state or even the published generic phrases | No | Expose HLE state/event and implement templates |
| Why plan comparison | Unsupported or incomplete | Compare selected plan and assumed human plan under both objectives; explain preference, alternative, and confidence | Reports planner name, a partial objective phrase, target, and latency | No | No comparative explanation is possible | No | Publish all candidate plans, objective-cost matrix, normalized votes, and selected/alternative plan |
| Route description | Unsupported or incomplete | Operationalize typed steps, bin segment angles/distances, summarize egocentric turns, annotate highway intersections | No route-language implementation and no typed plan in the message | No | Cannot answer “How are we getting there?” | No | Publish hierarchical plan and implement Chapter 5 angle/distance procedure |
| DecisionRecord diagnostics | Engineering extension | Why reads an internal knowledge store | Versioned ROS message with candidates, vetoes, contributions, timing, execution, fingerprint, and manifest | Yes | Strong auditability, but currently omits several Why inputs | No by itself | Extend rather than replace it with an explanation trace message |

## Compatibility-mode exit criteria

Compatibility mode becomes runnable only when all of the following are true:

1. Every temporary approximation required by a selected compatibility profile
   is either dissertation-faithful or explicitly excluded by validation.
2. Tier execution and action attribution match the published decision cycle.
3. Cost-graph planners use a validated occupancy graph and published
   affordance costs.
4. HLE, LLE, spatial update schedules, and target boundaries pass algorithm
   oracle fixtures.
5. Why answers the Chapter 5 question set from recorded explanation traces.
6. Compatibility tests pass without an unrecorded hard-safety intervention.
7. A replay report includes behavior mode, profile, seed, fingerprint,
   manifest, source revision, and exact-match/divergence annotations.
