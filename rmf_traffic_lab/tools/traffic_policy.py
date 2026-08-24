"""Corridor-aware traffic policy used by the RMF Traffic lab.

This module is deliberately independent from Qt and ROS.  The C++ runner writes
the same snapshot schema for the patched DifferentialDrivePlanner, while the
desktop UI uses these types to explain and replay decisions.  Schedule rows are
never presented as core values: all corridor association and policy decisions
are tagged POLICY_DERIVED.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
import math
from pathlib import Path
from typing import Iterable


SOURCE_RMF_CORE = "RMF_CORE"
SOURCE_SCHEDULE = "SCHEDULE"
SOURCE_POLICY = "POLICY_DERIVED"
SOURCE_SIMULATION = "SIMULATION_EVENT"


class PolicyMode(str, Enum):
    BASELINE = "baseline"
    SOFT = "soft"
    HYBRID = "hybrid"
    HYBRID_NEGO = "hybrid_nego"


class CorridorDirection(str, Enum):
    A_TO_B = "A_TO_B"
    B_TO_A = "B_TO_A"
    UNKNOWN = "UNKNOWN"

    def opposite(self, other: "CorridorDirection") -> bool:
        return (
            self is not CorridorDirection.UNKNOWN
            and other is not CorridorDirection.UNKNOWN
            and self is not other
        )


class CorridorState(str, Enum):
    FREE = "FREE"
    RESERVED = "RESERVED"
    OCCUPIED = "OCCUPIED"
    UNKNOWN_HOLD = "UNKNOWN_HOLD"


@dataclass(frozen=True)
class CorridorDefinition:
    corridor_id: str
    lanes_forward: tuple[int, ...] = ()
    lanes_reverse: tuple[int, ...] = ()
    capacity: int = 1
    passing_allowed: bool = False
    hard_opposite_direction_block: bool = True
    holding_entry_a: int | None = None
    holding_entry_b: int | None = None
    base_penalty: float = 0.0

    def __post_init__(self) -> None:
        if not self.corridor_id:
            raise ValueError("corridor_id must not be empty")
        if self.capacity < 1:
            raise ValueError("corridor capacity must be at least one")
        if not math.isfinite(self.base_penalty) or self.base_penalty < 0:
            raise ValueError("corridor base_penalty must be finite and non-negative")
        duplicate = set(self.lanes_forward).intersection(self.lanes_reverse)
        if duplicate:
            raise ValueError(f"directed lanes belong to both directions: {sorted(duplicate)}")

    def direction_for_lane(self, lane_id: int) -> CorridorDirection:
        if lane_id in self.lanes_forward:
            return CorridorDirection.A_TO_B
        if lane_id in self.lanes_reverse:
            return CorridorDirection.B_TO_A
        return CorridorDirection.UNKNOWN


@dataclass(frozen=True)
class PolicyWeights:
    same_direction_per_second: float = 0.25
    opposite_direction_per_second: float = 8.0
    occupied_per_second: float = 1.5
    future_reservation_per_second: float = 0.6
    no_escape: float = 25.0
    static: float = 0.0
    overlap_margin_s: float = 0.25

    def __post_init__(self) -> None:
        for name, value in asdict(self).items():
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and non-negative")


@dataclass(frozen=True)
class OccupancyInterval:
    corridor_id: str
    participant_id: int
    plan_id: int
    route_id: int
    direction: CorridorDirection
    enter_time_s: float
    exit_time_s: float
    state: CorridorState = CorridorState.RESERVED
    owner: bool = False
    responsive: bool = True
    itinerary_version: int = 0
    cumulative_delay_s: float = 0.0
    reached_checkpoint: int | None = None
    source: str = SOURCE_SCHEDULE

    def __post_init__(self) -> None:
        if not math.isfinite(self.enter_time_s) or not math.isfinite(self.exit_time_s):
            raise ValueError("occupancy interval times must be finite")
        if self.exit_time_s < self.enter_time_s:
            raise ValueError("occupancy interval exit must not precede entry")


@dataclass(frozen=True)
class CandidateTraversal:
    participant_id: int
    corridor_id: str
    direction: CorridorDirection
    predicted_enter_time_s: float
    predicted_exit_time_s: float
    lane_ids: tuple[int, ...] = ()
    is_entry: bool = True
    is_exit: bool = False
    already_counted_corridors: frozenset[str] = frozenset()
    has_escape: bool = True
    parent_g: float = 0.0
    base_move_cost: float = 0.0
    rotation_cost: float = 0.0
    event_cost: float = 0.0
    wait_cost: float = 0.0
    h: float = 0.0

    def __post_init__(self) -> None:
        if self.predicted_exit_time_s < self.predicted_enter_time_s:
            raise ValueError("candidate exit must not precede entry")


@dataclass
class ScheduleOverlap:
    participant_id: int
    plan_id: int
    route_id: int
    corridor_id: str
    direction: CorridorDirection
    occupancy_enter_s: float
    occupancy_exit_s: float
    overlap_enter_s: float
    overlap_exit_s: float
    overlap_duration_s: float
    admission_overlap_duration_s: float
    relation: str
    state: CorridorState
    source: str = SOURCE_SCHEDULE


@dataclass
class PolicyDecision:
    hard_block: bool = False
    decision: str = "ACCEPT"
    reason_code: str = "NO_POLICY_COST"
    reason: str = "No dynamic corridor policy cost was applied"
    blockers: list[int] = field(default_factory=list)
    schedule_overlaps: list[ScheduleOverlap] = field(default_factory=list)
    static_penalty: float = 0.0
    shared_traffic_penalty: float = 0.0
    same_direction_penalty: float = 0.0
    opposite_direction_penalty: float = 0.0
    corridor_occupancy_penalty: float = 0.0
    no_escape_penalty: float = 0.0
    total_policy_penalty: float = 0.0
    base_move_cost: float = 0.0
    rotation_cost: float = 0.0
    event_cost: float = 0.0
    wait_cost: float = 0.0
    parent_g: float = 0.0
    final_g: float = 0.0
    h: float = 0.0
    f: float = 0.0
    source: str = SOURCE_POLICY

    def as_dict(self) -> dict:
        result = asdict(self)
        for overlap in result["schedule_overlaps"]:
            overlap["direction"] = str(overlap["direction"])
            overlap["state"] = str(overlap["state"])
        return result


class CorridorIntervalIndex:
    """Immutable, per-planning-invocation interval index.

    The caller must build a new instance from a single Schedule Viewer snapshot
    before invoking Planner.  Expansion then only scans intervals for the
    candidate corridor; it never queries the live Schedule and never scans all
    participants.
    """

    def __init__(
        self,
        definitions: Iterable[CorridorDefinition],
        intervals: Iterable[OccupancyInterval],
        *,
        schedule_version: int = 0,
    ) -> None:
        self.definitions = {item.corridor_id: item for item in definitions}
        self.schedule_version = int(schedule_version)
        self.by_corridor: dict[str, tuple[OccupancyInterval, ...]] = {}
        buckets: dict[str, list[OccupancyInterval]] = {}
        for interval in intervals:
            if interval.corridor_id in self.definitions:
                buckets.setdefault(interval.corridor_id, []).append(interval)
        for corridor_id, values in buckets.items():
            values.sort(key=lambda item: (
                item.enter_time_s, item.exit_time_s, item.participant_id,
                item.plan_id, item.route_id))
            self.by_corridor[corridor_id] = tuple(values)

    @staticmethod
    def _overlap(candidate: CandidateTraversal, interval: OccupancyInterval,
                 margin_s: float) -> tuple[float, float, float, float]:
        enter = max(candidate.predicted_enter_time_s, interval.enter_time_s)
        exit = min(candidate.predicted_exit_time_s, interval.exit_time_s)
        actual_duration = max(0.0, exit - enter)
        admission_duration = max(
            0.0,
            min(candidate.predicted_exit_time_s + margin_s,
                interval.exit_time_s + margin_s)
            - max(candidate.predicted_enter_time_s - margin_s,
                  interval.enter_time_s - margin_s),
        )
        return enter, exit, actual_duration, admission_duration

    def evaluate(
        self,
        candidate: CandidateTraversal,
        mode: PolicyMode,
        weights: PolicyWeights,
    ) -> PolicyDecision:
        base_physical = (
            candidate.base_move_cost + candidate.rotation_cost
            + candidate.event_cost + candidate.wait_cost
        )
        output = PolicyDecision(
            parent_g=candidate.parent_g,
            base_move_cost=candidate.base_move_cost,
            rotation_cost=candidate.rotation_cost,
            event_cost=candidate.event_cost,
            wait_cost=candidate.wait_cost,
            h=candidate.h,
        )
        if mode is PolicyMode.BASELINE:
            output.final_g = candidate.parent_g + base_physical
            output.f = output.final_g + candidate.h
            output.reason = "BASELINE mode bypasses every custom corridor policy"
            return output

        definition = self.definitions.get(candidate.corridor_id)
        if definition is None:
            output.final_g = candidate.parent_g + base_physical
            output.f = output.final_g + candidate.h
            output.reason_code = "CORRIDOR_NOT_CONFIGURED"
            output.reason = "The traversal is not associated with a configured corridor"
            return output

        first_charge = candidate.corridor_id not in candidate.already_counted_corridors
        if first_charge:
            output.static_penalty = weights.static + definition.base_penalty

        same_overlap_s = 0.0
        opposite_overlap_s = 0.0
        occupied_overlap_s = 0.0
        future_overlap_s = 0.0
        blocking_intervals: list[OccupancyInterval] = []
        for interval in self.by_corridor.get(candidate.corridor_id, ()):
            if interval.participant_id == candidate.participant_id:
                continue
            overlap_enter, overlap_exit, overlap_s, admission_overlap_s = self._overlap(
                candidate, interval, weights.overlap_margin_s)
            if admission_overlap_s <= 0.0:
                continue
            relation = (
                "OPPOSITE" if candidate.direction.opposite(interval.direction)
                else "SAME" if candidate.direction is interval.direction
                else "UNKNOWN"
            )
            output.schedule_overlaps.append(ScheduleOverlap(
                participant_id=interval.participant_id,
                plan_id=interval.plan_id,
                route_id=interval.route_id,
                corridor_id=interval.corridor_id,
                direction=interval.direction,
                occupancy_enter_s=interval.enter_time_s,
                occupancy_exit_s=interval.exit_time_s,
                overlap_enter_s=overlap_enter,
                overlap_exit_s=overlap_exit,
                overlap_duration_s=overlap_s,
                admission_overlap_duration_s=admission_overlap_s,
                relation=relation,
                state=interval.state,
                source=interval.source,
            ))
            if relation == "SAME":
                same_overlap_s += overlap_s
            elif relation == "OPPOSITE":
                opposite_overlap_s += overlap_s
                if interval.owner or interval.state in {
                    CorridorState.OCCUPIED, CorridorState.UNKNOWN_HOLD,
                }:
                    blocking_intervals.append(interval)
            if interval.state in {CorridorState.OCCUPIED, CorridorState.UNKNOWN_HOLD}:
                occupied_overlap_s += overlap_s
            else:
                future_overlap_s += overlap_s

        if first_charge:
            output.same_direction_penalty = (
                same_overlap_s * weights.same_direction_per_second)
            output.opposite_direction_penalty = (
                opposite_overlap_s * weights.opposite_direction_per_second)
            output.corridor_occupancy_penalty = (
                occupied_overlap_s * weights.occupied_per_second
                + future_overlap_s * weights.future_reservation_per_second)
            if not candidate.has_escape and opposite_overlap_s > 0.0:
                output.no_escape_penalty = weights.no_escape

        output.shared_traffic_penalty = (
            output.same_direction_penalty + output.opposite_direction_penalty
            + output.corridor_occupancy_penalty)
        output.total_policy_penalty = (
            output.static_penalty + output.shared_traffic_penalty
            + output.no_escape_penalty)

        hard_enabled = mode in {PolicyMode.HYBRID, PolicyMode.HYBRID_NEGO}
        occupant_exit = candidate.is_exit or not candidate.is_entry
        if (
            hard_enabled
            and definition.hard_opposite_direction_block
            and not definition.passing_allowed
            and definition.capacity <= 1
            and candidate.is_entry
            and not occupant_exit
            and blocking_intervals
        ):
            output.hard_block = True
            output.decision = "HARD_CORRIDOR_BLOCK"
            output.reason_code = "OPPOSITE_DIRECTION_CORRIDOR_BLOCK"
            output.blockers = sorted({item.participant_id for item in blocking_intervals})
            owner = min(
                blocking_intervals,
                key=lambda item: (
                    0 if item.state in {CorridorState.OCCUPIED, CorridorState.UNKNOWN_HOLD} else 1,
                    item.enter_time_s,
                    item.participant_id,
                ),
            )
            output.reason = (
                f"New {candidate.direction.value} entry overlaps the admitted "
                f"{owner.direction.value} use by participant {owner.participant_id}; "
                "the non-passing corridor admission rule rejects this child"
            )
        elif output.total_policy_penalty > 0.0:
            output.decision = "SOFT_PENALIZED"
            output.reason_code = "SCHEDULE_AWARE_CORRIDOR_COST"
            output.reason = (
                f"{len(output.schedule_overlaps)} schedule interval(s) overlap; "
                f"same={same_overlap_s:.3f}s, opposite={opposite_overlap_s:.3f}s"
            )

        # Policy affects ranking only.  It never changes predicted timestamps or h.
        output.final_g = candidate.parent_g + base_physical + output.total_policy_penalty
        output.f = output.final_g + candidate.h
        return output


@dataclass
class Reservation:
    participant_id: int
    direction: CorridorDirection
    reserved_at_s: float
    predicted_enter_s: float
    predicted_exit_s: float
    priority: int = 0


@dataclass
class CorridorRuntime:
    definition: CorridorDefinition
    state: CorridorState = CorridorState.FREE
    owner_direction: CorridorDirection = CorridorDirection.UNKNOWN
    owner: int | None = None
    occupants: set[int] = field(default_factory=set)
    reserved: dict[int, Reservation] = field(default_factory=dict)
    unknown_participants: set[int] = field(default_factory=set)
    last_update_s: float = 0.0
    source: str = SOURCE_SIMULATION

    @staticmethod
    def _reservation_key(reservation: Reservation) -> tuple:
        # Higher task priority first, then first reservation, then participant ID.
        return (-reservation.priority, reservation.reserved_at_s, reservation.participant_id)

    def request_admission(self, reservation: Reservation) -> bool:
        if self.state is CorridorState.FREE:
            self.owner = reservation.participant_id
            self.owner_direction = reservation.direction
            self.reserved[reservation.participant_id] = reservation
            self.state = CorridorState.RESERVED
            self.last_update_s = reservation.reserved_at_s
            return True
        if reservation.direction is self.owner_direction:
            if len(self.occupants) + len(self.reserved) < self.definition.capacity:
                self.reserved[reservation.participant_id] = reservation
                self.last_update_s = reservation.reserved_at_s
                return True
            return False
        # Existing occupants always win.  Otherwise use one deterministic order.
        if self.occupants or self.unknown_participants:
            return False
        current = min(self.reserved.values(), key=self._reservation_key)
        if self._reservation_key(reservation) < self._reservation_key(current):
            self.reserved.clear()
            self.reserved[reservation.participant_id] = reservation
            self.owner = reservation.participant_id
            self.owner_direction = reservation.direction
            self.last_update_s = reservation.reserved_at_s
            return True
        return False

    def enter(self, participant_id: int, now_s: float) -> None:
        reservation = self.reserved.pop(participant_id, None)
        if reservation is None and participant_id != self.owner:
            raise ValueError("participant has no corridor admission")
        self.occupants.add(participant_id)
        self.state = CorridorState.OCCUPIED
        self.last_update_s = now_s

    def communication_lost(self, participant_id: int, now_s: float) -> None:
        if participant_id in self.occupants:
            self.unknown_participants.add(participant_id)
            self.state = CorridorState.UNKNOWN_HOLD
            self.last_update_s = now_s

    def exit(self, participant_id: int, now_s: float, *, checkpoint_confirmed: bool) -> bool:
        if participant_id not in self.occupants:
            return False
        if not checkpoint_confirmed:
            return False
        self.occupants.discard(participant_id)
        self.unknown_participants.discard(participant_id)
        self.reserved.pop(participant_id, None)
        self.last_update_s = now_s
        if self.occupants:
            self.state = (
                CorridorState.UNKNOWN_HOLD
                if self.unknown_participants else CorridorState.OCCUPIED)
            return False
        if self.reserved:
            winner = min(self.reserved.values(), key=self._reservation_key)
            self.owner = winner.participant_id
            self.owner_direction = winner.direction
            self.state = CorridorState.RESERVED
            return False
        self.owner = None
        self.owner_direction = CorridorDirection.UNKNOWN
        self.state = CorridorState.FREE
        return True


def write_snapshot(
    path: Path,
    definitions: Iterable[CorridorDefinition],
    intervals: Iterable[OccupancyInterval],
    weights: PolicyWeights,
    *,
    mode: PolicyMode,
    schedule_version: int,
    participant_id: int,
) -> None:
    """Write the dependency-free TSV schema consumed by the core patch."""
    lines = [
        "META\t3\t{}\t{}\t{}".format(
            mode.value, int(schedule_version), int(participant_id)),
        "WEIGHTS\t{:.12g}\t{:.12g}\t{:.12g}\t{:.12g}\t{:.12g}\t{:.12g}\t{:.12g}".format(
            weights.same_direction_per_second,
            weights.opposite_direction_per_second,
            weights.occupied_per_second,
            weights.future_reservation_per_second,
            weights.no_escape,
            weights.static,
            weights.overlap_margin_s,
        ),
    ]
    for definition in definitions:
        lines.append("CORRIDOR\t{}\t{}\t{}\t{}\t{:.12g}".format(
            definition.corridor_id,
            definition.capacity,
            int(definition.passing_allowed),
            int(definition.hard_opposite_direction_block),
            definition.base_penalty,
        ))
        for lane in definition.lanes_forward:
            lines.append(f"LANE\t{lane}\t{definition.corridor_id}\tA_TO_B")
        for lane in definition.lanes_reverse:
            lines.append(f"LANE\t{lane}\t{definition.corridor_id}\tB_TO_A")
    for interval in intervals:
        lines.append(
            "INTERVAL\t{}\t{}\t{}\t{}\t{}\t{:.12g}\t{:.12g}\t{}\t{}\t{}\t{}\t{}".format(
                interval.corridor_id,
                interval.participant_id,
                interval.plan_id,
                interval.route_id,
                interval.direction.value,
                interval.enter_time_s,
                interval.exit_time_s,
                interval.state.value,
                int(interval.owner),
                int(interval.responsive),
                interval.itinerary_version,
                interval.source,
            )
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
