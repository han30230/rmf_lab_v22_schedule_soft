from __future__ import annotations

import unittest

from tools.traffic_policy import (
    CandidateTraversal,
    CorridorDefinition,
    CorridorDirection,
    CorridorIntervalIndex,
    CorridorRuntime,
    CorridorState,
    OccupancyInterval,
    PolicyMode,
    PolicyWeights,
    Reservation,
)


def corridor(*, passing: bool = False, capacity: int = 1) -> CorridorDefinition:
    return CorridorDefinition(
        "C1", (10, 11, 12), (13, 14, 15), capacity,
        passing, True, 0, 6, 0.0)


def interval(
    participant: int,
    direction: CorridorDirection,
    enter: float,
    exit: float,
    state: CorridorState = CorridorState.RESERVED,
    *,
    responsive: bool = True,
    owner: bool = True,
) -> OccupancyInterval:
    return OccupancyInterval(
        "C1", participant, participant + 100, 0, direction,
        enter, exit, state, owner,
        responsive)


def candidate(
    participant: int,
    direction: CorridorDirection,
    enter: float,
    exit: float,
    **kwargs,
) -> CandidateTraversal:
    return CandidateTraversal(
        participant, "C1", direction, enter, exit,
        base_move_cost=exit - enter, h=3.0, **kwargs)


class TrafficPolicyScenarios(unittest.TestCase):
    def setUp(self) -> None:
        self.weights = PolicyWeights(overlap_margin_s=0.0)

    def evaluate(self, intervals, request, mode=PolicyMode.HYBRID):
        return CorridorIntervalIndex([corridor()], intervals).evaluate(
            request, mode, self.weights)

    def test_s1_one_vs_one_opposite_direction_blocks_new_entry(self) -> None:
        result = self.evaluate(
            [interval(1, CorridorDirection.A_TO_B, 10, 25)],
            candidate(2, CorridorDirection.B_TO_A, 15, 30),
        )
        self.assertTrue(result.hard_block)
        self.assertEqual(result.reason_code, "OPPOSITE_DIRECTION_CORRIDOR_BLOCK")
        self.assertEqual(result.blockers, [1])

    def test_s2_convoy_same_direction_allowed_opposite_waits(self) -> None:
        schedule = [
            interval(1, CorridorDirection.A_TO_B, 10, 25),
            interval(2, CorridorDirection.A_TO_B, 12, 27),
        ]
        convoy = self.evaluate(
            schedule, candidate(3, CorridorDirection.A_TO_B, 14, 29))
        opposing = self.evaluate(
            schedule, candidate(4, CorridorDirection.B_TO_A, 14, 29))
        self.assertFalse(convoy.hard_block)
        self.assertGreater(convoy.same_direction_penalty, 0)
        self.assertTrue(opposing.hard_block)

    def test_s3_deterministic_owner_prevents_symmetric_deadlock(self) -> None:
        runtime = CorridorRuntime(corridor())
        right = Reservation(2, CorridorDirection.B_TO_A, 0.0, 5.0, 15.0)
        left = Reservation(1, CorridorDirection.A_TO_B, 0.0, 5.0, 15.0)
        self.assertTrue(runtime.request_admission(right))
        self.assertTrue(runtime.request_admission(left))
        self.assertEqual(runtime.owner, 1)
        self.assertEqual(runtime.owner_direction, CorridorDirection.A_TO_B)

    def test_s4_detour_can_win_on_soft_cost(self) -> None:
        result = self.evaluate(
            [interval(1, CorridorDirection.A_TO_B, 10, 25)],
            candidate(2, CorridorDirection.B_TO_A, 12, 20),
            PolicyMode.SOFT,
        )
        self.assertFalse(result.hard_block)
        self.assertGreater(result.total_policy_penalty, 0)
        self.assertEqual(result.decision, "SOFT_PENALIZED")

    def test_s5_delay_shift_increases_overlap(self) -> None:
        request = candidate(2, CorridorDirection.B_TO_A, 20, 32)
        original = self.evaluate(
            [interval(1, CorridorDirection.A_TO_B, 10, 25)], request,
            PolicyMode.SOFT)
        delayed = self.evaluate(
            [interval(1, CorridorDirection.A_TO_B, 15, 30)], request,
            PolicyMode.SOFT)
        self.assertGreater(
            delayed.opposite_direction_penalty,
            original.opposite_direction_penalty)

    def test_s6_unknown_robot_keeps_ownership(self) -> None:
        runtime = CorridorRuntime(corridor())
        request = Reservation(1, CorridorDirection.A_TO_B, 0, 1, 10)
        self.assertTrue(runtime.request_admission(request))
        runtime.enter(1, 1.0)
        runtime.communication_lost(1, 4.0)
        self.assertEqual(runtime.state, CorridorState.UNKNOWN_HOLD)
        self.assertFalse(runtime.request_admission(
            Reservation(2, CorridorDirection.B_TO_A, 5, 6, 16)))

    def test_s7_release_requires_confirmed_exit_checkpoint(self) -> None:
        runtime = CorridorRuntime(corridor())
        runtime.request_admission(
            Reservation(1, CorridorDirection.A_TO_B, 0, 1, 10))
        runtime.enter(1, 1)
        self.assertFalse(runtime.exit(1, 10, checkpoint_confirmed=False))
        self.assertEqual(runtime.state, CorridorState.OCCUPIED)
        self.assertTrue(runtime.exit(1, 11, checkpoint_confirmed=True))
        self.assertEqual(runtime.state, CorridorState.FREE)

    def test_s8_soft_penalty_never_removes_all_paths(self) -> None:
        result = self.evaluate(
            [interval(1, CorridorDirection.A_TO_B, 0, 100)],
            candidate(2, CorridorDirection.B_TO_A, 10, 20),
            PolicyMode.SOFT,
        )
        self.assertFalse(result.hard_block)
        self.assertTrue(result.final_g < float("inf"))

    def test_s9_hard_policy_disabled_matches_soft_acceptance(self) -> None:
        schedule = [interval(1, CorridorDirection.A_TO_B, 10, 25)]
        request = candidate(2, CorridorDirection.B_TO_A, 15, 30)
        soft = self.evaluate(schedule, request, PolicyMode.SOFT)
        hybrid = self.evaluate(schedule, request, PolicyMode.HYBRID)
        self.assertFalse(soft.hard_block)
        self.assertTrue(hybrid.hard_block)

    def test_s10_zero_weights_match_baseline_g_h_f(self) -> None:
        zero = PolicyWeights()
        zero = PolicyWeights(
            same_direction_per_second=0,
            opposite_direction_per_second=0,
            occupied_per_second=0,
            future_reservation_per_second=0,
            no_escape=0,
            static=0,
            overlap_margin_s=0,
        )
        request = candidate(2, CorridorDirection.B_TO_A, 15, 30)
        index = CorridorIntervalIndex(
            [corridor()], [interval(1, CorridorDirection.A_TO_B, 10, 25)])
        baseline = index.evaluate(request, PolicyMode.BASELINE, zero)
        soft = index.evaluate(request, PolicyMode.SOFT, zero)
        self.assertEqual(soft.final_g, baseline.final_g)
        self.assertEqual(soft.h, baseline.h)
        self.assertEqual(soft.f, baseline.f)

    def test_existing_occupant_exit_is_never_blocked(self) -> None:
        result = self.evaluate(
            [interval(2, CorridorDirection.B_TO_A, 10, 30)],
            candidate(
                1, CorridorDirection.A_TO_B, 15, 20,
                is_entry=False, is_exit=True),
        )
        self.assertFalse(result.hard_block)

    def test_unowned_opposite_reservation_does_not_symmetrically_block(self) -> None:
        result = self.evaluate(
            [interval(
                2, CorridorDirection.B_TO_A, 10, 30,
                CorridorState.RESERVED, owner=False)],
            candidate(1, CorridorDirection.A_TO_B, 15, 20),
        )
        self.assertFalse(result.hard_block)

    def test_margin_is_not_reported_or_charged_as_actual_overlap(self) -> None:
        weights = PolicyWeights(overlap_margin_s=1.0)
        request = candidate(2, CorridorDirection.B_TO_A, 10.5, 20.0)
        index = CorridorIntervalIndex(
            [corridor()],
            [interval(1, CorridorDirection.A_TO_B, 0.0, 10.0)],
        )
        result = index.evaluate(request, PolicyMode.HYBRID, weights)
        self.assertEqual(result.schedule_overlaps[0].overlap_duration_s, 0.0)
        self.assertGreater(
            result.schedule_overlaps[0].admission_overlap_duration_s, 0.0)
        self.assertEqual(result.opposite_direction_penalty, 0.0)
        self.assertTrue(result.hard_block)

    def test_multi_lane_corridor_charges_only_once(self) -> None:
        schedule = [interval(1, CorridorDirection.A_TO_B, 10, 25)]
        first = self.evaluate(
            schedule,
            candidate(2, CorridorDirection.A_TO_B, 12, 18),
            PolicyMode.SOFT)
        continued = self.evaluate(
            schedule,
            candidate(
                2, CorridorDirection.A_TO_B, 18, 22,
                already_counted_corridors=frozenset({"C1"})),
            PolicyMode.SOFT)
        self.assertGreater(first.total_policy_penalty, 0)
        self.assertEqual(continued.total_policy_penalty, 0)

    def test_h_is_never_modified_by_policy(self) -> None:
        request = candidate(2, CorridorDirection.B_TO_A, 15, 30)
        result = self.evaluate(
            [interval(1, CorridorDirection.A_TO_B, 10, 25)], request,
            PolicyMode.SOFT)
        self.assertEqual(result.h, request.h)
        self.assertEqual(result.f, result.final_g + request.h)


if __name__ == "__main__":
    unittest.main()
