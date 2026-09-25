#!/usr/bin/env python3
"""Topology-driven action horizon assembly.

This layer owns graph traversal and the virtual DFS overlay. MotionPlanner is
kept as a geometry-only compiler for one explicit ``(prev, cell, next)`` triple.
"""

from .motion_planner import MotionPlanner, GRAB_V, validate_geometry
from .motion_primitive import MotionPrimitive
from .pose import Pose2D


class ActionHorizon:
    def __init__(self, motion_planner, max_steps=64):
        if not isinstance(motion_planner, MotionPlanner):
            raise TypeError("motion_planner must be a MotionPlanner")
        if not isinstance(max_steps, int) or max_steps < 1:
            raise ValueError("max_steps must be a positive integer")
        self.motion_planner = motion_planner
        self.max_steps = max_steps

    def compile(self, nav, pose, cursor):
        """Build primitives from graph transitions within a bounded horizon.

        Returns ``(primitives, terminal_cursor, planned_cell_sequence)``.
        The overlay tracks only simulated branch descents and returns; committed
        navigation state remains owned by ``nav``.
        """
        planner = self.motion_planner
        prims = []
        cur = pose.copy()
        prev, cell = cursor
        seq = [cell]
        plan_state = {}

        for _ in range(self.max_steps):
            is_branch = nav.is_exploration_branch(cell)
            if is_branch:
                parent = None if cell == nav.entry else prev
                state = nav.plan_branch_enter(plan_state, cell, parent)
                active_child = state.get('active_child')
                if prev is not None and prev == active_child:
                    nav.plan_branch_return(plan_state, cell, prev)
                elif (prev is not None and state.get('committed_snapshot') and
                      prev in state.get('children', ()) and prev not in state.get('done', ())):
                    nav.plan_branch_return(plan_state, cell, prev, allow_pending=True)

            nxt = nav.resolve_next(prev, cell, plan_state)
            if nxt is None:
                if prims and prims[-1].kind in ('STRAIGHT', 'ARC'):
                    prims[-1].v_end = 0.0
                prims.append(MotionPrimitive(
                    kind='STOP', start_pose=Pose2D(cur.x, cur.y, cur.yaw),
                    p0=(cur.x, cur.y), yaw0=cur.yaw, duration=0.2,
                    meta={'wait': cell}))
                validate_geometry(prims, cursor=(prev, cell))
                return prims, (prev, cell), seq

            if is_branch:
                state = plan_state[cell]
                if nxt != state['parent_cell']:
                    nav.plan_branch_descend(plan_state, cell, nxt)

            is_dead_end_reverse = prev is not None and nxt == prev
            if is_dead_end_reverse and prims and prims[-1].kind in ('STRAIGHT', 'ARC'):
                # Enter the dead-end retreat template slowly enough to stop
                # within its fixed center-to-edge segment before reversing.
                prims[-1].v_end = GRAB_V
            step, end_pt, end_cursor = planner.template(prev, cell, nxt, cur)
            prims += step
            cur = Pose2D(end_pt[0], end_pt[1], cur.yaw)
            seq.append(nxt)
            prev, cell = end_cursor

        for i in range(len(prims) - 1):
            if prims[i + 1].kind == 'ARC' and prims[i].kind == 'STRAIGHT':
                prims[i].v_end = planner.v_arc
        validate_geometry(prims, cursor=(prev, cell))
        return prims, (prev, cell), seq
