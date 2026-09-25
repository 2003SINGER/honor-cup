"""Focused contracts for pure branch preview and virtual DFS progress."""

from m3pro_nav.stream_nav import StreamNav


def _classified_branch(nav, cell=(2, 2)):
    # Parent is south; branch children are north and east. West is a wall.
    for _ in range(2):
        for direction in ('N', 'E', 'S', 'W'):
            if direction == 'W':
                nav.edges.observe_wall(cell, direction, 0.2)
            else:
                nav.edges.observe_open(cell, direction, 0.2)


def test_branch_preview_is_pure_and_parent_stays_frozen():
    nav = StreamNav((2, 0), n=7, order='LFR')
    branch = (2, 2)
    parent = (2, 1)
    north = (2, 3)
    east = (3, 2)
    _classified_branch(nav, branch)
    nav._commit_branch(branch, parent)
    committed = nav.branch[branch]
    committed_before = (committed['parent_cell'], tuple(committed['children']),
                        frozenset(committed['done']))

    overlay = {}
    nav.plan_branch_enter(overlay, branch, parent)
    before = {k: (v['parent_cell'], v['incoming'], tuple(v['children']),
                  frozenset(v['done']), v['active_child'])
              for k, v in overlay.items()}
    for _ in range(4):
        assert nav.resolve_next(parent, branch, overlay) == north
    after = {k: (v['parent_cell'], v['incoming'], tuple(v['children']),
                 frozenset(v['done']), v['active_child'])
             for k, v in overlay.items()}

    assert before == after
    assert committed_before == (committed['parent_cell'], tuple(committed['children']),
                                frozenset(committed['done']))
    assert committed['parent_cell'] == parent
    assert east in committed['children']


def test_virtual_child_completes_only_after_return_transition():
    nav = StreamNav((2, 0), n=7, order='LFR')
    branch = (2, 2)
    parent = (2, 1)
    north = (2, 3)
    east = (3, 2)
    _classified_branch(nav, branch)
    nav._commit_branch(branch, parent)
    committed = nav.branch[branch]
    overlay = {}
    nav.plan_branch_enter(overlay, branch, parent)

    assert nav.resolve_next(parent, branch, overlay) == north
    nav.plan_branch_descend(overlay, branch, north)
    # Merely selecting/descending into a child must not virtually complete it.
    assert nav.resolve_next(north, branch, overlay) == north
    assert overlay[branch]['done'] == set()

    nav.plan_branch_return(overlay, branch, north)
    assert nav.resolve_next(north, branch, overlay) == east
    assert overlay[branch]['done'] == {north}
    assert overlay[branch]['parent_cell'] == parent
    assert overlay[branch]['active_child'] is None
    assert committed['done'] == set()
    assert committed['parent_cell'] == parent


def test_virtual_parent_is_not_reinferred_from_return_direction():
    nav = StreamNav((2, 0), n=7, order='LFR')
    branch = (2, 2)
    parent = (2, 1)
    north = (2, 3)
    _classified_branch(nav, branch)
    overlay = {}
    nav.plan_branch_enter(overlay, branch, parent)
    nav.plan_branch_descend(overlay, branch, north)
    nav.plan_branch_return(overlay, branch, north)
    east = (3, 2)
    nav.plan_branch_descend(overlay, branch, east)
    nav.plan_branch_return(overlay, branch, east)

    # Exhausting children routes to the original parent even when the current
    # call's prev_cell is the just-completed child.
    assert nav.resolve_next(north, branch, overlay) == parent
    assert overlay[branch]['parent_cell'] == parent


def test_nested_branch_overlay_keeps_each_committed_parent_and_completion_local():
    """Preview an inner branch, return from it, then resume the outer sibling."""
    nav = StreamNav((2, 0), n=7, order='LFR')
    outer, outer_parent = (2, 2), (2, 1)
    inner = (2, 3)
    outer_sibling = (3, 2)

    # Outer children: north (the nested branch), east (its sibling).
    _classified_branch(nav, outer)
    # Inner children: north and west; south leads back to the outer branch.
    for _ in range(2):
        for direction in ('N', 'E', 'S', 'W'):
            observer = nav.edges.observe_wall if direction == 'E' else nav.edges.observe_open
            observer(inner, direction, 0.2)

    # These events are the only operations that commit persistent parents.
    nav.on_entered(outer, outer_parent)
    nav.on_entered(inner, outer)
    outer_state, inner_state = nav.branch[outer], nav.branch[inner]
    assert outer_state['parent_cell'] == outer_parent
    assert inner_state['parent_cell'] == outer

    overlay = {}
    nav.plan_branch_enter(overlay, outer, outer_parent)
    assert nav.resolve_next(outer_parent, outer, overlay) == inner
    nav.plan_branch_descend(overlay, outer, inner)

    nav.plan_branch_enter(overlay, inner, outer)
    inner_first = nav.resolve_next(outer, inner, overlay)
    assert inner_first in inner_state['children']
    nav.plan_branch_descend(overlay, inner, inner_first)
    nav.plan_branch_return(overlay, inner, inner_first)
    inner_second = nav.resolve_next(inner_first, inner, overlay)
    assert inner_second in set(inner_state['children']) - {inner_first}
    nav.plan_branch_descend(overlay, inner, inner_second)
    nav.plan_branch_return(overlay, inner, inner_second)

    # Once the inner children finish, the overlay returns to its fixed parent;
    # only then is the nested branch complete in the outer overlay.
    assert nav.resolve_next(inner_second, inner, overlay) == outer
    nav.plan_branch_return(overlay, outer, inner)
    assert nav.resolve_next(inner, outer, overlay) == outer_sibling

    assert overlay[outer]['done'] == {inner}
    assert overlay[inner]['done'] == {inner_first, inner_second}
    assert outer_state['done'] == set()
    assert inner_state['done'] == set()
    assert outer_state['parent_cell'] == outer_parent
    assert inner_state['parent_cell'] == outer


def test_new_horizon_can_resume_pending_committed_child_return():
    nav = StreamNav((2, 0), n=7, order='LFR')
    branch = (2, 2)
    parent = (2, 1)
    north = (2, 3)
    east = (3, 2)
    _classified_branch(nav, branch)
    nav._commit_branch(branch, parent)

    # A fresh ActionHorizon overlay can begin at the boundary cursor after the
    # child path, before EnteredCell has committed the return to BranchState.
    overlay = {}
    nav.plan_branch_enter(overlay, branch, parent)
    try:
        nav.plan_branch_return(overlay, branch, north)
    except ValueError:
        pass
    else:
        raise AssertionError('pending return must require explicit resume permission')

    nav.plan_branch_return(overlay, branch, north, allow_pending=True)
    assert nav.resolve_next(north, branch, overlay) == east
    assert overlay[branch]['done'] == {north}
    assert nav.branch[branch]['done'] == set()


def _set_complete_cell(nav, cell, open_dirs, boundary_open=()):
    for direction in ('N', 'E', 'S', 'W'):
        if nav.is_boundary(cell, direction):
            nav.edges.set_boundary(
                cell, direction, 'OPEN' if direction in boundary_open else 'WALL')
        else:
            observe = (nav.edges.observe_open if direction in open_dirs
                       else nav.edges.observe_wall)
            observe(cell, direction, 0.2)
            observe(cell, direction, 0.2)


def test_single_child_entry_root_terminates_on_return():
    root = (0, 0)
    child = (1, 0)
    nav = StreamNav(root, n=7)
    _set_complete_cell(nav, root, {'E'}, boundary_open={'S'})

    assert nav.mark(root)['kind'] == 'WAY'
    assert not nav.is_exploration_branch(root)
    assert nav.resolve_next(None, root) == child
    assert nav.resolve_next(child, root) is None


def test_multi_child_entry_root_selects_unfinished_child_on_return():
    root = (0, 0)
    north = (0, 1)
    east = (1, 0)
    nav = StreamNav(root, n=7, order='LFR')
    _set_complete_cell(nav, root, {'N', 'E'}, boundary_open={'S'})
    nav._commit_branch(root, None)
    nav.branch[root]['done'].add(north)

    assert nav.is_exploration_branch(root)
    assert nav.resolve_next(north, root) == east
    assert nav.branch[root]['parent_cell'] is None


def test_boundary_exit_with_two_interior_neighbors_is_not_a_branch():
    exit_cell = (2, 0)
    north = (2, 1)
    east = (3, 0)
    nav = StreamNav((0, 0), n=7)
    _set_complete_cell(nav, exit_cell, {'N', 'E'}, boundary_open={'S'})

    assert nav.mark(exit_cell)['kind'] == 'BRANCH'
    assert not nav.is_exploration_branch(exit_cell)
    assert nav.resolve_next(north, exit_cell) == east
    nav.on_entered(exit_cell, north)
    assert exit_cell not in nav.branch


def test_home_route_uses_confirmed_open_edges_before_they_are_walked():
    nav = StreamNav((0, 0), n=3)
    current = (1, 0)
    exit_cell = (2, 0)
    for _ in range(2):
        nav.edges.observe_open((0, 0), 'E', 0.2)
        nav.edges.observe_open(current, 'E', 0.2)
    nav.edges.set_boundary(exit_cell, 'E', 'OPEN')
    nav.edges.set_boundary(exit_cell, 'S', 'WALL')

    assert not nav.traversal.is_walked(current, 'E')
    assert nav.route_cells(current, exit_cell) == [current, exit_cell]
    assert nav.home_route(current) == ([current, exit_cell], 'E')


def test_home_route_does_not_assume_unknown_edges_are_open():
    nav = StreamNav((0, 0), n=3)
    nav.edges.set_boundary((2, 0), 'E', 'OPEN')
    nav.edges.set_boundary((2, 0), 'S', 'WALL')
    nav.edges.observe_open((0, 0), 'E', 0.2)
    nav.edges.observe_open((0, 0), 'E', 0.2)

    assert nav.route_cells((0, 0), (2, 0)) is None
