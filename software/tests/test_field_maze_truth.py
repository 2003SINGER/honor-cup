"""Consistency checks for the user-supplied 7x7 field topology."""

import json
from pathlib import Path


TRUTH_PATH = (Path(__file__).resolve().parents[2]
              / 'field' / 'maze_truth_7x7.json')
DIRECTIONS = {
    'N': (0, 1, 'S'),
    'E': (1, 0, 'W'),
    'S': (0, -1, 'N'),
    'W': (-1, 0, 'E'),
}


def _load_truth():
    return json.loads(TRUTH_PATH.read_text(encoding='utf-8'))


def test_field_maze_truth_sources_agree_and_form_a_tree():
    truth = _load_truth()
    grid = truth['grid']
    width, height = grid['width'], grid['height']
    rows = truth['open_directions_by_cell']
    assert (width, height) == (7, 7)
    assert len(rows) == height
    assert all(len(row) == width for row in rows)

    opens = {(x, y): set(rows[y][x])
             for y in range(height) for x in range(width)}
    assert all(dirs <= DIRECTIONS.keys() for dirs in opens.values())

    ns_walls = {
        (x, int(y)) for y, xs in
        truth['interior_walls']['north_south_by_lower_row'].items()
        for x in xs
    }
    ew_walls = {
        (int(x), y) for x, ys in
        truth['interior_walls']['east_west_by_left_column'].items()
        for y in ys
    }

    # Check reciprocal cell directions and independently listed wall edges.
    adjacency = {cell: set() for cell in opens}
    open_edge_count = 0
    for (x, y), dirs in opens.items():
        for direction in ('N', 'E'):
            dx, dy, opposite = DIRECTIONS[direction]
            nx, ny = x + dx, y + dy
            if nx >= width or ny >= height:
                continue
            edge_open = direction in dirs
            assert edge_open == (opposite in opens[(nx, ny)]), (
                (x, y), direction, (nx, ny), opposite)
            wall_list = ns_walls if direction == 'N' else ew_walls
            edge_id = (x, y) if direction == 'N' else (x, y)
            assert ((edge_id not in wall_list) == edge_open), (
                (x, y), direction, 'cell directions disagree with wall list')
            if edge_open:
                adjacency[(x, y)].add((nx, ny))
                adjacency[(nx, ny)].add((x, y))
                open_edge_count += 1

    # Only the stated entrance and exit are open on the perimeter.
    perimeter = truth['perimeter']
    ports = {tuple(perimeter['entrance']), tuple(perimeter['exit'])}
    for (x, y), dirs in opens.items():
        for direction, (dx, dy, _opposite) in DIRECTIONS.items():
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height:
                continue
            assert ((x, y, direction) in ports) == (direction in dirs), (
                (x, y), direction, 'perimeter direction mismatch')
    assert perimeter['entrance'] == [3, 0, 'S']
    assert perimeter['exit'] == [0, 0, 'W']

    # The maze has 49 cells and 48 interior openings; verify connectivity too.
    assert open_edge_count == width * height - 1
    reached = set()
    pending = [(0, 0)]
    while pending:
        cell = pending.pop()
        if cell in reached:
            continue
        reached.add(cell)
        pending.extend(adjacency[cell] - reached)
    assert len(reached) == width * height

    assert len(set(map(tuple, truth['goals']))) == len(truth['goals'])
    assert all(0 <= x < width and 0 <= y < height
               for x, y in truth['goals'])

    # Previously walked facts agree in the abstract coordinate system.
    assert opens[(6, 0)] == {'N', 'W'}
    assert opens[(5, 0)] == {'E'}
    assert opens[(1, 2)] == {'N', 'E'}
    assert opens[(0, 4)] == {'S'}
    assert opens[(1, 4)] == {'S'}

    # Preserve the image/physical-view discrepancy instead of guessing a map.
    assert grid['coordinate_views']['physical_image_transform_resolved'] is False
