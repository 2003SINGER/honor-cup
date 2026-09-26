#!/usr/bin/env python3
"""SemanticSim Gate —— 每 seed 硬断言 (G1 验收, 不看平均值).

用法: python3 run_semantic_gate.py [--start 0] [--seeds 100] [--blocks]
                              [--json 失败现场路径]

每 seed 断言:
  not aborted            (watchdog/无出口 = FAILED)
  violations == 0        (几何/状态机层面零碰撞)
  topology_mismatch == 0 (事件与计划格序列一致)
  false_prune == 0       (剪掉的格不含真值方块)
  unresolved == 0        (仅 fullinfo: 每条 internal edge 都有答案)
  wrong_edges == 0       (每条答案都与真值一致)
  got == 8               (仅含块模式: 全部收集)

任一失败: 保存可重放 seed + 错误统计或异常 → exit 1.
"""
import argparse
import json
import random
import sys
import os
import time
import threading
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'ros2', 'm3pro_nav'))
import runtime_v2 as R
import maze_sim as M


def run_seed(seed, blocks_on):
    walls, entry, ex, side = M.gen_maze(seed)
    rnd = random.Random(1000 + seed)
    random.seed(seed)
    if blocks_on:
        cells = [(i, j) for i in range(R.N) for j in range(R.N)
                 if (i, j) not in (entry, ex)]
        blocks = set(rnd.sample(cells, 8))
    else:
        blocks = set()
    st = R.explore(walls, entry, 'LFR', blocks,
                   required_blocks=8 if blocks_on else 0, v_cruise=0.7)
    st['seed'] = seed
    st['_truth_exit'] = list(ex)
    st['_truth_exit_walls'] = sorted(walls[ex])
    return st


def check(st, blocks_on):
    """fullinfo: 全图探索, unresolved 必须为 0.
    blocks: 收齐 8 块即合法收工 (未探区域是任务语义, 不算失败),
            但已下结论的边仍必须全对 (wrong_edges==0)."""
    errs = []
    if st.get('aborted'):
        errs.append(f"aborted=True (watchdog/无出口候选)")
    if st.get('violations', 0) != 0:
        errs.append(f"violations={st['violations']}")
    if st.get('topology_mismatch', -1) != 0:
        errs.append(f"topology_mismatch={st.get('topology_mismatch', 'missing')}")
    if st.get('false_prune', -1) != 0:
        errs.append(f"false_prune={st.get('false_prune', 'missing')}")
    if not blocks_on and st.get('unresolved', -1) != 0:
        errs.append(f"unresolved={st['unresolved']}")
    if st.get('wrong_edges', -1) != 0:
        errs.append(f"wrong_edges={st['wrong_edges']} (认知地图与真值不一致)")
    if blocks_on and st.get('got', 0) != 8:
        errs.append(f"got={st['got']}/8")
    return errs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--start', type=int, default=0, help='起始 seed (含)')
    ap.add_argument('--seeds', type=int, default=100)
    ap.add_argument('--blocks', action='store_true', help='8 方块任务模式')
    ap.add_argument('--json', default=os.path.join(
        tempfile.gettempdir(), 'honor-cup-gate-failures.jsonl'),
        help='失败 fixture 路径 (默认写入系统临时目录)')
    args = ap.parse_args()
    if args.start < 0 or args.seeds <= 0:
        ap.error('--start must be >= 0 and --seeds must be > 0')

    def watchdog():
        time.sleep(3600 * 4)
        print("GATE GLOBAL TIMEOUT", flush=True)
        import os
        os._exit(3)
    threading.Thread(target=watchdog, daemon=True).start()

    t_all = time.time()
    times = []
    saved_cells = []
    saved_distance = []
    for seed in range(args.start, args.start + args.seeds):
        t0 = time.time()
        try:
            st = run_seed(seed, args.blocks)
            errs = check(st, args.blocks)
        except Exception as exc:
            st = {'seed': seed, 'aborted': True,
                  'exception_type': type(exc).__name__, 'exception': str(exc)}
            errs = [f"{type(exc).__name__}: {exc}"]
        if errs:
            fixture = {k: (round(v, 3) if isinstance(v, float) else v)
                       for k, v in st.items() if k not in ('nav',)}
            fixture['errors'] = errs
            with open(args.json, 'a') as f:
                f.write(json.dumps(fixture, ensure_ascii=False) + '\n')
            print(f"FAIL seed{seed}: {errs} → 已保存 {args.json}", flush=True)
            sys.exit(1)
        times.append(st['time'])
        saved_cells.append(st['pruned_cells_saved'])
        saved_distance.append(st['pruned_distance_saved'])
        print(f"seed{seed}: PASS t={st['time']:.1f}s d={st['dist']:.1f}m "
              f"enters={st['enters']} arcs={st['arcs']} "
              f"saved={st['pruned_cells_saved']} cells/{st['pruned_distance_saved']:.1f}m "
              f"({time.time()-t0:.1f}s cpu)", flush=True)
    n = len(times)
    print(f"\nGATE PASS: {n}/{n} seeds, avg {sum(times)/n:.1f}s "
          f"min {min(times):.1f} max {max(times):.1f}, "
          f"avg saved {sum(saved_cells)/n:.2f} cells/"
          f"{sum(saved_distance)/n:.2f}m, "
          f"总 CPU {time.time()-t_all:.0f}s", flush=True)


if __name__ == '__main__':
    main()
