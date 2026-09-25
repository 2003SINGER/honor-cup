#!/usr/bin/env python3
"""SemanticSim Gate —— 每 seed 硬断言 (G1 验收, 不看平均值).

用法: python3 run_semantic_gate.py [--seeds 100] [--blocks] [--json 失败现场路径]

每 seed 断言:
  not aborted            (watchdog/无出口 = FAILED)
  violations == 0        (几何/状态机层面零碰撞)
  unresolved == 0        (每条 internal edge 都有答案)
  wrong_edges == 0       (每条答案都与真值一致)
  got == len(blocks)     (含块模式: 全部收集)

任一失败: 保存 seed + 完整状态 fixture → exit 1.
"""
import argparse
import json
import random
import sys
import os
import time
import threading

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
    st = R.explore(walls, entry, ex, 'LFR', blocks, v_cruise=0.7)
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
    if not blocks_on and st.get('unresolved', -1) != 0:
        errs.append(f"unresolved={st['unresolved']}")
    if st.get('wrong_edges', -1) != 0:
        errs.append(f"wrong_edges={st['wrong_edges']} (认知地图与真值不一致)")
    if blocks_on and st.get('got', 0) != 8:
        errs.append(f"got={st['got']}/8")
    return errs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seeds', type=int, default=100)
    ap.add_argument('--blocks', action='store_true', help='8 方块任务模式')
    ap.add_argument('--json', default='gate_failures.json', help='失败 fixture 路径')
    args = ap.parse_args()

    def watchdog():
        time.sleep(3600 * 4)
        print("GATE GLOBAL TIMEOUT", flush=True)
        import os
        os._exit(3)
    threading.Thread(target=watchdog, daemon=True).start()

    t_all = time.time()
    times = []
    for seed in range(args.seeds):
        t0 = time.time()
        st = run_seed(seed, args.blocks)
        errs = check(st, args.blocks)
        if errs:
            fixture = {k: (round(v, 3) if isinstance(v, float) else v)
                       for k, v in st.items() if k not in ('nav',)}
            fixture['errors'] = errs
            with open(args.json, 'a') as f:
                f.write(json.dumps(fixture, ensure_ascii=False) + '\n')
            print(f"FAIL seed{seed}: {errs} → 已保存 {args.json}", flush=True)
            sys.exit(1)
        times.append(st['time'])
        print(f"seed{seed}: PASS t={st['time']:.1f}s d={st['dist']:.1f}m "
              f"enters={st['enters']} arcs={st['arcs']} "
              f"({time.time()-t0:.1f}s cpu)", flush=True)
    n = len(times)
    print(f"\nGATE PASS: {n}/{n} seeds, avg {sum(times)/n:.1f}s "
          f"min {min(times):.1f} max {max(times):.1f}, "
          f"总 CPU {time.time()-t_all:.0f}s", flush=True)


if __name__ == '__main__':
    main()
