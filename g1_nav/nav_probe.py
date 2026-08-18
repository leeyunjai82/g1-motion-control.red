#!/usr/bin/env python3
"""
nav_probe.py — SLAM 서비스 점검 도구

순서대로 실행하세요.

  python3 nav_probe.py --iface enp46s0 ping      # 1. RPC 가 살아있나
  python3 nav_probe.py --iface enp46s0 raw       # 2. 토픽 원문 그대로 보기
  python3 nav_probe.py --iface enp46s0 map-start # 3. 매핑 (토픽이 여기서부터 나옴)
  python3 nav_probe.py --iface enp46s0 map-end
  python3 nav_probe.py --iface enp46s0 reloc     # 4. 측위 (이후 위치가 나옴)
  python3 nav_probe.py --iface enp46s0 goto --x 1.0
  python3 nav_probe.py --iface enp46s0 repeat --x 1.0 --n 5

중요: rt/slam_info 는 SLAM 노드가 돌아야 나옵니다.
매핑이나 측위를 시작하기 전에는 아무것도 안 옵니다.
"""

import argparse
import json
import math
import statistics
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

from slam_client import DEFAULT_PCD, Pose, SlamClient, TOPIC_INFO, TOPIC_KEY_INFO


def connect(iface, pcd):
    ChannelFactoryInitialize(0, iface)
    cli = SlamClient(pcd_path=pcd)
    cli.Init()
    time.sleep(1.0)
    return cli


# ---------------------------------------------------------------------------
def cmd_ping(cli, args):
    """가장 무해한 호출(1201 pause)로 RPC 왕복만 확인."""
    print("[ping] 1201 pause_nav 호출 — RPC 왕복 확인")
    t0 = time.time()
    code, data = cli.pause_nav()
    dt = time.time() - t0
    print(f"  code={code}  data={data!r}  ({dt*1000:.0f}ms)")
    print()
    if code == 0:
        print("  RPC 연결 OK. SLAM 서비스가 응답합니다.")
        print("  -> 다음: map-start 또는 reloc")
    elif dt > 4.0:
        print("  타임아웃입니다. 서비스에 도달하지 못했습니다.")
        print("   1) ping 192.168.123.18  /  ping 192.168.123.161")
        print(f"   2) --iface {args.iface} 가 123 대역 카드가 맞는지: ip -br a")
        print("   3) 로봇에서 SLAM 프로그램이 실행 중인지 (아래 '로봇 쪽 확인' 참고)")
    else:
        print(f"  서비스는 응답했지만 에러 코드입니다 (code={code}).")
        print("  SLAM 노드가 아직 안 떠 있어서 pause 할 대상이 없을 수 있습니다.")
        print("  -> map-start 또는 reloc 을 먼저 해보세요.")
    print()
    print("  로봇 쪽 확인:")
    print("    ssh unitree@192.168.123.18   (비밀번호 123)")
    print("    ROS 환경 활성화 물으면 Enter 로 거부")
    print("    ps aux | grep -i slam")


def cmd_raw(cli, args):
    """두 토픽의 원문 JSON 을 그대로 출력. 무엇이 오는지 눈으로 확인."""
    count = {"n": 0}

    def dump(tag):
        def h(msg):
            count["n"] += 1
            try:
                d = json.loads(msg.data)
                print(f"[{tag}] {json.dumps(d, ensure_ascii=False)[:300]}")
            except Exception:
                print(f"[{tag}] (파싱 실패) {msg.data[:200]!r}")
        return h

    s1 = ChannelSubscriber(TOPIC_INFO, String_); s1.Init(dump("slam_info"), 10)
    s2 = ChannelSubscriber(TOPIC_KEY_INFO, String_); s2.Init(dump("key_info"), 10)

    print(f"[raw] {args.sec}초 동안 원문 출력\n  {TOPIC_INFO}\n  {TOPIC_KEY_INFO}\n")
    time.sleep(args.sec)
    print(f"\n  받은 메시지: {count['n']}건")
    if count["n"] == 0:
        print("  아무것도 안 옵니다. SLAM 노드가 안 돌고 있을 가능성이 큽니다.")
        print("  -> map-start 를 걸고 다시 raw 를 실행해보세요.")


def cmd_map_start(cli, args):
    print(f"[map-start] {cli.start_mapping('indoor')}")
    print("  로봇을 수동 주행시켜 맵을 만드세요. 끝나면 map-end.")
    print("  이 시점부터 rt/slam_info 가 나와야 합니다 — 다른 터미널에서 raw 로 확인하세요.")


def cmd_map_end(cli, args):
    print(f"[map-end] {cli.end_mapping(args.pcd)}  -> {args.pcd}")


def cmd_reloc(cli, args):
    p = Pose.from_yaw(args.x, args.y, args.yaw)
    print(f"[reloc] {p}  address={args.pcd}")
    print(f"  {cli.start_relocation(p, args.pcd)}")
    for i in range(10):
        time.sleep(1.0)
        if cli.pose_is_fresh():
            print(f"  위치 수신 OK: {cli.get_pose()[0]}")
            return
        print(f"  대기 {i+1}/10 ...")
    print("  위치가 안 옵니다.")
    print(f"   - pcd 파일이 로봇에 있는지: ls -l {args.pcd}")
    print("   - 로봇이 맵 안의 그 좌표에 실제로 서 있는지")
    print(f"   - raw 로 에러 메시지 확인: nav_probe.py --iface {args.iface} raw")


def cmd_listen(cli, args):
    print(f"[listen] {args.sec}초간 위치 수신 확인")
    end = time.time() + args.sec
    while time.time() < end:
        if cli.pose_is_fresh():
            print(f"  OK  {cli.get_pose()[0]}")
        else:
            print("  없음")
        time.sleep(0.5)
    if not cli.pose_is_fresh():
        print("\n  SLAM 노드가 안 돌고 있을 가능성이 큽니다. ping / raw 로 확인하세요.")


# ---------------------------------------------------------------------------
def report(target, actual, sec):
    dyaw = math.degrees(math.atan2(math.sin(actual.yaw - target.yaw),
                                   math.cos(actual.yaw - target.yaw)))
    d = {"dx_cm": (actual.x-target.x)*100, "dy_cm": (actual.y-target.y)*100,
         "dist_cm": actual.distance_to(target)*100, "dyaw_deg": dyaw, "sec": sec}
    print(f"  오차 dx={d['dx_cm']:+.1f}cm dy={d['dy_cm']:+.1f}cm "
          f"dist={d['dist_cm']:.1f}cm dyaw={d['dyaw_deg']:+.1f}deg ({sec:.1f}s)")
    return d


def _goto(cli, target, timeout):
    t0 = time.time()
    ok, msg = cli.goto_and_wait(target, timeout=timeout)
    sec = time.time() - t0
    print(f"  {msg}")
    if not ok:
        return None
    time.sleep(1.5)
    return report(target, cli.get_pose()[0], sec)


def cmd_goto(cli, args):
    if not cli.pose_is_fresh():
        print("  측위가 안 됐습니다. reloc 을 먼저 하세요.")
        return
    target = Pose.from_yaw(args.x, args.y, args.yaw)
    print(f"[goto] {target}")
    _goto(cli, target, args.timeout)


def cmd_repeat(cli, args):
    if not cli.pose_is_fresh():
        print("  측위가 안 됐습니다. reloc 을 먼저 하세요.")
        return
    home = cli.get_pose()[0]
    target = Pose.from_yaw(args.x, args.y, args.yaw)
    print(f"[repeat] 홈={home}\n         목표={target}\n         {args.n}회 왕복\n")
    samples = []
    for i in range(args.n):
        print(f"--- {i+1}/{args.n} 목표로 ---")
        d = _goto(cli, target, args.timeout)
        if d:
            samples.append(d)
        print(f"--- {i+1}/{args.n} 홈으로 ---")
        _goto(cli, home, args.timeout)

    if not samples:
        print("\n측정 실패")
        return
    print("\n===== 목표 지점 정지 오차 =====")
    for key in ("dx_cm", "dy_cm", "dist_cm", "dyaw_deg"):
        vals = [s[key] for s in samples]
        sd = statistics.pstdev(vals) if len(vals) > 1 else 0.0
        print(f"  {key:9s} 평균={statistics.fmean(vals):+7.2f}  표준편차={sd:5.2f}  "
              f"최소={min(vals):+7.2f}  최대={max(vals):+7.2f}")
    worst = max(s["dist_cm"] for s in samples)
    worst_yaw = max(abs(s["dyaw_deg"]) for s in samples)
    print(f"\n  최악 거리오차 {worst:.1f}cm / 최악 각도오차 {worst_yaw:.1f}deg")
    print("  -> 마커 정밀 정렬 없이 바로 파지 시도 가능해 보입니다."
          if worst <= 5.0 and worst_yaw <= 10.0 else
          "  -> 마커(ArUco) 기반 정밀 보정 단계가 필요합니다.")


COMMANDS = {"ping": cmd_ping, "raw": cmd_raw, "listen": cmd_listen,
            "map-start": cmd_map_start, "map-end": cmd_map_end,
            "reloc": cmd_reloc, "goto": cmd_goto, "repeat": cmd_repeat}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("command", choices=sorted(COMMANDS))
    p.add_argument("--iface", default="eth0")
    p.add_argument("--pcd", default=DEFAULT_PCD)
    p.add_argument("--x", type=float, default=1.0)
    p.add_argument("--y", type=float, default=0.0)
    p.add_argument("--yaw", type=float, default=0.0)
    p.add_argument("--n", type=int, default=5)
    p.add_argument("--sec", type=float, default=15.0)
    p.add_argument("--timeout", type=float, default=180.0)
    args = p.parse_args()

    cli = connect(args.iface, args.pcd)
    try:
        COMMANDS[args.command](cli, args)
    finally:
        if args.command in ("goto", "repeat"):
            cli.pause_nav()


if __name__ == "__main__":
    main()
