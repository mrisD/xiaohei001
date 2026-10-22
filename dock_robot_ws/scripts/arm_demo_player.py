#!/usr/bin/env python3
"""
双臂演示动作播放器

动作定义在 arm_demo_motions.yaml，用关键帧描述，改幅度/时长只改 yaml，不用重编。

用法:
  python3 arm_demo_player.py --list                        # 列出所有动作
  python3 arm_demo_player.py --motion greet                # 播放单个动作
  python3 arm_demo_player.py --motion greet --speed 0.6    # 0.6 倍速（更慢更稳）
  python3 arm_demo_player.py --sequence                    # 按 yaml 里 sequence 的顺序连播
  python3 arm_demo_player.py --sequence --loop             # 循环播放，用于展台
  python3 arm_demo_player.py --motion greet --dry-run      # 只校验和打印，不动机械臂

安全提示:
  本脚本直接向 joint_trajectory_controller 下发轨迹，**绕过 MoveIt 的碰撞检测**。
  新写的动作一定先用 --dry-run 看数值，再单独把每一帧摆到位确认不碰撞，
  最后才连播。
"""

import argparse
import os
import sys
import time

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient

LEFT_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
RIGHT_JOINTS = ['jointr1', 'jointr2', 'jointr3', 'jointr4', 'jointr5']

LEFT_ACTION = '/left_arm_controller/follow_joint_trajectory'
RIGHT_ACTION = '/right_arm_controller/follow_joint_trajectory'

# 关节软限位，来自 dock_robot_description/urdf/common_properties.xacro
# 注意 joint3 / jointr3 是不对称的
LIMITS = {
    'joint1': (-1.570, 1.570), 'joint2': (-1.570, 1.570),
    'joint3': (-2.356, 0.785), 'joint4': (-1.570, 1.570), 'joint5': (-3.140, 3.140),
    'jointr1': (-1.570, 1.570), 'jointr2': (-1.570, 1.570),
    'jointr3': (-2.356, 0.785), 'jointr4': (-1.570, 1.570), 'jointr5': (-3.140, 3.140),
}

# 超过这个角速度就提示（rad/s），只是提醒，不阻止
# 与 joint_limits.yaml 的 max_velocity（默认 0.51）保持一致留点余量
VEL_WARN = 0.45

DEFAULT_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'arm_demo_motions.yaml')


def to_duration(sec: float) -> Duration:
    sec = max(0.0, sec)
    whole = int(sec)
    return Duration(sec=whole, nanosec=int(round((sec - whole) * 1e9)))


def fmt(v):
    return '[' + ', '.join(f'{x:+.2f}' for x in v) + ']'


class ArmDemoPlayer(Node):
    def __init__(self):
        super().__init__('arm_demo_player')
        self.current = {}
        self.create_subscription(JointState, '/joint_states', self._on_js, 10)
        self.left_ac = ActionClient(self, FollowJointTrajectory, LEFT_ACTION)
        self.right_ac = ActionClient(self, FollowJointTrajectory, RIGHT_ACTION)

    def _on_js(self, msg: JointState):
        self.current.update(zip(msg.name, msg.position))

    # ---------------- 基础工具 ----------------

    def wait_for_joint_states(self, timeout=5.0) -> bool:
        t0 = time.time()
        need = LEFT_JOINTS + RIGHT_JOINTS
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(n in self.current for n in need):
                return True
        return False

    def wait_for_servers(self, timeout=5.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.left_ac.server_is_ready() and self.right_ac.server_is_ready():
                return True
            time.sleep(0.1)
        return False

    def snapshot(self):
        return ([self.current.get(n, 0.0) for n in LEFT_JOINTS],
                [self.current.get(n, 0.0) for n in RIGHT_JOINTS])

    # ---------------- 校验 ----------------

    def validate(self, motion_name, motion, speed):
        """
        检查限位和角速度，返回是否通过（有越限则拒绝播放）。

        时间基准必须和 build_trajectory() 完全一致：
          第 1 帧用 approach 秒，之后每帧用各自的 duration 秒。
        否则算出来的角速度和实际下发的不一样。
        """
        ok = True
        left, right = self.snapshot()
        prev_l, prev_r = left, right
        approach = float(motion.get('approach', 3.0)) / speed
        first = True

        for i, pt in enumerate(motion.get('points', []), start=1):
            lv = pt.get('left', prev_l)
            rv = pt.get('right', prev_r)
            dt = approach if first else float(pt.get('duration', 1.0)) / speed
            first = False
            if dt <= 0:
                print(f'  ✘ [{motion_name}] 第 {i} 帧 duration 必须 > 0')
                ok = False
                continue

            for name, val in zip(LEFT_JOINTS, lv):
                lo, hi = LIMITS[name]
                if not (lo - 1e-6 <= val <= hi + 1e-6):
                    print(f'  ✘ [{motion_name}] 第 {i} 帧 {name}={val:+.3f} '
                          f'超出限位 [{lo:+.3f}, {hi:+.3f}]')
                    ok = False
            for name, val in zip(RIGHT_JOINTS, rv):
                lo, hi = LIMITS[name]
                if not (lo - 1e-6 <= val <= hi + 1e-6):
                    print(f'  ✘ [{motion_name}] 第 {i} 帧 {name}={val:+.3f} '
                          f'超出限位 [{lo:+.3f}, {hi:+.3f}]')
                    ok = False

            for name, a, b in zip(LEFT_JOINTS, prev_l, lv):
                v = abs(b - a) / dt
                if v > VEL_WARN:
                    print(f'  ! [{motion_name}] 第 {i} 帧 {name} 角速度 '
                          f'{v:.2f} rad/s (> {VEL_WARN})，加长 duration 或减小幅度')
            for name, a, b in zip(RIGHT_JOINTS, prev_r, rv):
                v = abs(b - a) / dt
                if v > VEL_WARN:
                    print(f'  ! [{motion_name}] 第 {i} 帧 {name} 角速度 '
                          f'{v:.2f} rad/s (> {VEL_WARN})，加长 duration 或减小幅度')

            prev_l, prev_r = lv, rv

        if ok:
            print(f'  ✔ [{motion_name}] 限位检查通过')
        return ok

    # ---------------- 轨迹构造 ----------------

    def build_trajectory(self, motion, speed):
        """
        第 0 个点 = 当前实测姿态（保证从所在位置平滑接入），
        之后按 yaml 的关键帧累加 time_from_start。
        """
        left, right = self.snapshot()
        approach = float(motion.get('approach', 3.0)) / speed

        traj_l = JointTrajectory()
        traj_l.joint_names = LEFT_JOINTS
        traj_r = JointTrajectory()
        traj_r.joint_names = RIGHT_JOINTS

        p0l = JointTrajectoryPoint()
        p0l.positions = list(left)
        p0l.velocities = [0.0] * 5
        p0l.time_from_start = to_duration(0.0)
        traj_l.points.append(p0l)

        p0r = JointTrajectoryPoint()
        p0r.positions = list(right)
        p0r.velocities = [0.0] * 5
        p0r.time_from_start = to_duration(0.0)
        traj_r.points.append(p0r)

        t = approach
        prev_l, prev_r = left, right
        for pt in motion.get('points', []):
            lv = pt.get('left', prev_l)
            rv = pt.get('right', prev_r)

            pl = JointTrajectoryPoint()
            pl.positions = list(lv)
            pl.velocities = [0.0] * 5
            pl.time_from_start = to_duration(t)
            traj_l.points.append(pl)

            pr = JointTrajectoryPoint()
            pr.positions = list(rv)
            pr.velocities = [0.0] * 5
            pr.time_from_start = to_duration(t)
            traj_r.points.append(pr)

            t += float(pt.get('duration', 1.0)) / speed
            prev_l, prev_r = lv, rv

        return traj_l, traj_r, t

    # ---------------- 播放 ----------------

    def play(self, motion_name, motion, speed):
        traj_l, traj_r, total = self.build_trajectory(motion, speed)

        print(f'  → 播放 "{motion.get("name", motion_name)}" '
              f'({len(traj_l.points)} 帧, 约 {total:.1f}s)')
        for i, (pl, pr) in enumerate(zip(traj_l.points, traj_r.points)):
            if i == 0:
                print(f'      {i}: t={pl.time_from_start.sec:>4}.{pl.time_from_start.nanosec // 1000000:03d}s'
                      f'  (当前姿态)')
            else:
                print(f'      {i}: t={pl.time_from_start.sec:>4}.{pl.time_from_start.nanosec // 1000000:03d}s'
                      f'  L{fmt(pl.positions)}  R{fmt(pr.positions)}')

        goals = []
        for ac, traj in ((self.left_ac, traj_l), (self.right_ac, traj_r)):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = traj
            goals.append(ac.send_goal_async(goal))

        # 等两个目标都被接受
        t0 = time.time()
        while time.time() - t0 < 5.0 and not all(g.done() for g in goals):
            rclpy.spin_once(self, timeout_sec=0.05)
        if not all(g.done() for g in goals):
            print('  ✘ 动作目标没被接受，检查 controller 是否 active')
            return False

        handles = [g.result() for g in goals]
        if any(h is None or not h.accepted for h in handles):
            print('  ✘ 控制器拒绝了轨迹（可能超限或起点不连续）')
            return False

        results = [h.get_result_async() for h in handles]

        # 等执行完（留 5s 余量）
        deadline = time.time() + total + 5.0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(r.done() for r in results):
                break

        codes = []
        for r in results:
            if r.done() and r.result() is not None:
                codes.append(r.result().result.error_code)
            else:
                codes.append(None)

        if any(c not in (0, None) for c in codes):
            print(f'  ✘ 执行结束但返回错误码: left={codes[0]} right={codes[1]} '
                  f'(0=SUCCESSFUL, -1=INVALID_GOAL, -4=INVALID_JOINT_NAMES, '
                  f'-5=PATH_TOLERANCE_VIOLATED, -6=GOAL_TOLERANCE_VIOLATED)')
            return False

        print('  ✔ 完成')
        return True


def load_motions(path):
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def main():
    ap = argparse.ArgumentParser(description='双臂演示动作播放器')
    ap.add_argument('--motions', default=DEFAULT_YAML, help='动作库 yaml 路径')
    ap.add_argument('--list', action='store_true', help='列出所有动作')
    ap.add_argument('--motion', help='播放指定动作')
    ap.add_argument('--sequence', action='store_true', help='按 sequence 顺序连播')
    ap.add_argument('--speed', type=float, default=1.0, help='速度倍率，>1 更快')
    ap.add_argument('--loop', action='store_true', help='循环播放')
    ap.add_argument('--dry-run', action='store_true', help='只校验和打印，不动机械臂')
    args = ap.parse_args()

    if not os.path.exists(args.motions):
        print(f'找不到动作库: {args.motions}')
        return 1

    data = load_motions(args.motions)
    motions = data.get('motions', {})
    sequence = data.get('sequence', [])

    if args.list:
        print('可用动作:')
        for k, v in motions.items():
            n = len(v.get('points', []))
            print(f'  {k:<16} {v.get("name", "")}  ({n} 帧)')
        print(f'\nsequence: {" -> ".join(sequence) if sequence else "(未定义)"}')
        return 0

    if not args.motion and not args.sequence:
        ap.print_help()
        return 1

    play_list = sequence if args.sequence else [args.motion]
    if args.sequence and not sequence:
        print('yaml 里没有定义 sequence')
        return 1

    missing = [m for m in play_list if m not in motions]
    if missing:
        print(f'动作库里没有: {missing}')
        return 1

    rclpy.init()
    node = ArmDemoPlayer()
    rc = 0
    try:
        if not node.wait_for_joint_states():
            print('✘ 收不到 /joint_states，先确认 ros2_control 已启动')
            return 1

        # 先整体校验
        print('校验:')
        all_ok = True
        for name in play_list:
            if not node.validate(name, motions[name], args.speed):
                all_ok = False
        if not all_ok and not args.dry_run:
            print('\n有帧超出限位，已拒绝播放。修正 yaml 或加 --dry-run 查看。')
            return 2

        if args.dry_run:
            print('\n(--dry-run，不播放)')
            for name in play_list:
                node.build_trajectory(motions[name], args.speed)
                print(f'  {name}: 轨迹构造正常')
            return 0

        if not node.wait_for_servers():
            print('✘ 两个手臂的 follow_joint_trajectory action server 没就绪')
            return 1

        while True:
            for name in play_list:
                if not node.play(name, motions[name], args.speed):
                    rc = 3
                time.sleep(0.3)
            if not args.loop:
                break
            print('--- 循环 ---')
    except KeyboardInterrupt:
        print('\n已中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return rc


if __name__ == '__main__':
    sys.exit(main())
