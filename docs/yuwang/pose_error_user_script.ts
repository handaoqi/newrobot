// 位姿偏差计算 —— Foxglove User Script
//
// 用途：计算定位输出 (/odom/localization_odom) 与扫描匹配校验位姿
// (/localization/scan_match_pose) 之间的平面距离偏差，在 Plot 面板里观察漂移
// 尖峰出现在什么时刻、是否与自愈校正同步。
//
// 使用：Foxglove 里新建一个 User Script 面板，把本文件内容粘进去，
// 然后在 Plot 面板添加路径 /studio_script/pose_error.distance
//
// 与需求文档 (Foxglove Studio ROS2 Bag回放调试界面设计.docx) 的差异：
//   - 文档用的 messageHandler / publish() 不是 Foxglove User Script API
//   - 输出话题必须以 /studio_script/ 开头，不能用 /debug/pose_error
//   - 输入话题必须通过 export const inputs 声明
//   - 话题名按本仓库真实名称更正（原文的 /ukf/odometry、/ndt_pose 不存在）
//
// 注意：本脚本未在宿主中执行验证过（本机未安装 Lichtblick/Foxglove），
// User Script API 在不同宿主版本间存在差异。

import { Input } from "./types";

const LOCALIZATION_ODOM = "/odom/localization_odom";
const SCAN_MATCH_POSE = "/localization/scan_match_pose";

export const inputs = [LOCALIZATION_ODOM, SCAN_MATCH_POSE];
export const output = "/studio_script/pose_error";

type Point = { x: number; y: number; z: number };

type Output = {
  /** 平面距离偏差，米。两路位姿都到齐之前为 0。 */
  distance: number;
  dx: number;
  dy: number;
  /** 两条消息 header.stamp 的时间差，秒。绝对值偏大说明这次比较跨了太久，distance 不可信。 */
  stampSkewSec: number;
};

let lastOdom: { point: Point; stampSec: number } | undefined;
let lastScanMatch: { point: Point; stampSec: number } | undefined;

function toSeconds(stamp: { sec: number; nsec: number }): number {
  return stamp.sec + stamp.nsec / 1e9;
}

export default function script(
  event: Input<typeof LOCALIZATION_ODOM> | Input<typeof SCAN_MATCH_POSE>,
): Output | undefined {
  if (event.topic === LOCALIZATION_ODOM) {
    lastOdom = {
      point: event.message.pose.pose.position,
      stampSec: toSeconds(event.message.header.stamp),
    };
  } else {
    lastScanMatch = {
      point: event.message.pose.position,
      stampSec: toSeconds(event.message.header.stamp),
    };
  }

  // 只有两路都收到过才输出，否则 Plot 上会出现一段无意义的 0。
  if (lastOdom == undefined || lastScanMatch == undefined) {
    return undefined;
  }

  const dx = lastOdom.point.x - lastScanMatch.point.x;
  const dy = lastOdom.point.y - lastScanMatch.point.y;

  return {
    distance: Math.sqrt(dx * dx + dy * dy),
    dx,
    dy,
    stampSkewSec: lastOdom.stampSec - lastScanMatch.stampSec,
  };
}
