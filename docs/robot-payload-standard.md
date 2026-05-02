# 机器人板端上报数据标准

## 1. 接口说明

- 上报地址：`POST /api/telemetry/ingest/`
- 内容类型：`application/json`
- 鉴权方式：
  - 一期演示版可走内网白名单或网关鉴权
  - 正式版建议补充 `X-Device-Key`、签名字段或双向证书

## 2. 推荐上报频率

- 心跳与状态：每 `5s` 上报一次
- 位置与运动信息：每 `1-2s` 上报一次
- 告警与识别事件：实时触发即时报送
- 图片/视频片段：通过对象存储上传后回传 URL

## 3. 标准 JSON 结构

```json
{
  "sequence_id": "telemetry-20260502-000001",
  "robot_code": "ZSL-1A-07",
  "robot_name": "南入口巡检机器人",
  "reported_at": "2026-05-02T14:35:18+08:00",
  "position": {
    "name": "太阳宫公园南入口",
    "latitude": 39.983521,
    "longitude": 116.447153
  },
  "motion": {
    "speed": 1.26,
    "heading": 83.50
  },
  "power": {
    "battery_level": 78,
    "charging": false
  },
  "network": {
    "signal_strength": 92,
    "network_type": "5G"
  },
  "runtime": {
    "mode": "auto",
    "status": "online"
  },
  "detections": [
    {
      "type": "vehicle_illegal_parking",
      "label": "自行车违停",
      "confidence": 0.925,
      "risk_level": "medium",
      "bbox": {
        "x": 124,
        "y": 88,
        "width": 162,
        "height": 236
      },
      "snapshot_url": "https://example.com/snapshots/event-1024.jpg",
      "event_time": "2026-05-02T14:35:16+08:00"
    }
  ]
}
```

## 4. 字段定义

| 字段 | 类型 | 必填 | 说明 |
| --- | --- | --- | --- |
| `sequence_id` | string | 是 | 单次上报流水号，要求全局唯一 |
| `robot_code` | string | 是 | 机器人唯一编号 |
| `robot_name` | string | 否 | 机器人显示名称 |
| `reported_at` | string(datetime) | 是 | 板端上报时间，ISO 8601 格式 |
| `position.name` | string | 是 | 当前所在区域名称 |
| `position.latitude` | number | 否 | 纬度 |
| `position.longitude` | number | 否 | 经度 |
| `motion.speed` | number | 否 | 当前速度，单位 m/s |
| `motion.heading` | number | 否 | 当前朝向角度，0-360 |
| `power.battery_level` | integer | 是 | 电量百分比，0-100 |
| `power.charging` | boolean | 否 | 是否在充电 |
| `network.signal_strength` | integer | 是 | 网络信号强度，0-100 |
| `network.network_type` | string | 否 | 如 `4G`、`5G`、`WiFi` |
| `runtime.mode` | string | 是 | `auto`、`manual`、`standby`、`returning` |
| `runtime.status` | string | 是 | `online`、`offline`、`warning`、`charging` |
| `detections` | array | 否 | AI 识别结果列表 |

## 5. detections 子项建议

| 字段 | 类型 | 必填 | 说明 |
| --- | --- | --- | --- |
| `type` | string | 是 | 识别类型编码 |
| `label` | string | 是 | 识别名称 |
| `confidence` | number | 是 | 0-1 或后端换算为百分比 |
| `risk_level` | string | 否 | `high`、`medium`、`low` |
| `bbox` | object | 否 | 检测框坐标 |
| `snapshot_url` | string | 否 | 抓拍图 URL |
| `event_time` | string(datetime) | 否 | 识别发生时间 |

## 6. 事件编码建议

- `vehicle_illegal_parking`：车辆违停
- `crowd_gathering`：人员聚集
- `smoke_alert`：烟雾告警
- `fall_detection`：跌倒识别
- `intrusion_detection`：越界识别
- `equipment_fault`：设备异常

## 7. 后端处理建议

- 心跳类数据写入遥测表，用于状态展示和轨迹回放
- 告警类数据进入事件表，用于值班人员处置
- 大文件只上传 URL，不建议直接把图片二进制塞进 JSON
- `sequence_id` 应支持幂等校验，防止弱网重传造成重复入库
