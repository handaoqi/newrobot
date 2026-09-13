# 场景低模资产库

平台提供一套面向公园导航和 3D 地图拼接的轻量低模资产，目录地址为：

```text
GET /scene-assets/catalog.json
GET /scene-assets/{asset_id}.glb
```

当前目录 schema 为 `roamerx.scene-assets.v1`，资产使用真实米制、右手坐标系、Z 轴向上。资产无外部纹理和动画，适合场景语义标识、地图拼接和可视化，不作为视觉模型训练数据集。

## 资产 ID

| 类别 | 资产 ID |
| --- | --- |
| 行人 | `person.adult`、`person.child`、`person.walking` |
| 墙体 | `wall.straight`、`wall.low`、`wall.corner` |
| 建筑 | `building.kiosk`、`building.service-room`、`building.restroom` |
| 车辆 | `vehicle.sedan`、`vehicle.van`、`vehicle.golf-cart` |
| 树木 | `tree.deciduous`、`tree.conifer`、`tree.shrub` |
| 道路 | `road.straight`、`road.curve90`、`road.intersection` |

## 地图实例引用

地图的 `scene_manifest.static_assets` 只保存实例引用和变换，不复制 GLB：

```json
{
  "asset_id": "tree.deciduous",
  "position": { "x": 12.4, "y": 8.2, "z": 0 },
  "orientation": { "x": 0, "y": 0, "z": 0, "w": 1 },
  "scale": 1,
  "source": "ai",
  "confidence": 0.96
}
```

普通物体以底面中心为原点；直墙和直道路段以起点为原点并沿本地 `+X` 延伸；转角墙以内角点为原点；路口以交汇中心为原点。对象和车辆等使用等比缩放，墙体和道路可以按目录中的 `scale_mode` 做长度/宽度缩放。

## 本地生成与校验

```bash
cd platform/frontend
npm run generate:scene-assets
npm run test:scene-assets
```

生成器位于 `scripts/generate_scene_assets.mjs`，每次生成会更新 GLB、catalog 中的文件大小、三角面数和 SHA-256。前端场景视角调试页会优先加载 GLB；资产目录或单个 GLB 不可用时回退到程序化占位几何。

## 场景视角地图模式

场景视角调试页提供“点云场景”“卫星地图”“街区模式”三种地图模式。卫星地图默认直接读取 `scene-map-config.json` 中的 XYZ 卫星瓦片地址，以地图 manifest 的锁定 GNSS 原点定位，支持拖拽和缩放，不需要高德 Key；也可通过运行时配置替换瓦片地址。没有有效原点或瓦片来源时显示“无可用来源”，不会使用默认坐标。

街区模式只读取经过机器人端点云语义处理生成的 `scene_semantics.json`，并要求静态实例置信度不低于 0.80；没有可靠清单时保留原始点云并显示“未生成可靠语义模型”，不会在浏览器中按网格配额猜测类别。结果通过 `POST /api/maps/{map_id}/scene-semantics/` 上传，也可以随地图包以 `scene_semantics.json` 作为小型 sidecar 携带。行人、车辆和自行车只来自实时三维语义对象，且机器狗速度不超过 `0.05 m/s` 或速度未知时会清空，不会写入静态地图。

如需切换其他瓦片源，可通过部署环境覆盖 `scene-map-config.json` 的 `satellite.tileUrl`；高德 Web JS API 仍作为可选兼容配置保留。高德配置和安全密钥不应写入版本库。
