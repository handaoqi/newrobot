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
