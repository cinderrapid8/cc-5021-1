# 虚拟运动示范动作重定向工作台

把一套源骨架上的**挥臂、转体、迈步**动画迁移到比例、坐标帧、层级都不同的目标角色，并在网页中**同时播放两者**。重定向建立在显式骨骼对应、静止姿态与局部坐标修正之上，而非按名称复制四元数。

## 运行

```bash
npm install
npm run serve          # http://127.0.0.1:8080/
```

测试（引擎纯数学，无需浏览器）：

```bash
npm test               # Node 内置 test runner，11 项
```

无头浏览器冒烟测试（需要可用的 Chrome/Chromium，可用环境变量 `CHROME_BIN`、
`CHROME_LD_PATH`、`FONTCONFIG_FILE` 指定）：

```bash
npm run test:browser
```

## 两个角色刻意不同

| | 源 Slim | 目标 Stocky |
| --- | --- | --- |
| 朝向 | Y-up，forward −Z | Z-up，forward +Y |
| 单位 | 厘米（1m=100u） | 米 |
| 静止姿态 | T-pose | A-pose，手臂外展 35°，膝盖预弯 |
| 比例 | 瘦高 | 矮壮，四肢更短 |
| 层级 | 含 neck、toe | 无 neck、无 toe；额外有 Belt_Twist、Skull |
| 命名 | `L_shoulder` 等 | `arm_L` 等（无法按名称匹配） |

## 核心设计（对应需求）

- **坐标帧统一**（`src/engine/frame.js`）：每个骨架声明 up/forward/手系/单位，
  全部转换到规范帧（Y-up、−Z forward、右手系、米）。旋转用共轭
  `Qc = F·Qa·F⁻¹` 转换，位移按单位缩放；左右手系差异显式记录而非假装是旋转。
- **显式骨骼对应**（`src/data/skeletons.js` 的 `defaultMapping`）：目标→源映射，
  不依赖名字。界面可逐关节修改。
- **静止姿态与局部坐标修正**（`src/engine/retarget.js`）：
  - 补偿策略用世界空间旋转增量 `Δ = SAnimW·SRestW⁻¹`，再 `TAnimW = Δ·TRestW`；
  - 跳过策略用局部增量 `TLocal = TRestLocal·(SAnimLocal·SRestLocal⁻¹)`。
- **根移动与关节旋转分离**：根位置按骨盆高度比缩放后叠加到目标自身根位置；
  其余偏移一律沿用目标骨架，因此**目标骨长始终等于模型定义**（有测试断言）。
- **缺失关节可见化**：目标独有关节（Belt_Twist、Skull）标黄且不被驱动；源被跳过
  的关节（neck、L_toe、R_toe）标灰。补偿模式下缺失链的扭转由下游继承，跳过模式
  下无对应的目标关节冻结在自身静止姿态，两者差异由量化测试捕捉。
- **四元数符号连续性**（`stitchSigns` + `slerp`）：逐关节最短路径规范化，循环
  首尾强制同号，接缝处不反转。
- **量化验证而非观感**（`src/engine/validation.js`）：8 类检查，含自制不对称姿态
  的手算轨迹、关节朝向/位置误差、骨长不变量、根轨迹换算、循环接缝、跳过/补偿
  差异、配置往返一致性，以及「按名称/序号直接复制四元数必然失败」的对照实验。
- **配置保存/载入**（`src/engine/config.js`）：结果完全由配置决定，往返输出逐位
  一致（误差 < 1e-15）。
- **可播放导出**（`src/three/exporter.js`）：把规范帧关键帧转回目标自身坐标帧，
  经 `GLTFExporter` 输出带 `AnimationClip` 的 GLB（20 个四元数轨道 + 1 个根位移
  轨道），另有自描述 JSON 可重新载入播放。这不是把两个模型叠在一起。

## 目录

```
src/engine/   数学、坐标帧、骨架/FK、重定向、验证、配置（无 three 依赖，可单测）
src/data/     两个骨架与解析源动画
src/three/    查看器与 GLB/JSON 导出
src/ui/       界面控制
tests/        Node 单元测试与无头浏览器冒烟测试
tools/        静态服务器
```
