# 6 轴 IMU 姿态解算（ADXRS453 × 3 + ADXL355）

这是一个 C99、`float` 的姿态解算库，面向 Zynq ZU2CG 等可提供稳定主机时钟的平台。核心算法是带陀螺仪零偏估计的 6 维误差状态 EKF（ESKF）；仓库还包含互补滤波对照、仿真测试和 CSV 回放工具。底层 SPI、FPGA 同步、驱动和板级标定存储**尚未实现**，需由应用提供。

**当前状态：尚未通过完整性能验收，也未在目标硬件上验证。** 最新主机测试为 15 项通过 14 项；振动叠加零偏突变场景的峰值姿态误差为 **0.182522 rad**，高于预定的 **0.174533 rad** 上限。30 个固定种子的倾角 RMS 对照为 ESKF **0.017359389 rad**、互补滤波 **0.0219991227 rad**；这个局部优势不代表全部场景达标。详细条件和结果见 [验证报告](docs/verification-report.md)。

## 构建与测试

需要支持 C99 的编译器、CMake ≥ 3.20，以及 Ninja（或替换为本机已安装的构建器）。以下命令在仓库根目录执行；`build-readme-ninja` 是独立的构建目录，不会改动已有构建产物。

```sh
cmake -S . -B build-readme-ninja -G Ninja
cmake --build build-readme-ninja
ctest --test-dir build-readme-ninja --output-on-failure
```

目前 `ctest` 的预期结果是 **14/15 通过**，`scenarios` 测试因上述峰值门限失败。不要将其视为全部测试通过。还可以运行 30 种子对照；在 Windows 上为可执行文件名加 `.exe`：

```sh
./build-readme-ninja/tools/imu_compare --seeds 30 --output build-readme-ninja/comparison.csv
./build-readme-ninja/tools/imu_replay tests/data/replay_smoke.csv build-readme-ninja/replay_output.csv
```

CMake 目标 `imu` 是静态库，公开头文件位于 `include/imu/`。嵌入式集成时按目标平台的浮点 ABI 编译，并在目标板测量执行时间与栈用量；主机数据不是 Zynq 实时性证明。

## 传感器和时间约定

- 建议以严格的 **485 Hz** 主机节拍生成单调递增的纳秒时间戳，名义周期约 **2.061856 ms**。算法由相邻时间戳计算 `dt`，不应把固定周期硬编码为实际采样间隔。
- ADXL355 配置为 `ODR_LPF=0x3`（500 Hz / 125 Hz）、`EXT_SYNC=10`、`EXT_CLK=0`，由主机 485 Hz 信号驱动 SYNC。同步及启动丢弃规则见 [Zynq 集成指南](docs/integration-zynq.md)。
- 三颗 ADXRS453 可同时拉低片选以对齐寄存器锁存时刻，但各自内部 MEMS 时钟**不会因此同步**。SPI 响应比命令晚一个交换，驱动必须将响应、状态和锁存时刻正确配对。
- 输入陀螺仪单位为 **rad/s**，加速度计为 **m/s²**，温度为 **°C**。`sample.timestamp_ns` 是共同主机锁存参考时刻，而非已扣除滤波延迟的时刻。
- 可配置传感器信号链延迟；三个 `gyro_delay_s[]` 必须**完全相等**。输出 `timestamp_ns` 保留主机参考时刻，`attitude_epoch_ns` 表示减去共同陀螺仪延迟后的名义姿态时刻。没有将姿态外推到主机时刻。

机体系为 **FRD**（前、右、下），导航系为 **NED**（北、东、下）；`q_nb` 是标量在前、将机体系向量旋转到导航系的四元数。水平静止时加速度计测得的比力约为 `[0, 0, -9.80665] m/s²`。没有可用的绝对航向观测，输出偏航角会随时间漂移。

## 最小接入流程

公开 API 见 [`include/imu/imu.h`](include/imu/imu.h) 和 [`include/imu/imu_types.h`](include/imu/imu_types.h)。应用持有持久化的 `imu_t`，并串行调用同一实例：

1. 调用 `imu_config_defaults(&config)`；设置三个相同的陀螺仪延迟、加速度计延迟和启动模式。默认延迟为零，应在板级测量后填写。需要正式运行时，加载并验证版本化的 `imu_calibration_t`；校验和由存储层验证。
2. 调用 `imu_init(&imu, &config, &calibration)`。仅作诊断时可显式设置 `config.allow_uncalibrated = true` 并传入 `NULL` 标定记录；此模式的输出会标记为未标定，不能当成正式精度结果。
3. 每个主机节拍由驱动组装完整的 `imu_sample_t`，包含四路测量的数值、温度、状态和“是否为新传感器测量”的 freshness 标志，再调用 `imu_process(&imu, &sample, &output)`。SPI 交换成功并不等于 MEMS 数据是新的。
4. 发布前检查返回状态、`output.initialized`、`output.health_flags` 和 `output.fusion_mode`。对 `TIME_GAP`、`SENSOR_DEGRADED` 或未初始化状态制定应用策略；不要仅凭四元数数值存在就认定当前帧健康。

输入通道只有在 `IMU_SENSOR_DATA_VALID` 置位、`IMU_SENSOR_FAULT_MASK` 全清且 freshness 为真时才可作为新观测。加速度计在明显线加速度、冲击或振动时会被降权或拒绝；此时姿态主要靠陀螺仪传播，合格的重力观测到来后才修正可观测的倾角误差。静止检测还会触发零角速率更新。**重力无法校正绝对偏航**。

`imu_update_magnetometer()` 目前只是预留接口：合法调用返回 `IMU_STATUS_AIDING_UNAVAILABLE`，不会修改滤波状态。俯仰接近 ±90° 时欧拉角不确定度不可定义，库会设置 `IMU_HEALTH_UNCERTAINTY_UNAVAILABLE` 并以有限哨兵值表示；此时应使用四元数。

## 进一步阅读

- [Zynq 集成指南](docs/integration-zynq.md)：SPI 响应流水线、SYNC/DRDY、校准、故障与发布策略。
- [验证报告](docs/verification-report.md)：测试命令、验收门限、资源测量及尚未解决的问题。
- [设计说明](docs/superpowers/specs/2026-09-21-imu-attitude-esekf-design.md)：算法和时序设计依据。

用于飞行控制或云台闭环前，仍需完成板级传感器标定、外部真值对照、真实振动日志回放，以及目标平台的实时性和故障处理验证。
