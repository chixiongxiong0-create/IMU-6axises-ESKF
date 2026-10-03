# 三颗 ADXRS453 的 FPGA SPI 采集

本模块在单个 100 MHz 时钟域中，并行读取三颗 ADXRS453。RTL 使用
Verilog-2001，默认 SCLK 为 5 MHz，主机采集节拍为平均 485 Hz。
三个寄存器在同一 CS 下降沿锁存，各器件内部 MEMS 采样时钟仍然独立。

## 文件与连接

将 `rtl/spi_master_3ch.v` 和 `rtl/adxrs453_acq.v` 添加到 Vivado 的 Design Sources。
行为模型和 testbench 仅用于仿真，不添加到综合源文件。

| FPGA 端口 | 板上连接 |
|---|---|
| `clk` | 100 MHz 系统时钟 |
| `rst_n` | 低有效复位，允许异步置位，释放必须同步到 clk |
| `sclk` | 三颗器件 SCLK 共用 |
| `mosi` | 三颗器件 MOSI 共用 |
| `miso[0]` / `[1]` / `[2]` | 各自的 MISO，分别连接，不能短接 |
| `cs_n[0]` / `[1]` / `[2]` | 各自的 CS，模块产生相同波形 |

通道 0/1/2 并不自动代表 X/Y/Z；由板上安装方向和软件坐标变换定义。
主控和器件 I/O 电平、供电、去耦、MISO 上拉按器件手册和板级设计核对。

## 最小例化

```verilog
wire signed [15:0] rate0, rate1, rate2;
wire [31:0] raw0, raw1, raw2;
wire [63:0] latch_cycles;
wire [2:0] seq, valid_axes, startup_axes;
wire [4:0] error0, error1, error2;
wire frame_valid, startup_done, overrun;

adxrs453_acq #(
    .CLK_HZ(100000000),
    .SPI_HZ(5000000),
    .SAMPLE_HZ(485)
) gyro_acq (
    .clk(clk_100m), .rst_n(reset_n_sync), .miso(gyro_miso),
    .sclk(gyro_sclk), .mosi(gyro_mosi), .cs_n(gyro_cs_n),
    .gyro0(rate0), .gyro1(rate1), .gyro2(rate2),
    .raw0(raw0), .raw1(raw1), .raw2(raw2),
    .timestamp_cycles(latch_cycles), .frame_seq(seq),
    .frame_valid(frame_valid), .gyro_valid(valid_axes),
    .err0(error0), .err1(error1), .err2(error2),
    .startup_done(startup_done), .startup_ok(startup_axes), .overrun(overrun)
);
```

所有输出数据和状态在 clk 域内消费。需要送到其它时钟域时，在该边界使用
异步 FIFO 或完整的握手机制，不分别同步总线中的每一位。

## 输出契约

| 端口 | 含义 |
|---|---|
| `gyro0/1/2` | 当前帧三路 16 位有符号二补码原始角速度 |
| `raw0/1/2` | 当前帧三路完整 32 位 SPI 响应，含故障和校验位 |
| `timestamp_cycles` | 本帧 A 交换的 CS 下降沿对应系统周期计数，64 位 |
| `frame_seq` | 本帧 A/B 命令序号，3 位，按节拍模 8 递增 |
| `frame_valid` | 一周期脉冲，表示一帧已完成、输出可读取 |
| `gyro_valid[2:0]` | 对应通道通过启动和当前响应检查；只在 frame_valid 时消费 |
| `err0/1/2` | 各路 5 位当前帧协议错误位掩码，见下表 |
| `startup_done` | 启动流程已执行完，保持到复位；不保证三路都成功 |
| `startup_ok[2:0]` | 各路 CHK 故障生成、消退检查通过，保持到复位 |
| `overrun` | 新节拍到达时上一帧仍忙，置位并保持到复位 |

输出在发布帧时一起更新，并保持到下一次发布。即使某路无效，仍保留本帧
实际接收的 raw 和解码后的 gyro；不能把这些数值当作健康测量使用。
`frame_valid` 不是“三路全部有效”的标志。启动失败通道即使后来返回正常
响应也不恢复 `gyro_valid`，需排查原因后复位重新检查。

| err 位 | 非零的含义 |
|---|---|
| 0 | 响应[31:16]未满足奇校验 |
| 1 | 响应[31:0]未满足奇校验 |
| 2 | 响应 SQ[31:29] 与 A 命令序号不符 |
| 3 | ST[27:26]不是 01（包括无效、自检、读写错误响应） |
| 4 | 故障字段[7:1]非零 |

`err=0` 而 `gyro_valid=0` 可以表示该路启动检查失败。
raw[7:1] 依次为 PLL、Q、NVM、POR、PWR、CST、CHK，raw[25:10] 为角速度。
读写错误响应的具体 SPI/RE/DU 位仍保留在 raw，首版不额外读 FAULT1 寄存器。

## 启动、流水线与节拍

在传感器电源已有效后释放 FPGA 复位。模块先等待至少 100 ms，再发送
CHK=1 命令 0x20000003；等待至少 50 ms 后发送 CHK=0 的 0x20000000；
再等待至少 50 ms，然后检查两次故障报告以及一次正常恢复报告。
由于 SPI 响应延迟一次交换，五次启动交换期间不发布采集帧。
FPGA 单独复位不会使传感器再次输出上电初始字，因此首响应仅丢弃，不要求
强制等于 0x00000001。启动总时长略大于 200 ms。

运行时每个节拍发 A、B 两次 32 位交换，两条命令使用同一序号。B 的响应
属于 A 锁存的数据；A 的旧响应丢弃。正常采集的两次传输约占 13.5 us，
相比 2.062 ms 的节拍有充足裕量。输出时间戳取 A 交换，不能替换成 B 的时间。

节拍用相位累加器生成，默认相邻节拍间隔为 206185 或 206186 个周期，
平均 485 Hz。若出现 overrun，当前帧仍完整发布，新节拍请求丢弃，命令
序号继续随节拍推进。不要仅靠 3 位序号统计大量丢帧，应结合时间戳。

`spi_master_3ch` 的 `start` 只在空闲接受；`busy` 包含交换和 CS 高间隔，
`done` 在高间隔完成后产生一周期脉冲。忙时 start 不排队。
SPI 半周期为 ceil(CLK_HZ/(2*SPI_HZ))，实际频率不超过指定值。默认每半周期
100 ns，片选提前及保持至少一个半周期，两次交换间 CS 高至少 200 ns。
单独使用该引擎时 `CS_HIGH_NS` 最低允许 100 ns，默认仍为 200 ns。
参数非法时仿真输出 `PARAM_REJECT` 并终止，脚本将其视为失败；invalid
suite 专门验证这些诊断。片选高间隔最多支持 2^32 个系统周期，超出会被拒绝。
诊断块在综合中由 translate_off 排除，综合前仍应
先用所选参数运行仿真。

## 单位、时间戳与 C99 算法接入

软件换算公式为：

```c
double gyro_rad_s = (double)raw_signed / 80.0 * (3.141592653589793 / 180.0);
```

每原始 LSB 为 0.0125 deg/s。再执行板级轴映射、零偏和标度校准。
默认 100 MHz 下 `timestamp_ns = timestamp_cycles * 10`；计数零点是 FPGA
复位释放后，计数值表示 CS 下降时该自由运行计数器的更新后值。换用其它
时钟时必须根据实际频率换算，不能继续固定乘 10。
若系统已有单调时钟，接入方负责零点映射，并在 FPGA 复位后处理时间轴重建。

本仓库 `imu_process()` 需要完整的 IMU 帧和明确 freshness。SPI 交换成功
以及 SQ 匹配只证明协议和配对，不证明传感器内部完成了新转换。相同数据
也不能直接判定重复采样。加速度计、温度、freshness、公共时间轴及驱动
状态映射由后续采集适配层提供；本 RTL 不直接构造 `imu_sample_t`。
主机 485 Hz 锁存不让三颗 MEMS 同步，不能宣称三轴物理同时采样。

## Vivado 仿真

在仓库根目录运行：

```powershell
powershell -NoProfile -File tests/rtl/run_vivado.ps1 `
  -VivadoRoot D:/Xilinx/2025.1/Vivado -Suite all
```

`Suite` 可选 spi、acq、invalid、all。`BuildDir` 可指定产物目录（支持空格），
默认 `build-rtl/`，各 suite 有独立子目录。脚本依次运行 xvlog、xelab、xsim；
RTL 不使用 `-sv`，testbench 使用 SystemVerilog。每阶段退出码和仿真日志
均受检查，不仅依赖 XSim 进程退出码。正常 suite 必须出现其 PASS 标志且
没有失败标志；invalid suite 必须出现十一个指定实例的参数诊断。
日志在 `build-rtl/<suite>/simulation.log`，工具输出为各 `*.console.log`。
脚本未默认记录全部波形，以免全长启动测试生成大量数据；展开使用 typical
debug，可在 Vivado 中加载 snapshot 后选择需要的信号重新运行和记录。

验证日期：2026-10-03，Vivado/XSim v2025.1，SW Build 6140274。

- SPI：5 MHz 时序，100 MHz/6 MHz 分频向上取整，三路不同返回字，精确
  32 时钟，CS/MOSI 建立保持，忙时请求、输出保持、复位恢复和极大合法
  CS 间隔不发生截断。
- 采集：完整默认启动时长，A/B 流水线和 A 时间戳，正负及边界值，16 帧以上
  序号回绕，485 Hz 周期与累计相位误差，所有协议错误和七种故障注入，
  MISO 卡高/卡低，单路启动失败、overrun，以及 FPGA 单独复位和第 5 位中止。
- 非法参数：十一个实例，覆盖零/负时钟与频率、SPI 上限、过短半周期与片选
  高间隔、超过计数器容量的高间隔、零/负采集频率及不足的帧带宽。
- 脚本失败识别：`-Suite spi -InjectFailure` 故意输出 FAIL，脚本返回非零；
  正常运行须返回零。这个开关仅用于验证脚本，不能用于正式验收。

最终 `-Suite all` 返回 0，spi、acq、invalid 三组全部通过。独立代码审查后
补充 A/B 不同锁存数据的测试，并修复极大 CS 间隔的有符号参数溢出；两项
回归均验证先失败、修复后通过。超过 32 位计数器容量的间隔也有独立拒绝用例。
采集 suite 包含约 685 ms 的仿真时间，完整运行三轮启动流程。
尚未指定完整 FPGA part、
I/O 引脚和走线延迟，因此未执行目标器件综合、布局布线、静态时序或实板测试。

## 板级时序与后续接入

为 clk 添加实际周期的主时钟约束；为 SPI 输出/返回时序建模，包括生成
SCLK 的相位关系、MOSI/CS 输出延迟、器件 MISO 最大有效时间 40 ns、首位
访问 20 ns、板上走线延迟和 FPGA 输入输出延迟。必要时依据选定器件的
Vivado 时序报告采用 I/O 寄存器或调整 SPI 频率。
MISO 不能通过随意加入两级同步器后仍按原位索引接收；它与受控 SCLK 相关，
必须结合实际板级延迟验证采样窗口。不要把这些路径全部设为 false path。

未提供假定引脚或通用板级 XDC。上板时用逻辑分析仪核查 32 时钟、CS 间隔、
启动命令与响应、负角速度编码，再接入处理器端缓冲和算法。
