# <a id="overview"></a>项目总览

本项目面向 **2026 年全国大学生嵌入式芯片与系统设计竞赛 FPGA 创新设计赛道**安路选题一“基于安路 EG4S20 的 HDMI 多媒体播放系统”，以 HX4S20C 开发板上的 `EG4S20BG256` 为核心，在单片 FPGA 上完成 TF 卡资源管理、图片处理、语音讲解、字幕叠加与 HDMI 音视频输出等功能。

作品以博物馆 **“中国航天事业70周年”主题海报展区**为应用场景，将静态海报转化为可浏览、可听讲、可导览的科普展陈内容。观众可按键切换海报、通过缩略图选择主题，结合语音讲解、逐字高亮字幕及全文页了解航天历程；焦点放大与自动运镜引导阅读重点，讲解时自动压低配乐，使图文细读与听讲相互配合。

系统支持环境光亮度联动和红外感应：观众近距离阅读时可暂停自动轮播，离开后恢复展播。程序经 Flash 固化后上电运行，设置可掉电保存，更新 TF 卡素材即可更换主题，适用于科普展区的自助听讲与无人值守展播，减少重复讲解和人工值守需求。

本仓库提供 RTL 源码、TD 工程、实现结果、模块测试平台及素材制作脚本，并附带 **16 组项目所用的示例图片、配乐和讲解字幕资源**。

**核心功能：**

- **图片播放与构图**：FAT32 资源扫描、BMP 解析、自适应缩放，支持最多 64 张图片
- **缩略图与智能导览**：2×2 总览、焦点选择、局部放大与平滑运镜
- **配乐与讲解字幕**：16 组素材关联，讲解自动压低配乐，字幕随采样进度更新并逐字高亮
- **图层叠加与转场**：参数 OSD、字幕与频谱融合，支持淡入淡出和随机方块溶解
- **实时音频可视化**：32 个 Goertzel 分析频点，生成 64 根频谱柱及峰值保持效果
- **交互与环境适配**：亮度、对比度、音量和轮播周期调节，支持光照及红外感应联动
- **固化与设置恢复**：上电运行、离线播放，参数写入 Flash 并在重新上电后恢复

---

<a id="structure"></a>

## 📁 目录结构

```text
.
├── README.md
├── prj/                            # Tang Dynasty 工程与实现结果
│   ├── tf_hdmi_media_player.al      # 工程入口，顶层模块为 top
│   └── tf_hdmi_media_player_Runs/
│       ├── best_result/            # 随仓库提供的下载比特流与结果摘要
│       └── phy_1/                  # 布局布线资源报告与完整时序报告
├── Report/                         # 技术文档
├── src/
│   ├── constraint/                 # 引脚约束 pin.adc、时序约束 timing.sdc
│   └── rtl/
│       ├── top/                    # 顶层集成
│       ├── common/                 # 复位同步、CDC、帧边界事件
│       ├── ip/                     # PLL、异步 FIFO、SDRAM 控制器
│       ├── sd_storage/             # SD SPI、FAT32、图片／音频／字幕加载
│       ├── frame_buffer/           # SDRAM 仲裁、四 Bank 帧缓存与预取
│       ├── control/                # 启动、按键、播放策略、导览与设置保存
│       ├── display/                # 视频时序、缩放、转场、缩略图与运镜
│       ├── osd/                    # 字幕、字库、参数信息与频谱叠加
│       ├── audio/                  # 流式播放、采样节拍、混音与音源选择
│       ├── hdmi/                   # 音频封包、TMDS 编码与串行输出
│       └── peripheral/             # 数码管、光照与红外传感器
├── sim/                            # 按子系统分类的 Verilog testbench
├── tools/
│   ├── image/                      # 图片转 BMP 与拷卡辅助工具
│   ├── audio/                      # 配乐／讲解转换、字幕生成与检查
│   ├── font/                       # HZK32 字库生成
│   ├── tour/                       # 导览焦点表与运镜微程序工具
│   └── spectrum/                   # 频谱参数与配色生成工具
└── TF_source/                      # 示例素材，五个子文件夹可直接拷入 TF 卡
    ├── 图片/
    ├── 背景音乐/
    ├── 字幕及配音/
    ├── 字库/
    └── 运镜/
```

常用入口：[顶层源码](src/rtl/top/top_tf_hdmi_audio.v) · [引脚约束](src/constraint/pin.adc) · [时序约束](src/constraint/timing.sdc) · [TD 工程](prj/tf_hdmi_media_player.al)

---

<a id="hardware"></a>

## 🔧 硬件与开发环境

### 硬件平台

| 组件                 | 型号／要求                                                   | 用途                               |
| :------------------- | :----------------------------------------------------------- | :--------------------------------- |
| FPGA 开发板          | HX4S20C，安路 `EG4S20BG256`                                | 系统控制与音视频处理               |
| SDRAM                | EG4S20 配套 `2M×32 bit`，容量 8 MiB                       | 图片帧缓存与缩略图缓存             |
| TF 卡                | FAT32 文件系统                                               | 存放图片、音频、字幕、字库及焦点表 |
| HDMI 显示设备        | 支持 640×480 输入，具备音频播放能力或外接 HDMI 音频接收设备 | 接收画面与声音                     |
| 下载器               | 与开发板配套的 JTAG 下载器                                   | 调试下载与程序固化                 |
| 光照传感器（可选）   | GY-302／BH1750，I²C                                         | 环境光亮度联动                     |
| 红外检测模块（可选） | YL-62，数字输出                                              | 接近检测与轮播策略联动             |

基础图片与音频播放可先使用板载硬件完成，再接入传感器验证环境交互。

<details>
<summary>外接传感器连接</summary>

| 模块   | 信号     | 开发板接口            | FPGA 引脚 |
| :----- | :------- | :-------------------- | :-------- |
| GY-302 | VCC／GND | J1-29（3.3 V）／J1-30 | —        |
| GY-302 | SCL      | J1-31                 | M15       |
| GY-302 | SDA      | J1-32                 | L16       |
| YL-62  | VCC／GND | J2-29（3.3 V）／J2-30 | —        |
| YL-62  | OUT      | J2-31                 | J12       |

两个模块均使用 3.3 V 供电并与开发板共地。GY-302 的默认 7 位 I²C 地址为 `0x23`。完整引脚分配见 [pin.adc](src/constraint/pin.adc)。

</details>

---

### 开发工具

| 工具                 | 版本／用途                                            |
| :------------------- | :---------------------------------------------------- |
| Anlogic Tang Dynasty | 工程记录版本 `6.2.168116`，用于综合、布局布线与下载 |
| ModelSim             | Verilog 仿真测试                                      |
| Python               | 建议 3.10 或更新版本，用于素材制作                    |

---

<a id="quick-start"></a>

## 🚀 快速开始

### 1. 获取工程

下载本仓库 ZIP 或克隆仓库，在本地保留完整目录结构。工程入口为：

```text
prj/tf_hdmi_media_player.al
```

以下工具命令均从仓库根目录执行，示例采用 PowerShell。

### 2. 准备 TF 卡

使用 FAT32 格式的 TF 卡，将 `TF_source/` 中的 **五个文件夹整体复制到 TF 卡根目录**，保留原有目录结构和文件名即可。

```text
TF 卡根目录/
├── 背景音乐/
│   └── TRACK1.PCM … TRACK16.PCM       # 48 kHz 背景音乐
├── 图片/
│   └── PIC1.BMP … PIC16.BMP           # 示例海报
├── 运镜/
│   └── FOCUS.BIN                      # 导览焦点表
├── 字库/
│   └── HZK32                          # 32×32 灰度字库，无扩展名
└── 字幕及配音/
    ├── TRACK1.VOC … TRACK16.VOC       # 24 kHz 讲解人声
    └── TRACK1.LRC … TRACK16.LRC       # 讲解字幕与时间轴
```

系统会自动扫描各子目录中的素材，并按文件编号关联图片、配乐、讲解和字幕。自定义素材时，按[素材制作与文件约定](#assets)替换对应文件即可。

### 3. 下载并运行

首次体验可直接使用随仓库提供的[比特流](prj/tf_hdmi_media_player_Runs/best_result/tf_hdmi_media_player.bit)：

1. 插入 TF 卡，连接 HDMI 显示设备、JTAG 下载器及开发板电源。
2. 在 TD 下载工具中选择正确器件和 JTAG 链，将 `.bit` 文件下载至 FPGA。
3. 将 `SW1` 拨上，`SW2`、`SW3`、`SW4` 拨下，以全屏模式播放 TF 卡素材。
4. 使用 `KEY2` 切图，`KEY4` 调节音量；完整操作见[交互操作](#controls)。

未保存配置时，默认开启自动轮播，周期为 5 s；已有有效 Flash 配置时按保存值恢复。需要上电自动运行时，使用开发板配套的配置 Flash 固化流程。

> 配置保存使用 W25Q64 的最后两个 4 KiB 扇区，起始地址分别为 `0x7FE000`、`0x7FF000`。制作固化镜像或分配其他 Flash 数据时，应保留这一区域。

### 4. 从源码构建

使用 Tang Dynasty 打开 [tf_hdmi_media_player.al](prj/tf_hdmi_media_player.al)，确认器件为 `EG4S20BG256`、顶层模块为 `top`，依次执行综合、布局布线和比特流生成，再下载新生成的结果。

工程源文件使用 `../src/` 相对路径。若首次打开时 TD 提示历史工程目录迁移，请确认当前 `prj/` 路径并重新生成输出。编译生成物中的历史绝对路径不作为源码引用入口。

---

<a id="controls"></a>

## 🎮 交互操作

### 按键与拨码

| 操作                        | 默认浏览模式下的功能                                         |
| :-------------------------- | :----------------------------------------------------------- |
| `KEY1`                    | 复位系统                                                     |
| `KEY2` 单击／长按约 1 s   | 下一张／上一张                                               |
| `KEY3` 单击               | 开启／关闭自动轮播                                           |
| `KEY3` 长按约 0.5 s       | 显示／隐藏底部跑马灯和频谱                                   |
| `KEY4` 单击／长按约 0.5 s | 循环调节音量／切换轮播周期                                   |
| `KEY2 + KEY3` 同按约 3 s  | 保存轮播周期、亮度偏移、对比度、音量、自动轮播及底部显示开关 |
| `KEY2 + KEY4` 同按        | 进入／退出焦点导览；导览中单击 `KEY2` 跳至下一焦点         |
| `KEY3 + KEY4` 同按        | 进入／退出画面调节模式                                       |
| `SW1` 拨上                | 启用 TF 卡背景音乐；拨下时使用内置音源                       |
| `SW2` 拨上                | 暂停讲解与字幕进度；拨下继续                                 |
| `SW3` 拨上                | 显示讲解全文页；拨下返回正常叠加                             |
| `SW4` 拨上／拨下          | 进入 2×2 缩略图总览／确认焦点并返回全屏                     |

缩略图模式下，`KEY2` 单击／长按用于向后／向前选择图片。图像未加载完成时由控制层管理提交，显示切换在帧边界生效。

<details>
<summary>画面调节模式与环境交互</summary>

| 调节模式内的操作        | 功能                   |
| :---------------------- | :--------------------- |
| `KEY3`／`KEY4` 单击 | 当前参数增加／减小     |
| `KEY3` 长按           | 在亮度与对比度之间切换 |
| `KEY4` 长按           | 当前参数恢复默认值     |
| 约 5 s 无操作           | 自动退出调节模式       |

模式内的 `KEY3`／`KEY4` 优先用于参数调节；退出后恢复轮播、底部显示、音量与周期控制。

光照传感器用于调整画面亮度基准，按键可进一步修正亮度偏移。红外模块检测到遮挡时切换为手动浏览，移开后恢复自动轮播；`KEY3` 仍可手动控制轮播状态。

</details>

---

<a id="design"></a>

## 📐 核心设计

### 系统数据流

图片、音频和字幕共用 TF 卡读取通道，分路处理后通过 HDMI 输出；按键、拨码及传感器统一控制播放、显示和设置保存。

```text
┌──────────────────────────────────────────────────────────────────────────────┐
│                       HDMI Multimedia Playback System                        │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│                               TF card (FAT32)                                │
│                                       │                                      │
│                   Resource scan / SPI access / Arbitration                   │
│                                       │                                      │
│                     ┌─────────────────┴─────────────────┐                    │
│                     ▼                                   ▼                    │
│    ┌────────────────────────────────┐  ┌────────────────────────────────┐    │
│    │ Video pipeline                 │  │ Audio & subtitle pipeline      │    │
│    │                                │  │                                │    │
│    │ BMP decode / Smart composition │  │ PCM music / VOC narration      │    │
│    │ 4-bank SDRAM / Image prefetch  │  │ Audio mixing / Music ducking   │    │
│    │ Thumbnail / Guided camera      │  │ LRC subtitles / Sample sync    │    │
│    │ Fade / Block dissolve          │  │ Goertzel spectrum              │    │
│    └────────────────────────────────┘  └────────────────────────────────┘    │
│                     │                                   │                    │
│                     └─────────────────┬─────────────────┘                    │
│                                       ▼                                      │
│                Layer blending / Audio packets / TMDS encoding                │
│                                       │                                      │
│                                       ▼                                      │
│                          HDMI video & audio output                           │
│                                                                              │
├──────────────────────────────────────────────────────────────────────────────┤
│  Control: Keys / Switches / Ambient light / IR sensing                       │
│  Settings: Dual-slot SPI Flash storage / Power-on restore                    │
└──────────────────────────────────────────────────────────────────────────────┘
```

### 1. 多路存储调度与四 Bank 帧缓存

- FAT32 扫描生成资源清单，图片、配乐、讲解和字库经扇区仲裁共享 SPI 接口
- 四 Bank 帧缓存配合图片预取，使当前显示与后续素材加载并行
- 显示侧采用异步 FIFO 和行预取，新画面在帧边界提交，保证完整更新

### 2. 自适应构图与智能导览

- 根据源图尺寸选择整数放大、居中填充或等比例适配
- 支持 2×2 缩略图总览、焦点选择及全屏切换，按 `FOCUS.BIN` 配置平滑运镜
- 缩放与导览共用几何信息，运镜采用微程序和分时运算，降低重复计算开销

### 3. 采样驱动的讲解字幕同步

- 字幕随实际消费的讲解音频采样推进，暂停时同步停走
- `TRACKn.LRC` 提供标题、导语和时间标签，`HZK32` 提供 32×32 灰度字形
- 用字去重后缓存，通过握手更新资源，实现滚动字幕、逐字高亮及全文排版

### 4. 单 DSP、单 BRAM 的 Goertzel 频谱引擎

- 分析实际混音数据，在约 `50 Hz～12 kHz` 范围计算 32 个频点
- 分时复用一个 18×18 乘法器及一块 9K BRAM，减少并行运算开销
- 相邻频点插值生成 64 根显示柱，叠加峰值保持、衰减与配色流动；独立分析频点为 32 个

### 5. 混音与 HDMI 音频封包

- 48 kHz 配乐与 24 kHz 讲解接入统一的 48 kHz 输出链路，讲解时压低配乐，转场时联动淡化
- 增益计算与样本缩放共用一个 16×9 乘法器，音频样本采用有效／就绪握手
- 在消隐期间调度音频采样包、ACR 和 Audio InfoFrame，经 BCH 校验、TERC4／TMDS 编码及 10:1 串行化输出

### 6. 跨时钟域与时序约束

- 各时钟域独立同步释放复位，状态、控制载荷和连续数据流分别采用同步器、握手及异步 FIFO
- 保留同一 PLL 输出之间的相关时钟分析，按同步器、握手载荷和 Gray 指针分别设置约束
- 结合功能仿真检查协议行为、静态时序分析检查路径时序，详见 [common/](src/rtl/common) 和 [timing.sdc](src/constraint/timing.sdc)

---

<a id="results"></a>

## 📊 性能与资源

### 系统参数

| 项目          | 参数／实现结果                                                      |
| :------------ | :------------------------------------------------------------------ |
| 视频输出      | 640×480，25 MHz 像素时钟，刷新率约 59.94 Hz                       |
| HDMI 音频输出 | 48 kHz、16 bit PCM；单声道素材送入双声道输出链路                    |
| 图片格式      | 24 bit、无压缩 BMP，宽与高各支持 1～4095 像素                       |
| 资源数量      | 当前最多支持播放64 张图片                                           |
| 示例资源      | 16 张图片、16 份配乐、16 组讲解与字幕                               |
| 频谱分析      | 32 个分析频点，约 50 Hz～12 kHz；64 根显示柱                        |
| 焦点导览      | 单图最多 4 个焦点，支持位置与缩放倍率配置                           |
| 导览运镜      | 单次移动 64 帧，约 1.08 s；默认停留 150 帧，约 2.52 s（按帧数换算） |

### 布局布线资源占用

| 资源     | 使用量 | 可用量 |  占用率 |
| :------- | -----: | -----: | ------: |
| LUT      | 16,638 | 19,600 |  84.89% |
| 寄存器   |  8,871 | 19,600 |  45.26% |
| 9K BRAM  |     48 |     64 |  75.00% |
| 32K BRAM |     16 |     16 | 100.00% |
| DSP      |     21 |     29 |  72.41% |
| PLL      |      1 |      4 |  25.00% |

### 时钟与时序结果

| 时钟              |               频率 | 用途                     |
| :---------------- | -----------------: | :----------------------- |
| `sys_clk`       |             50 MHz | 应用控制、资源管理与外设 |
| `mem_clk`       |             40 MHz | SDRAM 访问               |
| `mem_clk_shift` | 40 MHz，144° 相移 | SDRAM 接口               |
| `pixel_clk`     |             25 MHz | 视频、字幕与音频链路     |
| `pixel_clk_5x`  |            125 MHz | HDMI 串行化              |

建立时间最差裕量 **+0.892 ns**，保持时间最差裕量 **+0.036 ns**。
<a id="assets"></a>

## 🛠️ 素材制作与文件约定

### 文件命名与格式

| 文件                            | 格式要求                                           | 对应关系                   |
| :------------------------------ | :------------------------------------------------- | :------------------------- |
| `PIC1.BMP`～`PIC64.BMP`     | 24 bit 无压缩 BMP                                  | 按图片编号组织             |
| `TRACK1.PCM`～`TRACK16.PCM` | 48 kHz、16 bit、有符号小端、单声道、无文件头       | 与 `PICn.BMP` 对应       |
| `TRACK1.VOC`～`TRACK16.VOC` | 24 kHz、16 bit、有符号小端、单声道、无文件头       | 与同编号字幕共同使用       |
| `TRACK1.LRC`～`TRACK16.LRC` | GB2312 文本，含时间标签；可含 `[ti:]`、`[ar:]` | 与同编号讲解对应           |
| `HZK32`                       | 工具生成的 32×32、2 bit 灰度字库                  | `字库/` 文件夹，无扩展名 |
| `FOCUS.BIN`                   | 工具生成的 1024 字节二进制焦点表                   | 64 张图，每张最多 4 个焦点 |

`.PCM`、`.VOC` 是转换后的裸音频，不能通过修改 MP3／WAV 扩展名得到。`.LRC` 应先用检查工具确认编码、容量与用字范围。

### 常用工具

| 工具                                                            | 用途                                       |
| :-------------------------------------------------------------- | :----------------------------------------- |
| [convert_images_to_bmp.py](tools/image/convert_images_to_bmp.py) | 将图片转换为 640×480、24 bit BMP          |
| [make_audio_corpus.py](tools/audio/make_audio_corpus.py)         | 每次最多转换 4 份背景音乐                  |
| [make_voice.py](tools/audio/make_voice.py)                       | 每次最多转换 4 份讲解录音                  |
| [make_narration.py](tools/audio/make_narration.py)               | 根据录音辅助生成讲解音频及字幕时间轴       |
| [check_lrc.py](tools/audio/check_lrc.py)                         | 检查／转换字幕编码，核对字幕容量及录音时长 |
| [make_font.py](tools/font/make_font.py)                          | 从本地字体生成 `HZK32`                   |
| [make_focus_bin.py](tools/tour/make_focus_bin.py)                | 将焦点坐标文本转换为 `FOCUS.BIN`         |

<details>
<summary>图片、音频与字幕制作示例</summary>

安装图片与字库工具依赖：

```powershell
python -m pip install Pillow
```

转换单张图片：

```powershell
python tools/image/convert_images_to_bmp.py poster.jpg PIC1.BMP
```

该脚本统一输出 640×480。需要使用 FPGA 自适应构图时，可保留原始分辨率，仅转换为 RGB、24 bit BMP：

```powershell
python -c "from PIL import Image; Image.open('poster.jpg').convert('RGB').save('PIC1.BMP')"
```

转换配乐与讲解录音；处理 MP3 等格式时可安装 FFmpeg 并加入 `PATH`：

```powershell
python tools/audio/make_audio_corpus.py music.wav -o output/bgm --normalize
python tools/audio/make_voice.py narration.wav -o output/voice --normalize
```

如果已有字幕文本，检查后转换为板上使用的编码：

```powershell
python tools/audio/check_lrc.py narration_utf8.lrc --out TRACK1.LRC --voc output/voice/TRACK1.VOC
```

需要从录音辅助生成字幕时，另外安装 `faster-whisper`：

```powershell
python -m pip install faster-whisper
python tools/audio/make_narration.py narration.wav --track 1 -o output/narration
```

首次使用语音识别工具需要下载模型；生成后应人工校对文字和时间轴。该步骤在 PC 上完成，FPGA 播放不依赖语音识别模型。

</details>

---

<details>
<summary>按四个一批准备 16 组素材</summary>

现有音频工具可以反复运行，每批处理最多 4 份素材。各批次使用独立输出目录，完成后再按实际图片编号统一命名。

例如，先后生成第 1～4 首和第 5～8 首背景音乐：

```powershell
python tools/audio/make_audio_corpus.py music1.wav music2.wav music3.wav music4.wav -o output/bgm_01_04
python tools/audio/make_audio_corpus.py music5.wav music6.wav music7.wav music8.wav -o output/bgm_05_08
```

每次运行默认生成 `TRACK1.PCM`～`TRACK4.PCM`，第二批在自己的目录中重新编号：

```powershell
1..4 | ForEach-Object {
    Rename-Item -LiteralPath ("output/bgm_05_08/TRACK{0}.PCM" -f $_) -NewName ("TRACK{0}.PCM" -f ($_ + 4))
}
```

第 9～12 首、第 13～16 首分别按偏移量 `8`、`12` 编号。讲解 `.VOC` 与 `.LRC` 采用相同的分批方式，**同一段讲解的音频与字幕必须保持相同编号**；配乐放入 `背景音乐/`，讲解与字幕放入 `字幕及配音/`，再将对应文件夹复制到 TF 卡。

</details>

---

<details>
<summary>生成字库与配置导览焦点</summary>

生成字库；默认使用 Windows 黑体，也可通过 `--ttf` 指定字体文件：

```powershell
python tools/font/make_font.py --out output/font
```

将 `output/font/HZK32` 复制到 TF 卡的 `字库/` 文件夹。字幕使用 GB2312 常用字与 ASCII，字形缓存容量受片上 BRAM 限制。

创建 `FOCUS.TXT`，每行对应一张图，每组为 `x,y,zoom`，最多 4 组，用分号分隔：

```text
PIC1: 160,120,2; 480,360,2
PIC2: 320,240,1.5
```

坐标使用源图片坐标；倍率范围为 `1～4`，按 `0.5` 为步进量化。转换后，将生成的 `FOCUS.BIN` 复制到 TF 卡的 `运镜/` 文件夹：

```powershell
python tools/tour/make_focus_bin.py FOCUS.TXT FOCUS.BIN
```

上例以 640×480 图片说明坐标写法，实际应按自己的图片尺寸与关注区域配置。

</details>

---

## 🤝 贡献

欢迎通过 Issue 反馈问题或提交 Pull Request。

---

## 📄 许可证

本项目采用 [MIT 许可证](https://opensource.org/licenses/MIT)。

---

## 🙏 致谢

在此由衷感谢：

- [安路科技](https://www.anlogic.com/) — FPGA 平台支持
- 全国大学生 FPGA 创新设计竞赛组委会
