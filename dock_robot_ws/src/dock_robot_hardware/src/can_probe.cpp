// ============================================================
// can_probe.cpp  ——  CAN 总线原始报文嗅探 / 自检工具
//
// 用途：当 ArmHardwareInterface 在 on_activate() 里报告
//       "有 N 个电机初始位置读取失败" 时，用来判断到底是
//         (a) 总线上真的没有任何数据
//         (b) 电机有回帧，但格式不是 can_driver.cpp 假设的
//             DataLen>=5 && Data[0]==0x08
//
// 用法（必须先停掉 ros2_control，否则 USB 设备被占用）：
//   ros2 run dock_robot_hardware can_probe                 # 500kbps，正常模式
//   ros2 run dock_robot_hardware can_probe --baud 1000     # 1Mbps
//   ros2 run dock_robot_hardware can_probe --baud 250      # 250kbps
//   ros2 run dock_robot_hardware can_probe --loopback      # 自发自收自检
//   ros2 run dock_robot_hardware can_probe --seconds 5     # 嗅探时长
// ============================================================

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <thread>

#include "controlcan.h"

// 与 can_driver.hpp 保持一致
static const unsigned int LEFT_IDS[5]  = { 0x01, 0x02, 0x03, 0x04, 0x0A };
static const unsigned int RIGHT_IDS[5] = { 0x06, 0x07, 0x08, 0x09, 0x05 };

static const char *CH_NAME[2] = { "CAN1/左臂", "CAN2/右臂" };

// ZLG 官方波特率配置表（SJA1000, 16MHz）
struct BaudCfg { int kbps; UCHAR t0; UCHAR t1; };
static const BaudCfg BAUDS[] = {
    { 1000, 0x00, 0x14 },
    {  500, 0x00, 0x1C },
    {  250, 0x01, 0x1C },
    {  125, 0x03, 0x1C },
};

static int get_motor_channel(unsigned int id)
{
    for (int i = 0; i < 5; ++i) if (LEFT_IDS[i]  == id) return 0;
    for (int i = 0; i < 5; ++i) if (RIGHT_IDS[i] == id) return 1;
    return -1;
}

static void print_board_info()
{
    VCI_BOARD_INFO info;
    memset(&info, 0, sizeof(info));
    if (VCI_ReadBoardInfo(VCI_USBCAN2, 0, &info) != STATUS_OK) {
        printf("  VCI_ReadBoardInfo 失败\n");
        return;
    }
    printf("  板卡信息: hw=%X.%02X fw=%X.%02X dr=%X.%02X in=%X.%02X can_Num=%u\n",
           info.hw_Version >> 8, info.hw_Version & 0xFF,
           info.fw_Version >> 8, info.fw_Version & 0xFF,
           info.dr_Version >> 8, info.dr_Version & 0xFF,
           info.in_Version >> 8, info.in_Version & 0xFF,
           info.can_Num);
    printf("            序列号='%.20s'  类型='%.40s'\n",
           info.str_Serial_Num, info.str_hw_Type);
}

static bool init_channel(int ch, UCHAR t0, UCHAR t1, UCHAR mode)
{
    VCI_INIT_CONFIG cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.AccCode = 0x00000000;
    cfg.AccMask = 0xFFFFFFFF;   // 接收全部报文
    cfg.Filter  = 0;            // 单滤波
    cfg.Timing0 = t0;
    cfg.Timing1 = t1;
    cfg.Mode    = mode;         // 0=正常, 1=只听, 2=自发自收

    if (VCI_InitCAN(VCI_USBCAN2, 0, ch, &cfg) != STATUS_OK) {
        printf("  VCI_InitCAN(ch%d) 失败\n", ch);
        return false;
    }
    if (VCI_StartCAN(VCI_USBCAN2, 0, ch) != STATUS_OK) {
        printf("  VCI_StartCAN(ch%d) 失败\n", ch);
        return false;
    }
    VCI_ClearBuffer(VCI_USBCAN2, 0, ch);
    return true;
}

static bool send_frame(int ch, unsigned int id, const BYTE *data, BYTE len)
{
    VCI_CAN_OBJ f;
    memset(&f, 0, sizeof(f));
    f.ID         = id;
    f.SendType   = 1;          // 单次发送
    f.RemoteFlag = 0;
    f.ExternFlag = 0;          // 标准帧
    f.DataLen    = len;
    for (int i = 0; i < len && i < 8; ++i) f.Data[i] = data[i];

    ULONG n = VCI_Transmit(VCI_USBCAN2, 0, ch, &f, 1);
    if (n != 1) {
        printf("  发送失败 ch=%d id=0x%03X ret=%u\n", ch, id, n);
        return false;
    }
    return true;
}

// ------------------------------------------------------------
// 自发自收自检：证明 USBCAN 盒子 + libcontrolcan 本身是好的
// ------------------------------------------------------------
static int run_loopback(const BaudCfg &bc)
{
    printf("=== 自发自收自检 (loopback)  %d kbps ===\n", bc.kbps);
    if (VCI_OpenDevice(VCI_USBCAN2, 0, 0) != STATUS_OK) {
        printf("VCI_OpenDevice 失败：设备没插 / 被占用 / 没权限\n");
        return 1;
    }
    print_board_info();

    int ret = 0;
    if (!init_channel(0, bc.t0, bc.t1, 2 /*自发自收*/)) {
        printf("通道初始化失败\n");
        ret = 1;
    } else {
        const BYTE payload[8] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88 };
        send_frame(0, 0x123, payload, 8);

        VCI_CAN_OBJ buf[10];
        ULONG n = VCI_Receive(VCI_USBCAN2, 0, 0, buf, 10, 500);
        if (n > 0) {
            printf("  ✔ 收到自己发出的 %u 帧，USBCAN 设备与驱动库工作正常：\n", n);
            for (ULONG i = 0; i < n; ++i) {
                printf("     ID=0x%03X DLC=%u Data=", buf[i].ID, buf[i].DataLen);
                for (int k = 0; k < buf[i].DataLen && k < 8; ++k) printf("%02X ", buf[i].Data[k]);
                printf("\n");
            }
        } else {
            printf("  ✘ 没收到自己的帧 → USBCAN 盒子/驱动库有问题，或波特率配置无效\n");
            ret = 1;
        }
    }

    VCI_CloseDevice(VCI_USBCAN2, 0);
    return ret;
}

// ------------------------------------------------------------
// 正常模式嗅探：发出查询帧，然后把两个通道上收到的所有帧原样打出来
// ------------------------------------------------------------
static int run_sniff(const BaudCfg &bc, int seconds)
{
    printf("=== CAN 总线嗅探  %d kbps  %d 秒 ===\n", bc.kbps, seconds);
    if (VCI_OpenDevice(VCI_USBCAN2, 0, 0) != STATUS_OK) {
        printf("VCI_OpenDevice 失败：设备没插 / 被 ros2_control 占用 / 没权限\n");
        return 1;
    }
    print_board_info();

    bool ok0 = init_channel(0, bc.t0, bc.t1, 0);
    bool ok1 = init_channel(1, bc.t0, bc.t1, 0);
    if (!ok0 || !ok1) {
        printf("通道初始化失败 (ch0=%d ch1=%d)\n", ok0, ok1);
        VCI_CloseDevice(VCI_USBCAN2, 0);
        return 1;
    }

    // 与驱动一致的查询帧：DLC=1, Data[0]=0x08
    auto send_queries = []() {
        const BYTE q = 0x08;
        for (int i = 0; i < 5; ++i) send_frame(0, LEFT_IDS[i],  &q, 1);
        for (int i = 0; i < 5; ++i) send_frame(1, RIGHT_IDS[i], &q, 1);
    };

    send_queries();

    std::map<unsigned int, int> frame_count;   // ID -> 帧数
    std::map<unsigned int, std::string> first_data;
    int total = 0;
    auto t_start = std::chrono::steady_clock::now();
    auto t_last_query = t_start;

    VCI_CAN_OBJ buf[100];
    while (true) {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - t_start).count();
        if (elapsed > seconds) break;

        // 每 50ms 重发一次查询
        if (std::chrono::duration<double>(now - t_last_query).count() > 0.05) {
            send_queries();
            t_last_query = now;
        }

        for (int ch = 0; ch < 2; ++ch) {
            ULONG n = VCI_Receive(VCI_USBCAN2, 0, ch, buf, 100, 0);
            for (ULONG i = 0; i < n; ++i) {
                ++total;
                frame_count[buf[i].ID]++;

                char line[64];
                int off = snprintf(line, sizeof(line), "DLC=%u Data=", buf[i].DataLen);
                for (int k = 0; k < buf[i].DataLen && k < 8; ++k)
                    off += snprintf(line + off, sizeof(line) - off, "%02X ", buf[i].Data[k]);

                if (first_data.find(buf[i].ID) == first_data.end())
                    first_data[buf[i].ID] = line;

                // 只打印前 30 帧，避免刷屏
                if (total <= 30)
                    printf("  [%s] ID=0x%03X %s\n", CH_NAME[ch], buf[i].ID, line);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    printf("\n---------------- 统计 ----------------\n");
    printf("总收帧数: %d\n", total);
    if (total == 0) {
        printf("结论: 总线上没有任何报文。检查顺序：\n");
        printf("  1) 电机电源是否打开（最重要）\n");
        printf("  2) 波特率是否匹配：本工具可用 --baud 1000 / 500 / 250 逐个试\n");
        printf("  3) CAN_H / CAN_L 是否接反、是否共地\n");
        printf("  4) 120Ω 终端电阻是否装上\n");
        printf("  5) 左臂是否真的接在 CAN1、右臂接在 CAN2\n");
    } else {
        printf("按 ID 统计:\n");
        for (auto &kv : frame_count) {
            int ch = get_motor_channel(kv.first);
            const char *who = (ch < 0) ? "未知ID" : CH_NAME[ch];
            printf("  ID=0x%03X  收到 %4d 帧  (%s)  首帧 %s\n",
                   kv.first, kv.second, who, first_data[kv.first].c_str());
        }
        printf("\n对比 can_driver.cpp 的解析条件 (DataLen>=5 && Data[0]==0x08)：\n");
        printf("  上面每条的首帧 Data 第 1 个字节是 08 且 DLC>=5 → 驱动能认，问题在别处\n");
        printf("  否则 → 电机回帧格式与驱动假设不符，需要改 parse_can_message()\n");
    }

    VCI_CloseDevice(VCI_USBCAN2, 0);
    return total > 0 ? 0 : 2;
}

int main(int argc, char **argv)
{
    int baud_kbps = 500;
    int seconds   = 3;
    bool loopback = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--baud" && i + 1 < argc)          baud_kbps = atoi(argv[++i]);
        else if (a == "--seconds" && i + 1 < argc)  seconds   = atoi(argv[++i]);
        else if (a == "--loopback")                 loopback  = true;
        else if (a == "-h" || a == "--help") {
            printf("用法: can_probe [--baud 1000|500|250|125] [--seconds N] [--loopback]\n");
            return 0;
        }
    }

    const BaudCfg *bc = nullptr;
    for (const auto &b : BAUDS) if (b.kbps == baud_kbps) bc = &b;
    if (!bc) {
        printf("不支持的波特率 %d，可选: 1000 500 250 125\n", baud_kbps);
        return 1;
    }

    return loopback ? run_loopback(*bc) : run_sniff(*bc, seconds);
}
