#include "define.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <errno.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <linux/tcp.h>
#include <queue>
#include <random>
#include <string>
#include <sys/select.h>
#include <vector>

// for dq_sage
#include <deque>

#define ENABLE_ANOLE_RL false
#define CWND_INIT 10
#define MAX_CWND 10000
#define MIN_CWND 4
#define CWND_THRESHOLD 2 // CWND变化最小值
#define RTT_LENGTH 50
// 内核接口
#define TCP_CWND 38
#define TCP_CWND_USER 39
#define TCP_RTT 40
#define TCP_RTTS 41
#define TCP_RTTMIN 42
// 效用函数参数, 参考 vivace
#define EXPONENT_T 0.9
#define ALPHA 10
#define BETA 1
#define LAMBDA 2 // 11.35
#define MU 0.2   // 0.1 // RTT的震荡范围
// 置信度
#define THETA 0.05
#define ETA_ON 0.8
#define ETA_OFF 0.6
#define DELTA_ETA 0.05 // 每次增长的幅度

// sage参数
#define FLOORING 0
#define ROUNDING 1
#define CEILING 2
#define NONE 3
#define ROUNT_TYPE CEILING
#define SHORT_WIN 10 // 这三个值是short/mid/long,分别对应队列长度
#define MID_WIN 200
#define LONG_WIN 1000
#define WIN_SIZE 500
#define BW_NORM_FACTOR \
    100                      // 100 Mbps will be used to normalize throughput signal
#define MAX_32Bit 0x7FFFFFFF // Make sure we don't have (int32) overflow!

// 全局变量
struct tcp_info info, info_pre;
u32 mss_cache = 1460;
u32 min_rtt = 0;
uint32_t his_rtts[RTT_LENGTH]; // 最近RTT历史记录
struct utility_value
{
    u64 cwnd = 10;
    double utility = 0.0;
    double confidence_val = 1.0;
    bool isright = false;
} u_cl, u_rl, u_prev; // u_prev应该存放的是历史最佳
double u_optimal = 0.0;

/****************************************/
dq_sage<double> rtt_s(SHORT_WIN);
dq_sage<double> rtt_m(MID_WIN);
dq_sage<double> rtt_l(LONG_WIN);
void usage()
{
    DBGMARK(0, 0,
            "./server [port] [path to ddpg.py] [Report Period: 20 msec] [First "
            "Time: 1=yes(learn), 0=no(continue learning), 2=evaluate] [actor "
            "id=0, 1, ...]\n");
}

double mul(double a, double b)
{
    return ((a * b) > MAX_32Bit) ? MAX_32Bit : a * b;
}

void set_cwnd(u64 cwnd, int index)
{
    socklen_t length = sizeof(cwnd);
    if (setsockopt(sock_for_cnt[index], IPPROTO_TCP, TCP_CWND_USER, &cwnd,
                   length) < 0)
        perror("setsockopt: set cwnd\n");
}

void update_his_rtts(int index)
{
    socklen_t length = sizeof(his_rtts);
    if (getsockopt(sock_for_cnt[index], IPPROTO_TCP, TCP_RTTS, &his_rtts[0], &length) < 0)
        perror("getsockopt_his_rtts\n");
}

// double get_delta_rtt(dq_sage<double> &dq)
// {
//     int len =  20; //dq.get_size();
//     len = (len / 2) * 2;
//     double res = 0.0;
//     for (int i = 0; i < (len / 2); i++)
//         res += dq[i];
//     for (int i = (len / 2); i < len; i++)
//         res -= dq[i];
//     if (min_rtt)
//         return (res * 100000.0 / static_cast<double>(min_rtt) / (len / 2));
//     else
//         DBGMARK(DBGSERVER, 0, "ERROR: GET_DELTA_RTT MIN_RTT=0");
//     return 0;
// }

// double get_avg_rtt(dq_sage<double> &dq)
// {
//     // 单位为us
//     return (double)dq.get_avg() * 100000.0;
// }
void print_rtt()
{
    FILE *file = fopen("/home/snow/pantheon-available/src/experiments/log.txt", "a");
    for (int i = 0; i < RTT_LENGTH; i++)
    {
        if (file)
        {
            fprintf(file, "%d ", his_rtts[i]);
        }
    }
    fprintf(file, "\n");
    fclose(file);
}

double get_delta_rtt()
{
    print_rtt();
    double res = 0.0;
    // 让 begin 指向第一个非0的元素
    int begin;
    for (begin = 0; begin < RTT_LENGTH; begin++)
        if (his_rtts[begin])
            break;
    int len = ((RTT_LENGTH - begin) / 2);
    for (int i = begin + len; i < begin + 2 * len; i++)
        res += his_rtts[i];
    for (int i = begin; i < begin + len; i++)
        res -= his_rtts[i];
    if (min_rtt)
        return (res / static_cast<double>(min_rtt) / len);
    else
        DBGMARK(DBGSERVER, 0, "ERROR: GET_DELTA_RTT MIN_RTT=0");
    return 0;
}

double get_avg_rtt()
{
    uint32_t sum = 0;
    // 让 begin 指向第一个非0的元素
    int begin;
    for (begin = 0; begin < RTT_LENGTH; begin++)
        if (his_rtts[begin])
            break;
    for (int i = begin; i < RTT_LENGTH; i++)
        if (his_rtts[i])
            sum += his_rtts[i];

    return static_cast<double>(sum) / (RTT_LENGTH - begin);
    // 内核中的rtt估计采用的是7/8旧+1/8新
}

void update_confidence_val(utility_value &u)
{
    double y = u.utility / u_optimal + THETA;
    u.confidence_val = min(1.0, max(0.5, y * u.confidence_val));
}

bool compareStruct(const struct utility_value &s1,
                   const struct utility_value &s2)
{
    return s1.cwnd < s2.cwnd;
}

bool compareStruct_u(const struct utility_value &s1,
                   const struct utility_value &s2)
{
    return s1.utility < s2.utility;
}

u64 get_optimal_cwnd(int situation)
{
    uint64_t optimal_cwnd = 0;
    vector<struct utility_value> utilities = {u_cl, u_rl, u_prev};
    sort(utilities.begin(), utilities.end(), compareStruct_u);
    return utilities[2].cwnd;
    // sort(utilities.begin(), utilities.end(), compareStruct);
    // switch (situation)
    // {
    // case 0:
    //     optimal_cwnd = utilities[0].cwnd - 1;
    //     break;
    // case 1:
    //     // if (utilities[1].cwnd - utilities[0].cwnd <= CWND_THRESHOLD)
    //     optimal_cwnd = utilities[1].utility > utilities[0].utility
    //                        ? utilities[1].cwnd
    //                        : utilities[0].cwnd;
    //     // else
    //     //     // equation 8
    //     //     optimal_cwnd = (utilities[1].cwnd + utilities[0].cwnd) / 2;
    //     break;
    // case 2:
    //     // if (utilities[2].cwnd - utilities[1].cwnd <= CWND_THRESHOLD)
    //     optimal_cwnd = utilities[2].utility > utilities[1].utility
    //                        ? utilities[2].cwnd
    //                        : utilities[1].cwnd;
    //     // else
    //     //     optimal_cwnd = (utilities[2].cwnd + utilities[1].cwnd) / 2;
    //     break;
    // case 3:
    //     optimal_cwnd = utilities[2].cwnd + 1;
    //     break;
    // default:
    //     break;
    // }
    // return optimal_cwnd;
}

int get_situation()
{
    int res = 0;
    if (!u_cl.isright)
        res++;
    if (!u_rl.isright)
        res++;
    if (!u_prev.isright)
        res++;
    return res;
}

// TODO: 更新函数，rtt的统计方式要改，rtt数量不足要判断
void update_utility_value(uint64_t cwnd, utility_value *u,
                          dq_sage<double> &dq)
{
    update_his_rtts(0);
    double d_rtt = get_delta_rtt(); // get_delta_rtt(dq);
    double a_rtt = get_avg_rtt();   // get_avg_rtt(dq);
    // 单位, 当前单位为MBps
    double rate = static_cast<double>(cwnd) * mss_cache / a_rtt;

    if (d_rtt > 0.1) // 0.02
        u->isright = true;
    else
        u->isright = false;
    FILE *file = fopen("/home/snow/pantheon-available/src/experiments/log.txt", "a");
    if (file)
    {
        fprintf(file, "d_rtt = %.6lf\n", d_rtt);
        fprintf(file, "A = %.6lf\n", ALPHA * pow(rate, EXPONENT_T));
        fprintf(file, "B = %.6lf\n", BETA * rate * max(0.0, d_rtt));
    }
    u->utility = ALPHA * pow(rate, EXPONENT_T) -
                 BETA * rate * max(0.0, d_rtt); // 未添加对RTT绝对值的惩罚
    double ratio = a_rtt / static_cast<double>(min_rtt);
    if (ratio > (1 + MU))
    {
        u->utility -= LAMBDA * rate * ratio;
        if (file)
        {
            fprintf(file, "ratio = %.6lf\n", ratio);
            fprintf(file, "C = %.6lf\n", LAMBDA * rate * ratio);
        }
    }
    if (u->utility > u_optimal)
    {
        u_optimal = u->utility;
    }
    fclose(file);
}

void update_tcp_info(int index)
{
    // 更新tcpinfo的同时更新min_rtt和mss_cache;
    socklen_t length = sizeof(tcp_info);
    if (getsockopt(sock_for_cnt[index], IPPROTO_TCP, TCP_INFO, &info, &length) <
        0)
        perror("getsockopt_tcp_info\n");
    mss_cache = info.tcpi_snd_mss;
    min_rtt = info.tcpi_min_rtt;
    update_his_rtts(index);
}

void *CntThread(void *i)
{
    int reuse = 1;
    if (setsockopt(sock_for_cnt[0], IPPROTO_TCP, TCP_NODELAY, &reuse,
                   sizeof(reuse)))
    {
        perror("setsockopt tcp_nodelay\n");
        return ((void *)0);
    }
    // 状态标志
    bool slow_start_passed = false;

    while (send_traffic)
    {

        update_tcp_info(0);
        if (!slow_start_passed)
        {
            slow_start_passed = (info.tcpi_snd_ssthresh < info.tcpi_snd_cwnd) ? 1 : 0;
            // usleep(min_rtt / 2);
            continue;
        }
        set_cwnd(info.tcpi_snd_cwnd, 0);
        // set_cwnd(target_ratio, 0);
        // usleep(min_rtt / 2);
        // set_cwnd(150, 0);
        // usleep(min_rtt);

        if (info.tcpi_rtt > 0)
        {
            // 1. evaluation stage
            // 先更新prev，再更新u_rl和u_cl
            if (u_prev.utility == 0)
            {
                u_prev.cwnd = u_cl.cwnd < u_rl.cwnd ? u_cl.cwnd : u_rl.cwnd;
            }
            // 当前rl的cwnd存在target_ratio中，开始挑选最佳CWND
            u_rl.cwnd = target_ratio;
            u_cl.cwnd = info.tcpi_snd_cwnd;
            struct utility_value *first_run = u_cl.cwnd < u_rl.cwnd ? &u_cl : &u_rl;
            struct utility_value *second_run = u_cl.cwnd < u_rl.cwnd ? &u_rl : &u_cl;
            // TODO: rtt_m尝试换多个dq测试
            u32 ONE_WAY_DELAY = static_cast<u32>(get_avg_rtt() / 2);
            // TODO: rtt_m换0->估计上个RTT收到了多少个数据包，再取平均值
            if (first_run != NULL && second_run != NULL)
            {
                set_cwnd(first_run->cwnd, 0);
                usleep(ONE_WAY_DELAY);
                set_cwnd(second_run->cwnd, 0);
                ONE_WAY_DELAY = static_cast<u32>(get_avg_rtt() / 2);
                usleep(ONE_WAY_DELAY);
                set_cwnd(u_prev.cwnd, 0);
                update_utility_value(u_prev.cwnd, &u_prev, rtt_m); // 1RTT时取prev的RTT
                ONE_WAY_DELAY = static_cast<u32>(get_avg_rtt() / 2);
                usleep(ONE_WAY_DELAY);
                update_utility_value(first_run->cwnd, first_run,
                                     rtt_m); // 1.5RTT取first_run的u
                ONE_WAY_DELAY = static_cast<u32>(get_avg_rtt() / 2);
                usleep(ONE_WAY_DELAY);
                update_utility_value(second_run->cwnd, second_run,
                                     rtt_m); // 2 RTT取second_run的u
            }
            else
            {
                DBGPRINT(DBGSERVER, 0, "Get pointer failed!");
                return ((void *)0);
            }

            // 2. calculating the optimal rate
            int situation = get_situation();
            u64 optimal_cwnd = get_optimal_cwnd(situation);
            // u_prev.cwnd = optimal_cwnd;

            // PROBING
            // 3. check whether enter probing or acceleration stage
            update_confidence_val(u_cl);
            update_confidence_val(u_rl);
            /*****************************/
            FILE *file = fopen("/home/snow/pantheon-available/src/experiments/log.txt", "a");
            if (file)
            {
                fprintf(file, "Current rtt = %u\n", ONE_WAY_DELAY * 2);
                fprintf(file, "cwnd of cl, rl, prev are : %lu, %lu, %lu\n", u_cl.cwnd, u_rl.cwnd, u_prev.cwnd);
                fprintf(file, "isright of cl, rl, prev are : %lu, %lu, %lu\n", u_cl.isright, u_rl.isright, u_prev.isright);
                fprintf(file, "optimal_cwnd is : %lu\n", optimal_cwnd);
                // fprintf(file, "utility of optimal, cl, rl, prev are : %.2f,%.2f,%.2f,%.2f\n", u_optimal, u_cl.utility, u_rl.utility, u_prev.utility);
                fprintf(file, "utility of, cl, rl, prev are : %.2f,%.2f,%.2f\n", u_cl.utility, u_rl.utility, u_prev.utility);
                fprintf(file, "confidence val of optimal, cl, rl, prev are : %.2f,%.2f\n", u_cl.confidence_val, u_rl.confidence_val);
                fprintf(file, "----------------------------------------------------------------\n");
            }
            fclose(file);
            u_prev.cwnd = optimal_cwnd;
            // set_cwnd(optimal_cwnd, 0);
            // uint32_t avg_rtt = static_cast<uint32_t>(get_avg_rtt());
            // usleep(avg_rtt);
            if (u_cl.confidence_val >= ETA_OFF && u_rl.confidence_val >= ETA_OFF)
            {
                // PROBING
                set_cwnd(optimal_cwnd, 0);
                uint32_t avg_rtt = static_cast<uint32_t>(get_avg_rtt());
                usleep(2 * avg_rtt);
                // get_tcp_info(0);
                // pre_bytes_acked = static_cast<double>(info.tcpi_bytes_acked);
                // usleep(avg_rtt);
            }
            else
            {
                // ACCELERATION
                bool flag_cl = u_cl.confidence_val >= ETA_OFF;
                bool flag_rl = u_rl.confidence_val >= ETA_OFF;
                set_cwnd(optimal_cwnd, 0);
                double avg_rtt =
                    static_cast<uint32_t>(get_avg_rtt() * 1.5); // 直接运行1.5个RTT
                while (!flag_cl || !flag_rl)
                {
                    usleep(avg_rtt);
                    if (!flag_cl)
                    {
                        u_cl.confidence_val += DELTA_ETA;
                        flag_cl = (u_cl.confidence_val >= ETA_ON);
                    }
                    if (!flag_rl)
                    {
                        u_rl.confidence_val += DELTA_ETA;
                        flag_rl = (u_rl.confidence_val >= ETA_ON);
                    }
                    // get_min_rtt(0);
                    // get_his_rtts(0);
                    // TODO: acc阶段的机制还要更新
                    avg_rtt = static_cast<uint32_t>(get_avg_rtt() * 1.5);
                    if (static_cast<double>(avg_rtt) / min_rtt > 1 + MU)
                    {
                        set_cwnd(--optimal_cwnd, 0);
                    }
                    else
                    {
                        set_cwnd(++optimal_cwnd, 0);
                    }
                }
            }
        }
    }
    shmdt(shared_memory);
    shmctl(shmid, IPC_RMID, NULL);
    shmdt(shared_memory_rl);
    shmctl(shmid_rl, IPC_RMID, NULL);
    return ((void *)0);
}

// DataThread中发送数据
void *DataThread(void *info)
{
    cFlow *flow = (cFlow *)info;
    int sock_local = flow->flowinfo.sock;
    char *src_ip;
    char write_message[BUFSIZ + 1];
    char read_message[1024] = {0};
    int len;
    char *savePtr;
    char *dst_addr;
    u64 loop;
    u64 remaining_size;

    memset(write_message, 1, BUFSIZ);
    write_message[BUFSIZ] = '\0';
    /**
     * Get the RQ from client : {src_add} {flowid} {size} {dst_add}
     */
    len = recv(sock_local, read_message, 1024, 0);
    if (len <= 0)
    {
        DBGMARK(DBGSERVER, 1, "recv failed! \n");
        close(sock_local);
        return 0;
    }
    /**
     * For Now: we send the src IP in the RQ to!
     */
    src_ip = strtok_r(read_message, " ", &savePtr);
    if (src_ip == NULL)
    {
        // discard message:
        DBGMARK(DBGSERVER, 1, "id: %d discarding this message:%s \n",
                flow->flowinfo.flowid, savePtr);
        close(sock_local);
        return 0;
    }
    char *isstr = strtok_r(NULL, " ", &savePtr);
    if (isstr == NULL)
    {
        // discard message:
        DBGMARK(DBGSERVER, 1, "id: %d discarding this message:%s \n",
                flow->flowinfo.flowid, savePtr);
        close(sock_local);
        return 0;
    }
    flow->flowinfo.flowid = atoi(isstr);
    char *size_ = strtok_r(NULL, " ", &savePtr);
    flow->flowinfo.size = 1024 * atoi(size_);
    DBGPRINT(DBGSERVER, 4, "%s\n", size_);
    dst_addr = strtok_r(NULL, " ", &savePtr);
    if (dst_addr == NULL)
    {
        // discard message:
        DBGMARK(DBGSERVER, 1, "id: %d discarding this message:%s \n",
                flow->flowinfo.flowid, savePtr);
        close(sock_local);
        return 0;
    }
    char *time_s_ = strtok_r(NULL, " ", &savePtr);
    char *endptr;
    start_of_client = strtoimax(time_s_, &endptr, 10);
    got_message = 1;
    DBGPRINT(DBGSERVER, 2, "Got message: %" PRIu64 " us\n", timestamp());
    flow->flowinfo.rem_size = flow->flowinfo.size;
    DBGPRINT(DBGSERVER, 2, "time_rcv:%" PRIu64 " get:%s\n", start_of_client,
             time_s_);

    // Get detailed address
    strtok_r(src_ip, ".", &savePtr);
    if (dst_addr == NULL)
    {
        // discard message:
        DBGMARK(DBGSERVER, 1, "id: %d discarding this message:%s \n",
                flow->flowinfo.flowid, savePtr);
        close(sock_local);
        return 0;
    }

    // Calculate loops. In each loop, we can send BUFSIZ (8192) bytes of data
    loop = flow->flowinfo.size / BUFSIZ * 1024;
    // Calculate remaining size to be sent
    remaining_size = flow->flowinfo.size * 1024 - loop * BUFSIZ;
    // Send data with 8192 bytes each loop
    DBGPRINT(0, 0, "Server is sending the traffic ...\n");

    // for(u64 i=0;i<loop;i++)
    while (send_traffic)
    {
        len = strlen(write_message);
        DBGMARK(DBGSERVER, 5, "len = %d\n", len);
        while (len > 0)
        {
            DBGMARK(DBGSERVER, 5, "++++++\n");
            int s = send(sock_local, write_message, strlen(write_message), 0);
            DBGMARK(DBGSERVER, 5, "Successfully sent = %d\n", s);
            len -= s;
            usleep(50);
            DBGMARK(DBGSERVER, 5, "------\n");
        }
        usleep(100);
    }
    flow->flowinfo.rem_size = 0;
    done = true;
    close(sock_local);
    return ((void *)0);
}

void *TimerThread(void *information)
{
    uint64_t start = timestamp();
    unsigned int elapsed;
    if ((duration != 0))
    {
        while (send_traffic)
        {
            sleep(1);
            elapsed = (unsigned int)((timestamp() - start) / 1000000); // unit s
            if (elapsed > duration)
            {
                send_traffic = false;
            }
        }
    }

    return ((void *)0);
}

void *RLThread(void *information)
{
    // 初始化
    // 状态标志
    bool got_no_zero = false; // 本回合收到0字节
    bool got_alpha = false;
    bool slow_start_passed = false;

    // 读取值
    char message[1000];
    char *num;
    char *alpha;
    char *save_ptr;
    int pre_id = 9230;
    int pre_id_tmp = 0;
    int msg_id = 657;
    int get_info_error_counter = 0;
    int get_new_alpha_error_counter = 0;
    int get_alpha_error_counter = 0;

    // sage-rl 所需参数
    char *shared_memory_rl2 = (char *)malloc(strlen(shared_memory_rl));
    u64 pre_bytes_acked = 0, pre_bytes_sent = 0, pre_pkt_dlv = 0,
        pre_pkt_lost = 0;
    u64 max_delivery_rate = 0, max_sending_rate = 0;
    double dr_mbps = 0.0, l_mbps = 0.0, pre_dr_mbps = 0.0, sr_mbps = 0.0;
    double dr_max_mbps = 1.0, pre_dr_max_mbps = 1.0;

    double cwnd_precise = CWND_INIT;
    double cwnd_rate = 1.0;

    dq_sage<u64> sending_rates(WIN_SIZE * 10);

    dq_sage<double> lost_s(SHORT_WIN); // x = lost pkts / 1000
    dq_sage<double> lost_m(MID_WIN);
    dq_sage<double> lost_l(LONG_WIN);

    dq_sage<double> inflight_s(SHORT_WIN); // x = inflight pkts / 1000
    dq_sage<double> inflight_m(MID_WIN);
    dq_sage<double> inflight_l(LONG_WIN);

    dq_sage<double> thr_s(SHORT_WIN);
    dq_sage<double> thr_m(MID_WIN);
    dq_sage<double> thr_l(LONG_WIN);

    dq_sage<double> rtt_rate_s(SHORT_WIN);
    dq_sage<double> rtt_rate_m(MID_WIN);
    dq_sage<double> rtt_rate_l(LONG_WIN);

    dq_sage<double> rtt_var_s(SHORT_WIN);
    dq_sage<double> rtt_var_m(MID_WIN);
    dq_sage<double> rtt_var_l(LONG_WIN);

    dq_sage<u64> sent_db(100); // sent delta bytes
    dq_sage<u64> dlv_db(100);  // delivered delta bytes
    dq_sage<u64> loss_db(100); // lost delta bytes
    dq_sage<u64> uack_db(100); // inflight bytes
    dq_sage<u64> sent_dt(100); // delta time

    dq_sage<double> dr_w(200);
    dq_sage<double> rtt_w(200);

    u64 dt_pre = timestamp();
    u64 pre_calculation_time = timestamp();
    u64 t0 = timestamp();
    u64 dt = 0;
    u64 ts_begin = timestamp();
    u64 ts_last = timestamp();

    while (true)
    {
        usleep(10000);
        got_no_zero = false;
        update_tcp_info(0);
        // 向内存中传值
        while (!got_no_zero && send_traffic)
        {
            // update_tcp_info(0);
            if (info.tcpi_rtt > 0)
            {

                u64 d_dp = info.tcpi_delivered - pre_pkt_dlv; // 这段时间发送的包数
                u64 d_db = d_dp * mss_cache;                  // 这段时间送达的字节数
                u64 s_db;                                     // 这段时间发送的字节数
                u64 l_db;                                     // 这段时间丢失的字节数

                pre_pkt_dlv = info.tcpi_delivered;

                if (info.tcpi_bytes_acked - pre_bytes_acked > 0 || d_db > 0)
                {
                    FILE *file = fopen("/home/snow/pantheon-available/src/experiments/duration.txt", "a");
                    if (file)
                    {
                        fprintf(file, "time = %u, duration = %u\n", timestamp() - ts_begin, timestamp() - ts_last);
                    }
                    fclose(file);
                    ts_last = timestamp();
                    dt = timestamp() - dt_pre; // 单位是us
                    dt = (dt > 0) ? dt : 1;    // 时间相差要在1us以上
                    dt_pre = timestamp();

                    s_db = info.tcpi_bytes_sent - pre_bytes_sent; // sent bytes
                    l_db = (info.tcpi_lost > pre_pkt_lost)
                               ? (info.tcpi_lost - pre_pkt_lost) * info.tcpi_snd_mss
                               : 0; // lost bytes

                    uack_db.add((u64)info.tcpi_unacked * mss_cache);
                    sent_db.add(s_db);
                    dlv_db.add(d_db);
                    loss_db.add(l_db);
                    sent_dt.add(dt); // 每个时间段长度

                    // 累积平均字段
                    u64 dt_sum = sent_dt.sum();
                    dr_mbps = (double)8 * dlv_db.sum() / dt_sum; // delivery rate
                    l_mbps = (double)8 * loss_db.sum() / dt_sum; // loss rate
                    rtt_w.add((double)info.tcpi_rtt / 100000.0);
                    dr_w.add(dr_mbps);
                    dr_max_mbps = dr_w.max();
                    if (dr_max_mbps == 0.0)
                    {
                        dr_max_mbps = 1;
                    }
                    max_delivery_rate = dr_max_mbps * 1000000 / 8; // Bps为单位

                    double time_delta =
                        (double)(timestamp() - pre_calculation_time) / 1000000; // us
                    pre_calculation_time = timestamp();
                    double acked_rate =
                        (double)(info.tcpi_bytes_acked - pre_bytes_acked) / time_delta;
                    double sending_rate = (double)s_db / time_delta;

                    // 更新pre字段
                    pre_bytes_sent = info.tcpi_bytes_sent;
                    pre_pkt_lost = info.tcpi_lost;
                    pre_bytes_acked = info.tcpi_bytes_acked;
                    if (max_delivery_rate <= 0)
                    {
                        acked_rate = 0;
                    }

                    sending_rates.add((u64)sending_rate);
                    max_sending_rate = sending_rates.max();
                    double rtt_rate = (double)min_rtt / info.tcpi_rtt;
                    time_delta = time_delta * 1000000 / min_rtt;
                    acked_rate = acked_rate / max_sending_rate;

                    u64 s_db_tmp = sent_db.sum();
                    u64 ua_db_tmp = uack_db.avg();
                    double cwnd_unacked_rate =
                        (s_db_tmp > 0) ? (double)ua_db_tmp / s_db_tmp : (double)ua_db_tmp;
                    u64 cwnd_bits = (u64)info.tcpi_snd_cwnd * info.tcpi_snd_mss * 8;
                    if (cwnd_bits == 0)
                        cwnd_bits++;

                    /***
                     * Smoothed Versions ...
                     ***/
                    rtt_s.add((double)info.tcpi_rtt / 100000);
                    rtt_m.add((double)info.tcpi_rtt / 100000);
                    rtt_l.add((double)info.tcpi_rtt / 100000);

                    rtt_rate_s.add(rtt_rate);
                    rtt_rate_m.add(rtt_rate);
                    rtt_rate_l.add(rtt_rate);

                    rtt_var_s.add((double)info.tcpi_rttvar / 1000.0);
                    rtt_var_m.add((double)info.tcpi_rttvar / 1000.0);
                    rtt_var_l.add((double)info.tcpi_rttvar / 1000.0);

                    thr_s.add((double)info.tcpi_delivery_rate / 125000.0 /
                              BW_NORM_FACTOR);
                    thr_m.add((double)info.tcpi_delivery_rate / 125000.0 /
                              BW_NORM_FACTOR);
                    thr_l.add((double)info.tcpi_delivery_rate / 125000.0 /
                              BW_NORM_FACTOR);

                    inflight_s.add((double)info.tcpi_unacked / 1000.0);
                    inflight_m.add((double)info.tcpi_unacked / 1000.0);
                    inflight_l.add((double)info.tcpi_unacked / 1000.0);

                    lost_s.add((double)info.tcpi_lost / 100.0);
                    lost_m.add((double)info.tcpi_lost / 100.0);
                    lost_l.add((double)info.tcpi_lost / 100.0);
                    char message_extra[1000];

                    sprintf(
                        message_extra,
                        "%.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f "
                        "%.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f "
                        "%.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f "
                        "%.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f "
                        "%.7f %.7f %.7f %.7f %.7f %.7f",
                        // 4-12
                        rtt_s.get_avg(), rtt_s.get_min(), rtt_s.get_max(),
                        rtt_m.get_avg(), rtt_m.get_min(), rtt_m.get_max(),
                        rtt_l.get_avg(), rtt_l.get_min(), rtt_l.get_max(),
                        // 13-21
                        thr_s.get_avg(), thr_s.get_min(), thr_s.get_max(),
                        thr_m.get_avg(), thr_m.get_min(), thr_m.get_max(),
                        thr_l.get_avg(), thr_l.get_min(), thr_l.get_max(),
                        // 22-30
                        rtt_rate_s.get_avg(), rtt_rate_s.get_min(), rtt_rate_s.get_max(),
                        rtt_rate_m.get_avg(), rtt_rate_m.get_min(), rtt_rate_m.get_max(),
                        rtt_rate_l.get_avg(), rtt_rate_l.get_min(), rtt_rate_l.get_max(),
                        // 31-39
                        rtt_var_s.get_avg(), rtt_var_s.get_min(), rtt_var_s.get_max(),
                        rtt_var_m.get_avg(), rtt_var_m.get_min(), rtt_var_m.get_max(),
                        rtt_var_l.get_avg(), rtt_var_l.get_min(), rtt_var_l.get_max(),
                        // 40-48
                        inflight_s.get_avg(), inflight_s.get_min(), inflight_s.get_max(),
                        inflight_m.get_avg(), inflight_m.get_min(), inflight_m.get_max(),
                        inflight_l.get_avg(), inflight_l.get_min(), inflight_l.get_max(),
                        // 49-57
                        lost_s.get_avg(), lost_s.get_min(), lost_s.get_max(),
                        lost_m.get_avg(), lost_m.get_min(), lost_m.get_max(),
                        lost_l.get_avg(), lost_l.get_min(), lost_l.get_max());
                    sprintf(message,
                            "%d %.7f %.7f %.7f %u %s %.7f %.7f %.7f %.7f %.7f %.7f %.7f "
                            "%.7f %.7f %.7f %.7f",
                            msg_id,
                            // 0
                            (double)info.tcpi_rtt / 100000.0, /*sRTT in 100x (ms):e.g. 2 = 2x100=200 ms*/
                            // 1
                            (double)info.tcpi_rttvar /
                                1000.0, /*var of sRTT in 1x (ms). */
                            // 2
                            (double)info.tcpi_delivery_rate / 125000.0 /
                                BW_NORM_FACTOR, /*del rate 100x Mbps*/
                            // 3
                            info.tcpi_ca_state, /*TCP_CA_Open=0 -> TCP_CA_Loss=4*/
                            // 4-57
                            message_extra,
                            // 58
                            time_delta,
                            // 59
                            rtt_rate,
                            // 60
                            l_mbps / BW_NORM_FACTOR,
                            // 61
                            acked_rate,
                            // 62
                            (pre_dr_mbps > 0.0) ? (dr_mbps / pre_dr_mbps) : dr_mbps,
                            // 63
                            (double)(dr_max_mbps * info.tcpi_min_rtt) / (cwnd_bits),
                            // 64
                            (dr_mbps) / BW_NORM_FACTOR,
                            // 65
                            cwnd_unacked_rate,
                            // 66
                            (pre_dr_max_mbps > 0.0) ? (dr_max_mbps / pre_dr_max_mbps)
                                                    : dr_max_mbps,
                            // 67
                            dr_max_mbps / BW_NORM_FACTOR,
                            // 68
                            (cwnd_rate > 0.0) ? round(log2f(cwnd_rate) * 1000) / 1000.
                                              : log2f(0.0001));

                    pre_dr_mbps = dr_mbps;
                    pre_dr_max_mbps = dr_max_mbps;
                }
                else
                {
                    // DBGMARK(0, 0, "Warning: no pkt acked this round, skip!\n");
                    continue;
                } // endif d_db > 0
                memcpy(shared_memory, message, sizeof(message));
                msg_id = (msg_id + 1) % 1000;
                got_no_zero = true;
                get_info_error_counter = 0;
            }
            else
            {
                // get到的数据错误，记录一下错误信息
                get_info_error_counter++;
                if (get_info_error_counter > 30000)
                {
                    DBGMARK(0, 0,
                            "No valid state for 1 min. We (server of Actor %d) are going "
                            "down down down ...\n",
                            actor_id);
                    send_traffic = false;
                }
                usleep(20 * 100);
            } // endif rtt>0
        }     // endwhile got_no_zero
        // 观察模型是否已经返回值
        got_alpha = false;
        while (!got_alpha && send_traffic)
        {
            // Get alpha from RL-Module, shared_memory_rl中有两个值
            num = strtok_r(shared_memory_rl, " ", &save_ptr);
            alpha = strtok_r(NULL, " ", &save_ptr);
            if (num != NULL && alpha != NULL)
            {
                pre_id_tmp = atoi(num);
                /*****************************/
                FILE *file = fopen("/home/snow/pantheon-available/src/experiments/alpha.txt", "a");
                if (file)
                {
                    fprintf(file, "pre_id_tmp : %d\n", pre_id_tmp);
                }
                fclose(file);
                /*****************************/
                if (pre_id_tmp != pre_id)
                {
                    got_alpha = true;
                    pre_id = pre_id_tmp;
                    cwnd_precise = mul(cwnd_precise, atof(alpha)); // 具体的cwnd
                    cwnd_rate = atof(alpha);
                    /*****************************/
                    FILE *file = fopen("/home/snow/pantheon-available/src/experiments/alpha.txt", "a");
                    if (file)
                    {
                        fprintf(file, "cwnd_precise, cwnd_rate : %.2lf,%.2lf\n", cwnd_precise, cwnd_rate);
                    }
                    fclose(file);
                    /*****************************/
                    u64 cwnd_tmp;
                    if (ROUNT_TYPE == FLOORING)
                    {
                        cwnd_tmp = floor(cwnd_precise);
                    }
                    else if (ROUNT_TYPE == CEILING)
                    {
                        cwnd_tmp = ceil(cwnd_precise);
                    }
                    else if (ROUNT_TYPE == ROUNDING)
                    {
                        cwnd_tmp = round(cwnd_precise);
                    }
                    else
                    {
                        cwnd_tmp = (u64)(cwnd_precise);
                    }

                    if (cwnd_tmp >= MAX_32Bit)
                        target_ratio = MAX_32Bit;
                    else
                        target_ratio = cwnd_tmp;

                    if (target_ratio < MIN_CWND)
                    {
                        target_ratio = MIN_CWND;
                    }
                    else if (target_ratio > MAX_CWND)
                    {
                        target_ratio = MAX_CWND;
                        cwnd_precise = MAX_CWND;
                    }
                }
                else
                {
                    if (get_new_alpha_error_counter > 1000)
                    {
                        DBGPRINT(DBGSERVER, 0, "still no new value, id:%d prev_id:%d\n",
                                 pre_id_tmp, pre_id);
                        get_new_alpha_error_counter = 0;
                    }
                    get_new_alpha_error_counter++;
                    usleep(1000);
                } // endif new alpha
                get_alpha_error_counter = 0;
            }
            else
            {
                if (get_alpha_error_counter == 1000)
                {
                    // 50次都没获取到alpha值, 不要复杂的处理了
                    DBGMARK(0, 0, "NO VALID ALPHA FOR 1 MIN");
                    get_alpha_error_counter = 0;
                }
                else
                {
                    get_alpha_error_counter++;
                    usleep(10000);
                }
            } // endif no_alpha
        }     // endwhile got_alpha
    }
}

void start_server(int client_port)
{
    int num_lines = 0;
    cFlow *flow = new cFlow; // 仅生成单条流即可
    sInfo *info = new sInfo;

    if (flow == NULL)
    {
        DBGMARK(DBGSERVER, 0, "flow generation failed\n");
        return;
    }

    // threads
    pthread_t data_thread;
    pthread_t cnt_thread;
    pthread_t timer_thread;
    pthread_t rl_thread;

    struct sockaddr_in server_addr; // Server address
    struct sockaddr_in client_addr; // Client address
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;         // IP protocol
    server_addr.sin_addr.s_addr = INADDR_ANY; // Listen on "0.0.0.0"
    server_addr.sin_port = htons(client_port);

    if ((sock[0] = socket(PF_INET, SOCK_STREAM, 0)) < 0) // orca 默认给了sock数组
    {
        DBGMARK(0, 0, "sockopt: %s\n", strerror(errno));
        return;
    }

    int reuse = 1;
    if (setsockopt(sock[0], SOL_SOCKET, SO_REUSEADDR, (const char *)&reuse,
                   sizeof(reuse)) < 0)
    {
        perror("setsockopt(SO_REUSEADDR) failed");
    }

    // Bind socket on IP:Port
    if (bind(sock[0], (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) <
        0)
    {
        DBGMARK(0, 0, "bind error srv_ctr_ip: 000000: %s\n", strerror(errno));
        close(sock[0]);
        return;
    }

    // Set kernel Congestion control
    if (scheme)
    {
        if (setsockopt(sock[0], IPPROTO_TCP, TCP_CONGESTION, scheme,
                       strlen(scheme)) < 0)
        {
            DBGMARK(0, 0, "TCP congestion doesn't exist: %s\n", strerror(errno));
            return;
        }
    }
    info->trace = trace;
    info->num_lines = num_lines;

    /**
     *Setup Shared Memory
     */
    key = (key_t)(actor_id * 10000 + rand() % 10000 + 1);
    key_rl = (key_t)(actor_id * 10000 + rand() % 10000 + 1);
    // Setup shared memory, 11 is the size
    if ((shmid = shmget(key, shmem_size, IPC_CREAT | 0666)) < 0)
    {
        printf("Error getting shared memory id");
        return;
    }

    // Attached shared memory
    if ((shared_memory = (char *)shmat(shmid, NULL, 0)) == (char *)-1)
    {
        printf("Error attaching shared memory id");
        return;
    }

    // Setup shared memory, 11 is the size
    if ((shmid_rl = shmget(key_rl, shmem_size, IPC_CREAT | 0666)) < 0)
    {
        printf("Error getting shared memory id");
        return;
    }

    // Attached shared memory
    if ((shared_memory_rl = (char *)shmat(shmid_rl, NULL, 0)) == (char *)-1)
    {
        printf("Error attaching shared memory id");
        return;
    }

    char cmd[1000];
    // sprintf(cmd, "/usr/bin/python %s/d5.py --load --tb_interval=1
    // --base_path=%s --task=%d --job_name=actor --mem_r=%d --mem_w=%d&", path,
    // path, actor_id, (int)key, (int)key_rl);
    sprintf(cmd,
            "/home/snow/venvpy37/bin/python "
            "/home/snow/pantheon-available/third_party/sage/sage_rl/rl_module/"
            "tcpactor.py --mem_r=%d --mem_w=%d --id=%d &",
            (int)key, (int)key_rl, actor_id);
    initial_timestamp(); // 初始化时间戳
    system(cmd);

    // Wait to get OK signal (alpha=OK_SIGNAL)
    bool got_ready_signal_from_rl = false;
    int signal;
    char *num;
    char *alpha;
    char *save_ptr;
    int signal_check_counter = 0;
    while (!got_ready_signal_from_rl)
    {
        // Get alpha from RL-Module
        signal_check_counter++;
        num = strtok_r(shared_memory_rl, " ", &save_ptr);
        alpha = strtok_r(NULL, " ", &save_ptr);
        if (num != NULL && alpha != NULL)
        {
            signal = atoi(alpha);
            if (signal == OK_SIGNAL)
            {
                got_ready_signal_from_rl = true;
            }
            else
            {
                usleep(1000);
            }
        }
        else
        {
            usleep(10000);
        }
        if (signal_check_counter > 18000)
        {
            DBGERROR("After 3 minutes, no response (OK_Signal) from the Actor %d is "
                     "received! We are going down down down ...\n",
                     actor_id);
            return;
        }
    }
    DBGPRINT(0, 0, "RL Module is Ready. Let's Start ...\n\n");
    usleep(actor_id * 10000 + 10000); // actor_id=0, 停10ms

    // Start listen
    int maxfdp = -1;
    fd_set rset;
    FD_ZERO(&rset);
    listen(sock[0], 1);
    FD_SET(sock[0], &rset);
    if (sock[0] > maxfdp)
        maxfdp = sock[0];
    maxfdp++;
    struct timeval timeout;
    timeout.tv_sec = 600;
    timeout.tv_usec = 0;
    int rc = select(maxfdp, &rset, NULL, NULL, &timeout);
    if (rc < 0)
    {
        DBGERROR("select failed!\n");
        return;
    }
    else if (rc == 0)
    {
        DBGERROR("select timeout!\n");
        return;
    }

    // 建立链接
    int sin_size = sizeof(struct sockaddr_in);
    if (FD_ISSET(sock[0], &rset))
    {
        int val = accept(sock[0], (struct sockaddr *)&client_addr,
                         (socklen_t *)&sin_size);
        if (val < 0)
        {
            perror("accept error\n");
            close(sock[0]);
            return;
        }
        sock_for_cnt[0] = val; // 套接字描述符
        flow[0].flowinfo.sock = val;
        flow[0].dst_addr = client_addr;

        // 创建data thread
        if (pthread_create(&data_thread, NULL, DataThread, (void *)&flow[0]) < 0)
        {
            perror("could not create data thread\n");
            close(sock[0]);
            return;
        }

        if (pthread_create(&cnt_thread, NULL, CntThread, (void *)info) < 0)
        {
            perror("could not create control thread\n");
            close(sock[flow_index]);
            return;
        }

        if (pthread_create(&timer_thread, NULL, TimerThread, (void *)info) < 0)
        {
            perror("could not create timer thread\n");
            close(sock[flow_index]);
            return;
        }
        if (pthread_create(&rl_thread, NULL, RLThread, (void *)info) < 0)
        {
            perror("could not create RL thread\n");
            close(sock[flow_index]);
            return;
        }
    }
    pthread_join(data_thread, NULL); // 优先等待数据发送线程完成
}

int main(int argc, char **argv)
{
    if (argc != 8)
    {
        for (int i = 0; i < argc; i++)
            DBGERROR("argv[%d]:%s\n", i, argv[i]);
        usage();
        return 0;
    }
    srand(raw_timestamp());

    signal(SIGSEGV, handler); // install our handler
    signal(SIGTERM, handler); // install our handler
    signal(SIGABRT, handler); // install our handler
    signal(SIGFPE, handler);  // install our handler
    signal(SIGKILL, handler); // install our handler

    client_port = atoi(argv[1]); // client端口
    path = argv[2];              // rl-module的path
    target = 50;                 // 目标RTT
    target_ratio = 1;
    report_period = atoi(argv[3]);
    scheme = argv[4]; // 内核算法
    actor_id = atoi(argv[5]);
    duration = atoi(argv[6]);
    duration_steps = atoi(argv[7]);

    start_server(client_port);
    DBGPRINT(0, 0, "22222222222222222222222222\n\n");

    shmdt(shared_memory);
    shmctl(shmid, IPC_RMID, NULL);
    shmdt(shared_memory_rl);
    shmctl(shmid_rl, IPC_RMID, NULL);
    return 0;
}