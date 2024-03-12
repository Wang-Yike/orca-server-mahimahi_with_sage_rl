#include <cstdlib>
#include <sys/select.h>
#include "define.h"
#include <iostream>
#include <chrono>
#include <iomanip>
#include <fstream>
#include <string>
#include <cstdio>
#include <errno.h>
#include <queue>
#include <cmath>
#include <algorithm>
#include <random>
#include <vector>
#include <linux/tcp.h>

// for dq_sage
#include <deque>

#define ENABLE_ANOLE_RL false
#define MAX_CWND 10000
#define MIN_CWND 4
#define CWND_THRESHOLD 2
#define RTT_LENGTH 20
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
#define LAMBDA 11.35
#define MU 0.1 // RTT的震荡范围
// 置信度
#define THETA 0.05
#define ETA_ON 0.8
#define ETA_OFF 0.6
#define DELTA_ETA 0.05 // 每次增长的幅度

// --------------------------------------- sage logic start ---------------------------------------
// Wait "WAIT_FOR_ACTION_MAX_ms" for getting action from RL agent, 
// beyond that, simply assume no action is received and go for another state report to RL agent.
// #define WAIT_FOR_ACTION_MAX_ms 200 

//#define CHANGE_TARGET 1
#define FLOORING 0
#define ROUNDING 1
#define CEILING  2
#define NONE     3
#define ROUNT_TYPE CEILING

// #define TURN_ON_SAFETY 0
// #define SAFETY_MARGIN 3 //10

// #define ACCUMULATE_CWND 1

#define SHORT_WIN   10
#define MID_WIN     200
#define LONG_WIN    1000

// #define ESTIMATE 0
// #define ACCURACY 10000.0 //1000.0 //100.0 //100.0 //10000.0    //Final Cwnd precision: 1/ACCURACY


#define WIN_SIZE 500

// TODO pacing
// #define PACE_ENABLE 1
// #define PACE_TYPE PACE_WITH_SRTT
// #define PACE_COEF 100

#define CWND_INIT 10
#define MIN_CWND 4

#define BW_NORM_FACTOR 100     //100 Mbps will be used to normalize throughput signal

#define MAX_32Bit 0x7FFFFFFF
//Make sure we don't have (int32) overflow!
double mul(double a, double b)
{
    return ((a*b)>MAX_32Bit)?MAX_32Bit:a*b;
}

template<class T>
class dq_sage {
    std::deque<T>* dq;
    T default_max;
    u32 size;
    std::deque<T>* dq_min;
    std::deque<T>* dq_max;
    //std::deque<double>* dq_avg;
    double average;
    u32 length;
    public:
        dq_sage(u32 size)
        {
            init(size);
        };
        void init(u32 size,T default_max)
        {
            this->size = size;
            this->defualt_max = default_max;
            dq = new std::deque<T>;
            dq_min = new std::deque<T>;
            dq_max = new std::deque<T>;
            //dq_avg = new std::deque<double>;
            this->average = 0;
        };
        void init(u32 size)
        {
            this->size = size;
            dq = new std::deque<T>;
            this->default_max = (T)100;   //100Mbps
            dq_min = new std::deque<T>;
            dq_max = new std::deque<T>;
            //dq_avg = new std::deque<double>;
            this->average = 0;
        };
        T get_min()
        {
            return (this->dq_min->size())?this->dq_min->front():1e6;
        }
        T get_max()
        {
            return (this->dq_max->size())?this->dq_max->front():0;
        }
        double get_avg()
        {
            return this->average;
        }
        T get_sum()
        {
            return (T)(get_avg()*this->dq->size());
        }
        int add(T entry)
        {
            T new_min = get_min();
            T new_max = get_max();
            u32 len = this->dq->size();
            if(entry<new_min)
            {
                new_min = entry;
            }
            if(entry>new_max)
            {
                new_max = entry;
            }

            if(len>=this->size)
            {  
                T to_be_removed = this->dq->back();
                this->dq->pop_back();
                this->average = (this->average*len-(double)to_be_removed+(double)entry)/(len);

                if(to_be_removed==get_min())
                {
                    new_min = min();
                    if(entry<new_min)
                        new_min = entry;
                }
                this->dq_min->pop_back();
                this->dq_min->push_front(new_min);
            
                if(to_be_removed==get_max())
                {
                    new_max = max();
                    if(entry>new_max)
                        new_max = entry;
                }
                this->dq_max->pop_back(); 
                this->dq_max->push_front(new_max);
                
                //this->dq->pop_back();
                this->dq->push_front(entry);
            }
            else
            {
                this->average = (len)?(this->average*len+(double)entry)/(len+1):entry;
                this->dq_min->push_front(new_min);
                this->dq_max->push_front(new_max);
                this->dq->push_front(entry);
            }
        };
        T max()
        {
            T max=0;
            int occupancy=0;
            typename std::deque<T>::iterator it;
            for(it=this->dq->begin(); it!=this->dq->end(); it++)
            {
                if(max<*it)
                {
                    max=*it;
                }
                //occupancy++;
            }
            return max;
        };
        T min()
        {
            T min=1e6;
            typename std::deque<T>::iterator it;
            for(it=this->dq->begin(); it!=this->dq->end(); it++)
            {
                if(min>*it)
                {
                    min=*it;
                }
            }
            return min;
        };
        T sum()
        {
            T sum = 0;
            typename std::deque<T>::iterator it;
            for(it=this->dq->begin(); it!=this->dq->end(); it++)
            {
                sum += *it;                                              
            }
            return sum;                                                             
        }
        T avg()
        {
            T sum = 0;
            u32 counter=0;
            typename std::deque<T>::iterator it;
            for(it=this->dq->begin(); it!=this->dq->end(); it++)
            {
                sum += *it;                                             
                counter++;
            }
            return (counter)?(T)sum/counter:0;
        }
        void std(T& mean,T& std)
        {
            mean = avg();
            T var = 0;
            u32 counter=0;
            typename std::deque<T>::iterator it;
            for(it=this->dq->begin(); it!=this->dq->end(); it++)
            {
                var += (mean-*it)*(mean-*it);
                counter++;
            }
            std = var/counter;
            std = sqrt(std);
        }
};
// ---------------------------------------- sage logic end ----------------------------------------

// 全局变量, rtt的单位全部为uSec
uint32_t his_rtts[RTT_LENGTH]; // 最近RTT历史记录
uint32_t cur_rtt = 0;
uint32_t min_rtt = 0;
uint16_t mss_cache = 1460;
struct tcp_info info, info_pre;
struct utility_value
{
    uint64_t cwnd = 10;
    double utility = 0.0;
    double confidence_val = 1.0;
    bool isright = false;
} u_cl, u_rl, u_prev; // u_prev应该存放的是历史最佳
double u_optimal = 0.0;

void usage()
{
    DBGMARK(0, 0, "./server [port] [path to ddpg.py] [Report Period: 20 msec] [First Time: 1=yes(learn), 0=no(continue learning), 2=evaluate] [actor id=0, 1, ...]\n");
}

// 最新的RTT采样，srtt请读取info中的tcpi_rtt
void get_cur_rtt(int index)
{
    socklen_t length = sizeof(cur_rtt);
    if (getsockopt(sock_for_cnt[index], IPPROTO_TCP, TCP_RTT, &cur_rtt, &length) < 0)
        perror("getsockopt_cur_rtt\n");
}

void get_min_rtt(int index)
{
    socklen_t length = sizeof(min_rtt);
    if (getsockopt(sock_for_cnt[index], IPPROTO_TCP, TCP_RTTMIN, &min_rtt, &length) < 0)
        perror("getsockopt_cur_rtt\n");
}

void get_his_rtts(int index)
{
    socklen_t length = sizeof(his_rtts);
    if (getsockopt(sock_for_cnt[index], IPPROTO_TCP, TCP_RTTS, &his_rtts[0], &length) < 0)
        perror("getsockopt_his_rtts\n");
}

void get_tcp_info(int index)
{
    socklen_t length = sizeof(tcp_info);
    if (getsockopt(sock_for_cnt[index], IPPROTO_TCP, TCP_INFO, &info, &length) < 0)
        perror("getsockopt_tcp_info\n");
}

void get_mss_cache(int index)
{
    socklen_t length = sizeof(mss_cache);
    if (getsockopt(sock_for_cnt[index], IPPROTO_TCP, TCP_MAXSEG, &mss_cache, &length) == 0)
        if (!mss_cache)
            mss_cache = 1460;
}

void get_utility_value(uint64_t cwnd, utility_value *u)
{
    get_his_rtts(0);
    get_min_rtt(0);
    double d_rtt = cal_delta_rtt();
    double a_rtt = cal_avg_rtt();
    // 单位, 当前单位为MBps
    double rate = static_cast<double>(cwnd) * mss_cache / a_rtt;

    if (d_rtt > 0.02)
        u->isright = true;
    else
        u->isright = false;
    u->utility = ALPHA * pow(rate, EXPONENT_T) - BETA * rate * max(0.0, d_rtt); // 未添加对RTT绝对值的惩罚
    double ratio = a_rtt / static_cast<double>(min_rtt);
    if (ratio > (1 + MU))
    {
        u->utility -= LAMBDA * rate * ratio;
    }
    if (u->utility > u_optimal)
    {
        u_optimal = u->utility;
    }
}

void set_cwnd(uint64_t cwnd, int index)
{
    socklen_t length = sizeof(cwnd);
    if (setsockopt(sock_for_cnt[index], IPPROTO_TCP, TCP_CWND_USER, &cwnd, length) < 0)
        perror("setsockopt: set cwnd\n");
}

void get_confidence_val(utility_value &u)
{
    double y = u.utility / u_optimal + THETA;
    u.confidence_val = min(1.0, max(0.5, y * u.confidence_val));
}

double cal_delta_rtt()
{
    double res = 0.0;
    int len = (RTT_LENGTH / 2);
    for (int i = len; i < RTT_LENGTH; i++)
        res += his_rtts[i];
    for (int i = 0; i < (RTT_LENGTH / 2); i++)
        res -= his_rtts[i];
    if (min_rtt)
        return (res / static_cast<double>(min_rtt) / 10);
    else
        DBGMARK(DBGSERVER, 0, "ERROR: GET_DELTA_RTT MIN_RTT=0");
    return 0;
}

double cal_avg_rtt()
{
    uint32_t sum = 0;
    for (int i = 0; i < RTT_LENGTH; i++)
        if (his_rtts[i])
            sum += his_rtts[i];

    return static_cast<double>(sum) / RTT_LENGTH;
    // 内核中的rtt估计采用的是7/8旧+1/8新
}

int cal_situation()
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

bool compareStruct(const struct utility_value &s1, const struct utility_value &s2)
{
    return s1.cwnd < s2.cwnd;
}

uint64_t cal_optimal_cwnd(int situation)
{
    uint64_t optimal_cwnd = 0;
    vector<struct utility_value> utilities = {u_cl, u_rl, u_prev};
    sort(utilities.begin(), utilities.end(), compareStruct);
    switch (situation)
    {
    case 0:
        optimal_cwnd = utilities[0].cwnd - 1;
        break;
    case 1:
        if (utilities[1].cwnd - utilities[0].cwnd <= CWND_THRESHOLD)
            optimal_cwnd = utilities[1].utility > utilities[0].utility ? utilities[1].cwnd : utilities[0].cwnd;
        else
            // equation 8
            optimal_cwnd = (utilities[1].cwnd + utilities[0].cwnd) / 2;
        break;
    case 2:
        if (utilities[2].cwnd - utilities[1].cwnd <= CWND_THRESHOLD)
            optimal_cwnd = utilities[2].utility > utilities[1].utility ? utilities[2].cwnd : utilities[1].cwnd;
        else
            optimal_cwnd = (utilities[2].cwnd + utilities[1].cwnd) / 2;
        break;
    case 3:
        optimal_cwnd = utilities[2].cwnd + 1;
        break;
    default:
        break;
    }
    return optimal_cwnd;
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
        DBGMARK(DBGSERVER, 1, "id: %d discarding this message:%s \n", flow->flowinfo.flowid, savePtr);
        close(sock_local);
        return 0;
    }
    char *isstr = strtok_r(NULL, " ", &savePtr);
    if (isstr == NULL)
    {
        // discard message:
        DBGMARK(DBGSERVER, 1, "id: %d discarding this message:%s \n", flow->flowinfo.flowid, savePtr);
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
        DBGMARK(DBGSERVER, 1, "id: %d discarding this message:%s \n", flow->flowinfo.flowid, savePtr);
        close(sock_local);
        return 0;
    }
    char *time_s_ = strtok_r(NULL, " ", &savePtr);
    char *endptr;
    start_of_client = strtoimax(time_s_, &endptr, 10);
    got_message = 1;
    DBGPRINT(DBGSERVER, 2, "Got message: %" PRIu64 " us\n", timestamp());
    flow->flowinfo.rem_size = flow->flowinfo.size;
    DBGPRINT(DBGSERVER, 2, "time_rcv:%" PRIu64 " get:%s\n", start_of_client, time_s_);

    // Get detailed address
    strtok_r(src_ip, ".", &savePtr);
    if (dst_addr == NULL)
    {
        // discard message:
        DBGMARK(DBGSERVER, 1, "id: %d discarding this message:%s \n", flow->flowinfo.flowid, savePtr);
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

void *CntThread(void *information)
{
    int reuse = 1;
    if (setsockopt(sock_for_cnt[0], IPPROTO_TCP, TCP_NODELAY, &reuse, sizeof(reuse)))
    {
        perror("setsockopt tcp_nodelay\n");
        return ((void *)0);
    }
    get_mss_cache(0);

    // 状态标志
    bool got_no_zero = false;
    bool got_alpha = false;

    // 训练用参数
    double time_delta = 0.0;
    double loss_bytes = 0.0;
    double avg_rtt = 0.0;
    double delay = 0.0;
    double min_rtt_ = 0.0;
    double pacing_rate = 0.0;
    double lost_rate = 0.0;
    double srtt_ms = 0.0;
    double packets_out = 0.0;
    double retrans_out = 0.0;
    double max_packets_out = 0.0;
    double snd_ssthresh = 0.0;
    double pre_bytes_acked = 0.0; /* used to calculate delivery rate */
    double delivery_rate = 0.0;

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

    // --------------------------------------- sage logic start ---------------------------------------
    char* shared_memory_rl2=  (char*)malloc(strlen(shared_memory_rl));
    u64 total_bytes_acked_pre=0,max_delivary_rate=0,pre_bytes_sent=0,pre_pkt_dlv=0,pre_pkt_lost=0;
    u64 max_sending_rate=0;

    dq_sage<u64>  sending_rates(WIN_SIZE*10);

    dq_sage<double> rtt_s(SHORT_WIN);
    dq_sage<double> rtt_m(MID_WIN);
    dq_sage<double> rtt_l(LONG_WIN);

    dq_sage<double> lost_s(SHORT_WIN);      //x = lost pkts / 1000
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

    dq_sage<u64>  sent_db(100);
    dq_sage<u64>  sent_dt(100);
    dq_sage<u64>  dlv_db(100);
    dq_sage<u64>  loss_db(100);
    dq_sage<u64>  uack_db(100);

    double dr_w_mbps=0.0,l_w_mbps=0.0,pre_dr_w_mbps=0;
    //
    dq_sage<double>  dr_w(200);     //2 Seconds?    //dr_w(4000); //40 Seconds
    dq_sage<double>     rtt_w(200);    //2 Seconds?

    double pre_dr_w_max=1.0;
    double dr_w_max=1.0;

    double cwnd_precise=CWND_INIT;
    double cwnd_rate=1.0;

    u64 dt=0; 
    u64 dt_pre=timestamp();
    uint64_t pre_calculation_time = timestamp();
    // ---------------------------------------- sage logic end ----------------------------------------

    uint64_t t0 = timestamp();
    usleep(report_period * 1000); // 首次等20 * 1000 = 20000 us = 20 m, 之后把t0改到每个回合结束前的一个RTT
    while (send_traffic)
    {
        got_no_zero = false;
        while (!got_no_zero && send_traffic)
        {
            get_tcp_info(0);
            get_his_rtts(0);
            get_min_rtt(0);
            /***
             * 内核中取出时间单位为uSec
             * Orca模型中使用的RTT单位为mSec
             * 速率单位为MBps
             */
            if (info.tcpi_rtt > 0)
            {
                // uint64_t t1 = timestamp();
                // time_delta = static_cast<double>(t1 - t0); // uSec
                // loss_bytes = static_cast<double>(info.tcpi_lost) * mss_cache;
                // // double temp_rate = static_cast<double>(info.tcpi_bytes_acked - pre_bytes_acked) / time_delta; // 本RTT计算得出的速率, MBps, TODO: delivery 的更新改ewma
                // delivery_rate = static_cast<double>(info.tcpi_delivery_rate);
                // avg_rtt = cal_avg_rtt();
                // delay = avg_rtt / 1000; // 加CNT
                // min_rtt_ = static_cast<double>(min_rtt) / 1000;
                // pacing_rate = static_cast<double>(info.tcpi_snd_mss) * (max(info.tcpi_snd_cwnd, info.tcpi_unacked)) * 1000 / info.tcpi_rtt;
                // lost_rate = loss_bytes / time_delta; // MBps
                // srtt_ms = static_cast<double>(info.tcpi_rtt) / 1000;
                // snd_ssthresh = (double)(info.tcpi_snd_ssthresh);
                // packets_out = (double)(info.tcpi_unacked);
                // retrans_out = (double)(info.tcpi_retrans);
                // max_packets_out = packets_out;

                // report_period = 20;
                // if (!slow_start_passed)
                // {
                //     slow_start_passed = (info.tcpi_snd_ssthresh < info.tcpi_snd_cwnd) ? 1 : 0;
                //     // orca在此处添加1.1x
                //     t0 = timestamp();
                //     usleep(min_rtt / 2);
                //     continue;
                // }

                // sprintf(message, "%d %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f",
                //         msg_id, delay, delivery_rate, (double)50.0, (double)time_delta,
                //         (double)target, (double)info.tcpi_snd_cwnd, pacing_rate, lost_rate, srtt_ms, snd_ssthresh, packets_out, retrans_out, max_packets_out, (double)info.tcpi_snd_mss, min_rtt_);
                
                
                // --------------------------------------- sage logic start ---------------------------------------
                
                // t1=timestamp();
                // if((t1-t0)<(report_period*1000))
                // {
                //     usleep(report_period*1000-t1+t0);
                // }

                u64 s_db;
                u64 d_dp = (info.tcpi_delivered-pre_pkt_dlv);   //Delivered Delta Packets
                u64 d_db = (d_dp)*info.tcpi_snd_mss;            //Delivered Delta Bytes
                u64 l_db;
                pre_pkt_dlv = info.tcpi_delivered; 

                if (info.tcpi_bytes_acked-total_bytes_acked_pre>0 || d_db>0)
                {
                    dt = timestamp()- dt_pre;
                    dt = (dt>0)?dt:1;
                    dt_pre = timestamp();
                    s_db = info.tcpi_bytes_sent-pre_bytes_sent;                 //Sent Delta Bytes
                    l_db = (info.tcpi_lost>pre_pkt_lost)?(info.tcpi_lost - pre_pkt_lost)*info.tcpi_snd_mss:0;
                    pre_pkt_lost = info.tcpi_lost;
                    uack_db.add((u64)info.tcpi_unacked*info.tcpi_snd_mss);
                    sent_db.add(s_db);
                    dlv_db.add(d_db);
                    loss_db.add(l_db);
                    sent_dt.add(dt);
                    
                    u64 dt_sum = sent_dt.sum();
                    dr_w_mbps = (double)8*dlv_db.sum()/dt_sum;
                    l_w_mbps  = (double)8*loss_db.sum()/dt_sum;
                    dr_w.add(dr_w_mbps);
                    dr_w_max = dr_w.max();
                    if (dr_w_max==0.0)
                        dr_w_max = 1;
                    rtt_w.add((double)info.tcpi_rtt/100000.0);

                    double time_delta=(double)(timestamp()-pre_calculation_time)/1000000.0;
                    pre_calculation_time = timestamp();
                    //we divide to mss instead of mss+60! To make sure that we always have the right integer value!
                    
                    double acked_rate = info.tcpi_bytes_acked-total_bytes_acked_pre;
                    acked_rate = acked_rate/time_delta;
                    total_bytes_acked_pre = info.tcpi_bytes_acked;

                    double sending_rate = (double)(info.tcpi_bytes_sent-pre_bytes_sent); //= (double)info.tcpi_snd_cwnd;
                    pre_bytes_sent = info.tcpi_bytes_sent;

                    double rtt_rate;                 
                    
                    max_delivary_rate = dr_w_max*1e6/8;
                    if(max_delivary_rate<=0)
                    {
                            acked_rate = 0;
                    }
            
                    if(info.tcpi_rtt>0)
                    {
                        sending_rate = sending_rate/time_delta; // Bps
                        sending_rates.add((u64)sending_rate);
                        max_sending_rate = sending_rates.max();
                        rtt_rate = (double)info.tcpi_min_rtt/info.tcpi_rtt;
                        time_delta = (1000000*time_delta)/info.tcpi_min_rtt;
                        acked_rate = acked_rate/max_sending_rate;
                    }
                    else
                    {
                        time_delta =report_period;
                        sending_rate = 0;
                        rtt_rate = 0.0;
                    }

                    u64 s_db_tmp =sent_db.sum();
                    u64 ua_db_tmp = uack_db.avg();
                    double cwnd_unacked_rate = (s_db_tmp>0)?(double)ua_db_tmp/s_db_tmp:(double)ua_db_tmp; //(double)info.tcpi_unacked/info.tcpi_snd_cwnd;

                    u64 cwnd_bits = (u64) info.tcpi_snd_cwnd*info.tcpi_snd_mss*8;
                    if(cwnd_bits==0)
                        cwnd_bits++;

                    /***
                    * Smoothed Versions ...
                    ***/
                    rtt_s.add((double)info.tcpi_rtt/100000);
                    rtt_m.add((double)info.tcpi_rtt/100000);
                    rtt_l.add((double)info.tcpi_rtt/100000);
                    
                    rtt_rate_s.add(rtt_rate);
                    rtt_rate_m.add(rtt_rate);
                    rtt_rate_l.add(rtt_rate);

                    rtt_var_s.add((double)info.tcpi_rttvar/1000.0);
                    rtt_var_m.add((double)info.tcpi_rttvar/1000.0);
                    rtt_var_l.add((double)info.tcpi_rttvar/1000.0);

                    thr_s.add((double)info.tcpi_delivery_rate/125000.0/BW_NORM_FACTOR);
                    thr_m.add((double)info.tcpi_delivery_rate/125000.0/BW_NORM_FACTOR);
                    thr_l.add((double)info.tcpi_delivery_rate/125000.0/BW_NORM_FACTOR);
                        
                    inflight_s.add((double)info.tcpi_unacked/1000.0);
                    inflight_m.add((double)info.tcpi_unacked/1000.0);
                    inflight_l.add((double)info.tcpi_unacked/1000.0);
                
                    lost_s.add((double)info.tcpi_lost/100.0);
                    lost_m.add((double)info.tcpi_lost/100.0); 
                    lost_l.add((double)info.tcpi_lost/100.0);

                    /**
                    * Soheil: Other interesting fields: potential input state candidates
                    * */
                    char message_extra[1000];
                    sprintf(message_extra,
                            "%.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f",
                            //4-12
                            rtt_s.get_avg(),rtt_s.get_min(),rtt_s.get_max(),
                            rtt_m.get_avg(),rtt_m.get_min(),rtt_m.get_max(),
                            rtt_l.get_avg(),rtt_l.get_min(),rtt_l.get_max(),
                            //13-21
                            thr_s.get_avg(),thr_s.get_min(),thr_s.get_max(),
                            thr_m.get_avg(),thr_m.get_min(),thr_m.get_max(),
                            thr_l.get_avg(),thr_l.get_min(),thr_l.get_max(),
                            //22-30
                            rtt_rate_s.get_avg(),rtt_rate_s.get_min(),rtt_rate_s.get_max(),
                            rtt_rate_m.get_avg(),rtt_rate_m.get_min(),rtt_rate_m.get_max(),
                            rtt_rate_l.get_avg(),rtt_rate_l.get_min(),rtt_rate_l.get_max(),
                            //31-39
                            rtt_var_s.get_avg(),rtt_var_s.get_min(),rtt_var_s.get_max(),
                            rtt_var_m.get_avg(),rtt_var_m.get_min(),rtt_var_m.get_max(),
                            rtt_var_l.get_avg(),rtt_var_l.get_min(),rtt_var_l.get_max(),
                            //40-48                             
                            inflight_s.get_avg(),inflight_s.get_min(),inflight_s.get_max(),
                            inflight_m.get_avg(),inflight_m.get_min(),inflight_m.get_max(),
                            inflight_l.get_avg(),inflight_l.get_min(),inflight_l.get_max(), 
                            //49-57
                            lost_s.get_avg(),lost_s.get_min(),lost_s.get_max(),
                            lost_m.get_avg(),lost_m.get_min(),lost_m.get_max(),
                            lost_l.get_avg(),lost_l.get_min(),lost_l.get_max());
                    
                    sprintf(message,
                            "%d %.7f %.7f %.7f %u %s %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f",
                            msg_id,
                            //0
                            (double)info.tcpi_rtt/100000.0, /*sRTT in 100x (ms):e.g. 2 = 2x100=200 ms*/
                            //1
                            (double)info.tcpi_rttvar/1000.0,    /*var of sRTT in 1x (ms). */
                            //2
                            (double)info.tcpi_delivery_rate/125000.0/BW_NORM_FACTOR, /*del rate 100x Mbps*/
                            //3
                            info.tcpi_ca_state,             /*TCP_CA_Open=0 -> TCP_CA_Loss=4*/
                            //4-57
                            message_extra,
                            //58
                            time_delta,
                            //59
                            rtt_rate, 
                            //60
                            l_w_mbps/BW_NORM_FACTOR,
                            //61
                            acked_rate,
                            //62
                            (pre_dr_w_mbps>0.0)?(dr_w_mbps/pre_dr_w_mbps):dr_w_mbps,
                            //63
                            (double)(dr_w_max*info.tcpi_min_rtt)/(cwnd_bits),
                            //64
                            (dr_w_mbps)/BW_NORM_FACTOR,
                            //65
                            cwnd_unacked_rate,
                            //66
                            (pre_dr_w_max>0.0)?(dr_w_max/pre_dr_w_max):dr_w_max,
                            //67
                            dr_w_max/BW_NORM_FACTOR,
                            //68
                            (cwnd_rate>0.0)?round(log2f(cwnd_rate)*1000)/1000.:log2f(0.0001));

                    pre_dr_w_mbps = dr_w_mbps;
                    pre_dr_w_max = dr_w_max;
                }
                else
                {
                    DBGMARK(0, 0, "Warming: no pkt acked this round, skip!\n");
                    continue;
                }
                // ---------------------------------------- sage logic end ----------------------------------------
                // original logic
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
                    DBGMARK(0, 0, "No valid state for 1 min. We (server of Actor %d) are going down down down ...\n", actor_id);
                    send_traffic = false;
                }
                usleep(report_period * 100);
            } // endif rtt>0
        }

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
                if (pre_id_tmp != pre_id) // 确保来了新值
                {
                    got_alpha = true;
                    pre_id = pre_id_tmp;


                    // int i_alpha = atoi(alpha); // 保证alpha在50-400之间取值
                    // i_alpha = min(400, i_alpha);
                    // i_alpha = max(50, i_alpha);
                    // target_ratio = floor(static_cast<double>(info.tcpi_snd_cwnd) * i_alpha / 100);

                    // --------------------------------------- sage logic start ---------------------------------------
                    cwnd_precise = mul(cwnd_precise, atof(alpha));
                    
                    u64 cwnd_tmp;
                    if(ROUNT_TYPE==FLOORING)
                    {
                        cwnd_tmp = floor(cwnd_precise);
                    }
                    else if (ROUNT_TYPE==CEILING)
                    {
                        cwnd_tmp = ceil(cwnd_precise);
                    }
                    else if (ROUNT_TYPE==ROUNDING)
                    {
                        cwnd_tmp = round(cwnd_precise);
                    }
                    else
                    {
                        cwnd_tmp = (u64)(cwnd_precise);
                    }
                    
                    if(cwnd_tmp>=MAX_32Bit)
                        target_ratio = MAX_32Bit;
                    else
                        target_ratio = cwnd_tmp;
                    // ---------------------------------------- sage logic end ----------------------------------------

                    if (target_ratio < MIN_CWND)
                    {
                        target_ratio = MIN_CWND;
                    }
                    else if(target_ratio > MAX_CWND)
                    {
                        target_ratio = MAX_CWND;
                        cwnd_precise = MAX_CWND;
                    }
                    // 在这先不设置CWND
                    // if (setsockopt(sock_for_cnt[0], IPPROTO_TCP, TCP_CWND, &target_ratio, sizeof(target_ratio)) < 0)
                    // {
                    //     perror("setsockopt: set target ratio");
                    //     return;
                    // }
                    get_alpha_error_counter = 0;
                }
                else
                {
                    if (get_new_alpha_error_counter > 1000)
                    {
                        DBGPRINT(DBGSERVER, 0, "still no new value, id:%d prev_id:%d\n", pre_id_tmp, pre_id);
                        get_new_alpha_error_counter = 0;
                    }
                    get_new_alpha_error_counter++;
                    usleep(1000);
                }
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
            }
        }

        // 1. evaluation stage
        // 当前rl的cwnd存在target_ratio中，开始挑选最佳CWND
        u_rl.cwnd = target_ratio;
        // 时间过去了一会，更新一下info
        get_tcp_info(0);
        u_cl.cwnd = info.tcpi_snd_cwnd;
        if (u_prev.utility == 0) // 首次初始化为二者中更小的那个
            u_prev.cwnd = u_cl.cwnd < u_rl.cwnd ? u_cl.cwnd : u_rl.cwnd;

        struct utility_value *first_run = u_cl.cwnd < u_rl.cwnd ? &u_cl : &u_rl;
        struct utility_value *second_run = u_cl.cwnd < u_rl.cwnd ? &u_rl : &u_cl;
        uint32_t ONE_WAY_DELAY = static_cast<uint32_t>(cal_avg_rtt() / 2);
        if (first_run != NULL && second_run != NULL)
        {
            set_cwnd(first_run->cwnd, 0);
            usleep(ONE_WAY_DELAY);
            set_cwnd(second_run->cwnd, 0);
            usleep(ONE_WAY_DELAY);
            set_cwnd(u_prev.cwnd, 0);
            get_utility_value(u_prev.cwnd, &u_prev); // 1RTT时取prev的RTT
            usleep(ONE_WAY_DELAY);
            get_utility_value(first_run->cwnd, first_run); // 1.5RTT取first_run的u
            usleep(ONE_WAY_DELAY);
            get_utility_value(second_run->cwnd, second_run); // 2 RTT取second_run的u
        }
        else
        {
            DBGPRINT(DBGSERVER, 0, "Get pointer failed!");
            return ((void *)0);
        }

        // 2. calculating the optimal rate
        int situation = cal_situation();
        uint64_t optimal_cwnd = cal_optimal_cwnd(situation);
        u_prev.cwnd = optimal_cwnd;
        get_confidence_val(u_cl);
        get_confidence_val(u_rl);
        FILE *file = fopen("/home/snow/pantheon-available/third_party/anole/log_server.txt", "a");
        if (file)
        {
            fprintf(file, "Current rtt = %u", ONE_WAY_DELAY);
            fprintf(file, "cwnd of cl, rl, prev are : %lu, %lu, %lu\n", u_cl.cwnd, u_rl.cwnd, u_prev.cwnd);
            fprintf(file, "utility of optimal, cl, rl, prev are : %.2f,%.2f,%.2f,%.2f\n", u_optimal, u_cl.utility, u_rl.utility, u_prev.utility);
            fprintf(file, "confidence val of optimal, cl, rl, prev are : %.2f,%.2f\n", u_cl.confidence_val, u_rl.confidence_val);
        }
        fclose(file);

        // PROBING
        // 3. check whether enter probing or acceleration stage
        if (u_cl.confidence_val >= ETA_OFF && u_rl.confidence_val >= ETA_OFF)
        {
            // PROBING
            set_cwnd(optimal_cwnd, 0);
            get_his_rtts(0);
            uint32_t avg_rtt = static_cast<uint32_t>(cal_avg_rtt());
            usleep(avg_rtt);
            get_tcp_info(0);
            pre_bytes_acked = static_cast<double>(info.tcpi_bytes_acked);
            t0 = timestamp(); // 退出阶段之后要运行一个rtt
            usleep(avg_rtt);
        }
        else
        {
            // ACCELERATION
            bool flag_cl = u_cl.confidence_val >= ETA_OFF;
            bool flag_rl = u_rl.confidence_val >= ETA_OFF;
            set_cwnd(optimal_cwnd, 0);
            get_his_rtts(0);
            uint32_t avg_rtt = static_cast<uint32_t>(cal_avg_rtt() * 1.5); // 直接运行1.5个RTT
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

                get_min_rtt(0);
                get_his_rtts(0);
                avg_rtt = static_cast<uint32_t>(cal_avg_rtt() * 1.5);
                if (static_cast<double>(avg_rtt) / min_rtt > 1 + MU)
                {
                    set_cwnd(--optimal_cwnd, 0);
                }
                else
                {
                    set_cwnd(++optimal_cwnd, 0);
                }
            }
            get_tcp_info(0);
            pre_bytes_acked = static_cast<double>(info.tcpi_bytes_acked);
            t0 = timestamp(); // 退出阶段之后要运行一个rtt 用于数据统计
            usleep(avg_rtt);
        }

    } // end while send traffic
    shmdt(shared_memory);
    shmctl(shmid, IPC_RMID, NULL);
    shmdt(shared_memory_rl);
    shmctl(shmid_rl, IPC_RMID, NULL);
    return ((void *)0);
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
    if (setsockopt(sock[0], SOL_SOCKET, SO_REUSEADDR, (const char *)&reuse, sizeof(reuse)) < 0)
    {
        perror("setsockopt(SO_REUSEADDR) failed");
    }

    // Bind socket on IP:Port
    if (bind(sock[0], (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) < 0)
    {
        DBGMARK(0, 0, "bind error srv_ctr_ip: 000000: %s\n", strerror(errno));
        close(sock[0]);
        return;
    }

    // Set kernel Congestion control
    if (scheme)
    {
        if (setsockopt(sock[0], IPPROTO_TCP, TCP_CONGESTION, scheme, strlen(scheme)) < 0)
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
    // sprintf(cmd, "/usr/bin/python %s/d5.py --load --tb_interval=1 --base_path=%s --task=%d --job_name=actor --mem_r=%d --mem_w=%d&", path, path, actor_id, (int)key, (int)key_rl);
    sprintf(cmd,"/home/snow/venvpy37/bin/python /home/snow/pantheon-available/third_party/sage/sage_rl/rl_module/tcpactor.py --mem_r=%d --mem_w=%d --id=%d &", (int)key, (int)key_rl, actor_id);
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
            DBGERROR("After 3 minutes, no response (OK_Signal) from the Actor %d is received! We are going down down down ...\n", actor_id);
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
        int val = accept(sock[0], (struct sockaddr *)&client_addr, (socklen_t *)&sin_size);
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

    shmdt(shared_memory);
    shmctl(shmid, IPC_RMID, NULL);
    shmdt(shared_memory_rl);
    shmctl(shmid_rl, IPC_RMID, NULL);
    return 0;
}