#include <ecal/ecal.h>
#include <ecal/msg/protobuf/subscriber.h>
#include <robot_sdk.pb.h>
#include <iostream>
#include <iomanip>
#include <thread>
#include <cstdio>

static FILE* fp = nullptr;

void OnStateReceived(const char* topic_name, const robot_sdk::pb::RobotState& msg,
                     long long time, long long clock, long long id) {
    static int count = 0;
    count++;
    
    if (!fp) {
        fp = fopen("/tmp/robot_mujoco_state.log", "w");
        if (!fp) return;
        fprintf(fp, "# count bytes ");
        fprintf(fp, "q_abad[0..3] q_hip[0..3] q_knee[0..3] ");
        fprintf(fp, "qd_abad[0..3] qd_hip[0..3] qd_knee[0..3] ");
        fprintf(fp, "tau_abad[0..3] tau_hip[0..3] tau_knee[0..3] ");
        fprintf(fp, "quat[0..3] gyro[0..2] acc[0..2] ");
        fprintf(fp, "pos[0..2] v_world[0..2] rpy[0..2] ts\n");
    }
    
    // 每次都写入文件
    fprintf(fp, "%d %zu ", count, msg.ByteSizeLong());
    
    auto print_repeated = [&](const char* name, int size, auto getter) {
        for (int i = 0; i < size; i++) fprintf(fp, "%.6f ", getter(i));
    };
    
    print_repeated("q_abad", msg.q_abad_size(), [&](int i){ return msg.q_abad(i); });
    fprintf(fp, " ");
    print_repeated("q_hip", msg.q_hip_size(), [&](int i){ return msg.q_hip(i); });
    fprintf(fp, " ");
    print_repeated("q_knee", msg.q_knee_size(), [&](int i){ return msg.q_knee(i); });
    fprintf(fp, " ");
    print_repeated("qd_abad", msg.qd_abad_size(), [&](int i){ return msg.qd_abad(i); });
    fprintf(fp, " ");
    print_repeated("qd_hip", msg.qd_hip_size(), [&](int i){ return msg.qd_hip(i); });
    fprintf(fp, " ");
    print_repeated("qd_knee", msg.qd_knee_size(), [&](int i){ return msg.qd_knee(i); });
    fprintf(fp, " ");
    print_repeated("tau_abad", msg.tau_abad_fb_size(), [&](int i){ return msg.tau_abad_fb(i); });
    fprintf(fp, " ");
    print_repeated("tau_hip", msg.tau_hip_fb_size(), [&](int i){ return msg.tau_hip_fb(i); });
    fprintf(fp, " ");
    print_repeated("tau_knee", msg.tau_knee_fb_size(), [&](int i){ return msg.tau_knee_fb(i); });
    fprintf(fp, " ");
    print_repeated("quat", msg.quat_size(), [&](int i){ return msg.quat(i); });
    fprintf(fp, " ");
    print_repeated("gyro", msg.gyro_size(), [&](int i){ return msg.gyro(i); });
    fprintf(fp, " ");
    print_repeated("acc", msg.acc_size(), [&](int i){ return msg.acc(i); });
    fprintf(fp, " ");
    print_repeated("pos", msg.position_size(), [&](int i){ return msg.position(i); });
    fprintf(fp, " ");
    print_repeated("v_world", msg.v_world_size(), [&](int i){ return msg.v_world(i); });
    fprintf(fp, " ");
    print_repeated("rpy", msg.rpy_size(), [&](int i){ return msg.rpy(i); });
    fprintf(fp, " %lu", (unsigned long)msg.time_stamp());
    fprintf(fp, "\n");
    
    // 控制台: 每 500 条输出一次摘要
    if (count <= 3 || count % 500 == 0) {
        std::cout << "[RX] #" << count << ": " << msg.ByteSizeLong() << "B"
                  << " | pos=[" << std::fixed << std::setprecision(4)
                  << (msg.position_size()>0 ? msg.position(0) : 0) << ","
                  << (msg.position_size()>1 ? msg.position(1) : 0) << ","
                  << (msg.position_size()>2 ? msg.position(2) : 0) << "]"
                  << " quat_w=" << (msg.quat_size()>0 ? msg.quat(0) : 0)
                  << " gyro=[" << std::setprecision(6)
                  << (msg.gyro_size()>0 ? msg.gyro(0) : 0) << ","
                  << (msg.gyro_size()>1 ? msg.gyro(1) : 0) << ","
                  << (msg.gyro_size()>2 ? msg.gyro(2) : 0) << "]"
                  << " rpy_size=" << msg.rpy_size()
                  << " ts=" << msg.time_stamp()
                  << std::endl;
    }
}

int main(int argc, char** argv) {
    eCAL::Initialize(argc, argv, "ecal_state_monitor");
    
    eCAL::protobuf::CSubscriber<robot_sdk::pb::RobotState> sub("mujoco_state");
    sub.AddReceiveCallback(std::bind(OnStateReceived, std::placeholders::_1,
                                     std::placeholders::_2, std::placeholders::_3,
                                     std::placeholders::_4, std::placeholders::_5));
    
    std::cout << "Listening to mujoco_state topic..." << std::endl;
    std::cout << "Press Ctrl+C to stop." << std::endl;
    
    while (eCAL::Ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    eCAL::Finalize();
    return 0;
}
