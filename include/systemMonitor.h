#include <mutex>

namespace co_lrio
{
class systemMonitor
{
private:
    // system monitor
    float time_window_f; // sec
    int time_window_i;   // sec
    int nth_received;   // n-th received
    size_t window_head, window_tail;
    // hz
    std::vector<unsigned int> msg_rcv_count;
    unsigned int msg_rcv_count_sum;
    // runtime
    std::vector<float> runtime;
    float runtime_sum;
    std::vector<size_t> runtime_count;
    size_t runtime_count_sum;
    // bw
    std::vector<size_t> msg_byte_size;
    size_t msg_byte_size_sum;
    rclcpp::TimerBase::SharedPtr monitor_timer;

    std::atomic<bool> mutex_;
    std::string prefix;

public:
    systemMonitor(rclcpp::Node* node_ptr, int time_window = 10)
    {
        // system monitor
        nth_received = 0;
        time_window_i = time_window;
        time_window_f = (float)time_window;
        window_head = time_window - 1;
        window_tail = 0;
        msg_rcv_count.resize(time_window + 1);
        msg_byte_size.resize(time_window + 1);
        msg_rcv_count.assign(time_window + 1, 0);
        msg_byte_size.assign(time_window + 1, 0);
        runtime.resize(time_window + 1);
        runtime.assign(time_window + 1, 0.0);
        runtime_count.resize(time_window + 1);
        runtime_count.assign(time_window + 1, 0);
        msg_rcv_count_sum = 0;
        msg_byte_size_sum = 0;
        runtime_sum = 0.0;
        runtime_count_sum = 0;
        mutex_.store(false, std::memory_order_release);

        prefix = node_ptr->get_name();

        auto print_msg = [this]()
        {
            const size_t s = msg_rcv_count.size();
            // ************************************************
            //Mutex locked (Get and increase index)
            while (mutex_.exchange(true, std::memory_order_acquire));
            const size_t prev_head = window_head;
            window_head = (prev_head + 1) % s;
            // reset
            msg_rcv_count[window_head] = 0;
            msg_byte_size[window_head] = 0;
            runtime[window_head] = 0.0;
            runtime_count[window_head] = 0;
            // Mutex unlocked
            mutex_.store(false, std::memory_order_release);
            // ************************************************
            msg_rcv_count_sum += msg_rcv_count[prev_head];
            msg_byte_size_sum += msg_byte_size[prev_head];
            runtime_sum += runtime[prev_head];
            runtime_count_sum += runtime_count[prev_head];

            const float hz = (float)msg_rcv_count_sum / time_window_f;
            const float bw = (float)msg_byte_size_sum / time_window_f / 1024.0;
            const float rt = (float)runtime_sum / time_window_f;
            const float rtc = (float)std::max(int(runtime_count_sum),1) / time_window_f;
            if (nth_received >= time_window_i)
                RCLCPP_INFO(rclcpp::get_logger("monitor"), "(In %ds) KF opt hz: %.2f; rcv bw: %.2f kBps; avg opt time: %.2f ms for %d frames",
                    time_window_i, hz, bw, rt/rtc, runtime_count_sum);
            else
            {
                nth_received += 1;
                RCLCPP_WARN(rclcpp::get_logger("monitor"), "(In %ds) KF opt hz: %.2f; rcv bw: %.2f kBps; avg opt time: %.2f ms for %d frames <- [Invalid] Data too small for the window.",
                    time_window_i, hz, bw, rt/rtc, runtime_count_sum);
            }

            msg_rcv_count_sum -= msg_rcv_count[window_tail];
            msg_byte_size_sum -= msg_byte_size[window_tail];
            runtime_sum -= runtime[window_tail];
            runtime_count_sum -= runtime_count[window_tail];

            // Update tail
            window_tail = (window_tail + 1) % s;
        };
        monitor_timer = node_ptr->create_wall_timer(1s, print_msg);
    }

    void addReceviedMsg(int msg_size, bool enable_hz = false)
    {
        while (mutex_.exchange(true, std::memory_order_acquire));
        if (enable_hz)
        {
            msg_rcv_count[window_head] += 1;
        }
        msg_byte_size[window_head] += msg_size;
        mutex_.store(false, std::memory_order_release);
    }

    void addOptimizeTime(float time)
    {
        while (mutex_.exchange(true, std::memory_order_acquire));
        runtime[window_head] += time;
        runtime_count[window_head] += 1;
        mutex_.store(false, std::memory_order_release);
    }
};
}  