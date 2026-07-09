#pragma once

#include <atomic>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

struct LogSnapshot {
    double timestamp_sec;
    std::array<float, 12> motor_pos;
    std::array<float, 12> motor_vel;
    std::array<float, 12> motor_torque;
    std::array<float, 12> motor_cmd;
    std::array<float, 3>  imu_gyro;
    std::array<float, 3>  imu_gravity;
    float xbox_linear_x;
    float xbox_linear_y;
    float xbox_angular_z;
};

class DataLogger {
public:
    static constexpr size_t kRingSize = 10000;

    explicit DataLogger(const std::string& dir = "logs")
        : write_idx_(0), read_idx_(0), running_(true), writing_(false)
    {
        auto now = std::chrono::system_clock::now();
        auto tt = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << dir << "/log_" << std::put_time(std::localtime(&tt), "%Y%m%d_%H%M%S") << ".csv";

        std::filesystem::path p(ss.str());
        if (p.has_parent_path())
            std::filesystem::create_directories(p.parent_path());

        file_.open(ss.str());
        if (file_.is_open()) {
            filepath_ = ss.str();
            WriteHeader();
            writer_ = std::thread(&DataLogger::WriterLoop, this);
            printf("[DATALOGGER] Started: %s\n", filepath_.c_str());
        } else {
            fprintf(stderr, "[DATALOGGER] Failed to open: %s\n", ss.str().c_str());
        }
    }

    ~DataLogger()
    {
        running_ = false;
        if (writer_.joinable()) writer_.join();
        if (file_.is_open()) file_.close();
        if (!filepath_.empty())
            printf("[DATALOGGER] Stopped (%zu samples): %s\n", total_written_, filepath_.c_str());
    }

    DataLogger(const DataLogger&) = delete;
    DataLogger& operator=(const DataLogger&) = delete;

    void Log(const LogSnapshot& s)
    {
        size_t next = (write_idx_.load(std::memory_order_relaxed) + 1) % kRingSize;
        if (next == read_idx_.load(std::memory_order_acquire)) return;
        ring_[write_idx_.load(std::memory_order_relaxed)] = s;
        write_idx_.store(next, std::memory_order_release);
    }

    bool IsOpen() const { return file_.is_open(); }
    const std::string& GetFilepath() const { return filepath_; }

private:
    void WriteHeader()
    {
        file_ << "timestamp_s";
        for (int i = 0; i < 12; ++i) {
            file_ << ",j" << i << "_pos,j" << i << "_vel,j" << i << "_trq,j" << i << "_cmd";
        }
        file_ << ",imu_gx,imu_gy,imu_gz,imu_gravx,imu_gravy,imu_gravz";
        file_ << ",xbox_lx,xbox_ly,xbox_az\n";
        file_.flush();
    }

    void WriterLoop()
    {
        while (running_.load(std::memory_order_relaxed)) {
            bool did_work = false;
            while (write_idx_.load(std::memory_order_acquire) != read_idx_.load(std::memory_order_relaxed)) {
                const auto& s = ring_[read_idx_];
                WriteRow(s);
                read_idx_.store((read_idx_.load(std::memory_order_relaxed) + 1) % kRingSize,
                                std::memory_order_release);
                total_written_++;
                did_work = true;
            }
            if (!did_work) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        while (write_idx_.load(std::memory_order_acquire) != read_idx_.load(std::memory_order_relaxed)) {
            const auto& s = ring_[read_idx_];
            WriteRow(s);
            read_idx_.store((read_idx_.load(std::memory_order_relaxed) + 1) % kRingSize,
                            std::memory_order_release);
            total_written_++;
        }
    }

    void WriteRow(const LogSnapshot& s)
    {
        file_ << std::fixed << std::setprecision(6) << s.timestamp_sec;
        for (int i = 0; i < 12; ++i)
            file_ << ',' << s.motor_pos[i] << ',' << s.motor_vel[i]
                  << ',' << s.motor_torque[i] << ',' << s.motor_cmd[i];
        file_ << ',' << s.imu_gyro[0] << ',' << s.imu_gyro[1] << ',' << s.imu_gyro[2]
              << ',' << s.imu_gravity[0] << ',' << s.imu_gravity[1] << ',' << s.imu_gravity[2]
              << ',' << s.xbox_linear_x << ',' << s.xbox_linear_y << ',' << s.xbox_angular_z
              << '\n';
    }

    std::array<LogSnapshot, kRingSize> ring_;
    std::atomic<size_t> write_idx_;
    std::atomic<size_t> read_idx_;
    std::ofstream file_;
    std::string filepath_;
    std::thread writer_;
    std::atomic<bool> running_;
    std::atomic<bool> writing_;
    size_t total_written_{0};
};
