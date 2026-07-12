#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

class DataLogger {
public:
    static constexpr size_t kRingSize = 4096;

    explicit DataLogger(const std::string& dir = "logs")
        : read_idx_(0), queue_size_(0), running_(true)
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
            file_ << "timestamp_s,level,message\n";
            file_.flush();
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
            printf("[DATALOGGER] Stopped (%zu entries): %s\n", total_written_, filepath_.c_str());
    }

    DataLogger(const DataLogger&) = delete;
    DataLogger& operator=(const DataLogger&) = delete;

    void LogError(const std::string& msg)
    {
        Enqueue("ERROR", msg);
    }

    void LogWarning(const std::string& msg)
    {
        Enqueue("WARNING", msg);
    }

    bool IsOpen() const { return file_.is_open(); }
    const std::string& GetFilepath() const { return filepath_; }

private:
    struct LogEntry {
        double timestamp_sec;
        std::string level;
        std::string message;
    };

    void Enqueue(const char* level, const std::string& msg)
    {
        auto now = std::chrono::system_clock::now();
        double ts = std::chrono::duration<double>(now.time_since_epoch()).count();

        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (queue_size_ >= kRingSize) return;
        auto& e = ring_[(read_idx_ + queue_size_) % kRingSize];
        e.timestamp_sec = ts;
        e.level = level;
        e.message = msg;
        queue_size_++;
        cv_.notify_one();
    }

    void WriterLoop()
    {
        while (running_.load(std::memory_order_relaxed)) {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            cv_.wait_for(lock, std::chrono::milliseconds(10), [this]() {
                return queue_size_ > 0 || !running_.load(std::memory_order_relaxed);
            });
            DrainQueue(lock);
        }
        std::unique_lock<std::mutex> lock(queue_mutex_);
        DrainQueue(lock);
    }

    void DrainQueue(std::unique_lock<std::mutex>& /*lock*/)
    {
        while (queue_size_ > 0) {
            const auto& e = ring_[read_idx_];
            file_ << std::fixed << std::setprecision(6) << e.timestamp_sec
                  << ',' << e.level << ',';
            WriteEscaped(e.message);
            file_ << '\n';
            read_idx_ = (read_idx_ + 1) % kRingSize;
            queue_size_--;
            total_written_++;
        }
        if (total_written_ > 0)
            file_.flush();
    }

    void WriteEscaped(const std::string& s)
    {
        bool need_quote = false;
        for (char c : s) {
            if (c == ',' || c == '"' || c == '\n') { need_quote = true; break; }
        }
        if (need_quote) {
            file_ << '"';
            for (char c : s) {
                if (c == '"') file_ << '"';
                else file_ << c;
            }
            file_ << '"';
        } else {
            file_ << s;
        }
    }

    std::array<LogEntry, kRingSize> ring_;
    size_t read_idx_;
    size_t queue_size_ = 0;
    std::mutex queue_mutex_;
    std::condition_variable cv_;
    std::ofstream file_;
    std::string filepath_;
    std::thread writer_;
    std::atomic<bool> running_;
    size_t total_written_{0};
};
