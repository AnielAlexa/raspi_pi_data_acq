#ifndef GLOG_LOGGING_H
#define GLOG_LOGGING_H

#include <iostream>
#include <cstdlib>
#include <sstream>

enum LogSeverity {
    INFO = 0,
    WARNING = 1,
    ERROR = 2,
    FATAL = 3
};

// Simple stream logger
class LoggerStub {
public:
    LoggerStub(bool abort_on_destruct = false) : abort_(abort_on_destruct) {}
    ~LoggerStub() {
        std::cerr << stream_.str() << std::endl;
        if (abort_) std::abort();
    }
    template <typename T>
    LoggerStub& operator<<(const T& msg) {
        stream_ << msg;
        return *this;
    }
private:
    std::stringstream stream_;
    bool abort_;
};

class NullLogger {
public:
    template <typename T>
    NullLogger& operator<<(const T&) { return *this; }
};

#define LOG(severity) LoggerStub( (severity) == FATAL )
#define VLOG(n) NullLogger()

#define CHECK(condition) \
    if (!(condition)) LoggerStub(true) << "Check failed: " #condition << " "

#define CHECK_NOTNULL(val) \
    if ((val) == nullptr) LoggerStub(true) << "Check failed: " #val << " is null "

#define CHECK_EQ(val1, val2) \
    if ((val1) != (val2)) LoggerStub(true) << "Check failed: " #val1 " == " #val2 << " "

#define CHECK_NE(val1, val2) \
    if ((val1) == (val2)) LoggerStub(true) << "Check failed: " #val1 " != " #val2 << " "

#define CHECK_GT(val1, val2) \
    if ((val1) <= (val2)) LoggerStub(true) << "Check failed: " #val1 " > " #val2 << " "

#define CHECK_LT(val1, val2) \
    if ((val1) >= (val2)) LoggerStub(true) << "Check failed: " #val1 " < " #val2 << " "

#define CHECK_GE(val1, val2) \
    if ((val1) < (val2)) LoggerStub(true) << "Check failed: " #val1 " >= " #val2 << " "

#define CHECK_LE(val1, val2) \
    if ((val1) > (val2)) LoggerStub(true) << "Check failed: " #val1 " <= " #val2 << " "

#endif
