// Copyright ©2022-2022 BYD Company Ltd. All rights reserved.
#ifndef TIMER_H
#define TIMER_H
#define MAX_PTPDEV_NAME 32
#include <chrono>
class HPTimer
{
 public:
uint64_t NowMicroSecond()
{
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(duration);
    return microseconds.count();
}

double NowMilliSecond()
{
    return NowMicroSecond() * 1e-3;
}

double NowSecond()
{
    return NowMicroSecond() * 1e-6;
}

HPTimer(): m_started(false), m_paused(false)
{
    Start();
}

void Start()
{
    m_started = true;
    m_paused = false;
    m_startTime = NowMicroSecond();
}

void Restart()
{
    m_started = false;
    Start();
}

void Pause()
{
    m_paused = true;
    m_pauseTime = NowMicroSecond();
}

void Resume()
{
    m_paused = false;
    m_startTime += NowMicroSecond() - m_pauseTime;
}

void Reset()
{
    m_started = false;
    m_paused = false;
}

double ElapsedMicroSeconds() 
{
    if (!m_started) {
        return 0.0;
    }
    if (m_paused) {
        return m_pauseTime - m_startTime;
    } else {
        return NowMicroSecond() - m_startTime;
    }
}

double ElapsedMilliSeconds() 
{
    return ElapsedMicroSeconds() / 1e3;
}

double ElapsedSeconds() 
{
    return ElapsedMicroSeconds() / 1e6;
}

double ElapsedMinutes() 
{
    return ElapsedSeconds() / 60;
}

double ElapsedHours() 
{
    return ElapsedMinutes() / 60;
}

 private:
    bool m_started;
    bool m_paused;
    int64_t m_startTime;
    int64_t m_pauseTime;
};

#endif // HPTIMER_H
