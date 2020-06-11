#pragma once

#include <time.h>
#include <chrono>

class RTTimer
{
	const unsigned int kSecondsToNanoseconds = 1e9;

private:
	struct timespec time_;
	double frequency_{1000.0};

public:
	RTTimer() = default;
	virtual ~RTTimer(){};

	/**
     * @brief Sets timer frequency
     * @param[in] f Frequency of the timer
     */
	void SetFrequency(const double& f) { frequency_ = f; }

	/**
     * @brief Gets timer frequency
     * @return Timer frequency
     */
	double Frequency() const { return frequency_; }

	/**
     * @brief Waits for next tick
     */
	void WaitForTick()
	{
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &time_, NULL);
		time_.tv_nsec += ((1.0 / frequency_) * kSecondsToNanoseconds);
		while (time_.tv_nsec >= kSecondsToNanoseconds)
		{
			time_.tv_nsec -= kSecondsToNanoseconds;
			time_.tv_sec++;
		}
	}

	/**
     * @brief Starts timer after a delay
     * @param[in] delay Delay in seconds
     */
	void StartWithDelay(const unsigned int& delay)
	{
		clock_gettime(CLOCK_MONOTONIC, &time_);
		time_.tv_sec += delay;
	}
};

class NRTTimer
{
private:
	std::chrono::_V2::system_clock::time_point start_;
	std::chrono::_V2::system_clock::time_point stop_;

public:
	NRTTimer() = default;
	virtual ~NRTTimer(){};

	/**
     * @brief Starts timer
     */
	void Start() { start_ = std::chrono::high_resolution_clock::now(); }

	/**
     * @brief Stops timer
     */
	void Stop() { stop_ = std::chrono::high_resolution_clock::now(); }

	/**
     * @brief Returns duration in us between start and stop times
     * @return Duration in microseconds
     */
	int64_t Duration() const
	{
		return std::chrono::duration_cast<std::chrono::microseconds>(stop_ - start_)
			.count();
	}
};
