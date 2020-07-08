#include <stdio.h>

class sequence
{    
    int period_;
    int start_;
    int end_;
    std::array<std::pair<int,int>, 5> pulses;

public:
    sequence(int start, int duration, int interval, int repeat) 
        : period_(2*interval+3*duration),
          start_(start),
          end_(start+repeat*period_),
          pulses
          {{
              {0                    , duration             }, // pulse 1
              {interval             , interval  +  duration}, // pulse 2
              {2*interval           , 2*interval+  duration}, // pulse 3
              {2*interval+  duration, 2*interval+2*duration}, // pulse 4
              {2*interval+2*duration, period_              }  // pulse 5
          }}
    {
        if(duration <= 0){
            throw std::runtime_error("Duration must be positive integer.");
        }

        if(interval < 0){
            throw std::runtime_error("Interval must be non negative integer.");
        }
    }

    bool isActive(int time, std::size_t idx) const
    {
        const auto& pulse = pulses[idx];

        // 0 for each start time of sequence (pulse 1)
        const auto refTime = (time - start_)%period_;

        return (pulse.first <= refTime) && (refTime < pulse.second) && (time < end_);
    }

    int getPeriod() const{
        return period_;
    }

    int getStartTime() const{
        return start_;
    }

    int getEndTime() const{
        return end_;
    }

    std::size_t getPulseNum() const{
        return pulses.size();
    }
};