#include <stdio.h>

class sequence
{    
    int period_;
    int start_;
    int end_;
    std::array<std::pair<int,int>, 1> pulses;

public:
    sequence() :
        sequence(0,1,1,0) {}
    sequence(int start, int duration, int interval, int repeat) 
        : period_(interval * 1000),
          start_(start * 1000),
          end_(start_ + repeat*period_),
          pulses
          {{
              {0, duration * 1000}, // pulse 1
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

    void setParams(int start, int duration, int interval, int repeat) {
          period_ = interval * 1000;
          start_ = start * 1000;
          end_ = start_ + repeat*period_;
          std::array<std::pair<int,int>, 1> temp
          {{
              {0., duration*1000}, // pulse 1
          }};
          pulses = temp;
    }

};