#ifndef TIMER_H_INL
#define TIMER_H_INL

#include <cassert>
#include <sys/time.h>

namespace Strategy
{
  class Timer
  {
  private:
    struct timeval timeStart, timeSplit, timeStop;
    int intervalms_, intervalus_;
    bool timerStarted;

  public:
    Timer() :
      timerStarted(false)
    { }

    ~Timer()
    { }

    inline void start(void)
    {
      int ret = gettimeofday(&timeStart, NULL);
      assert(ret == 0);
      timerStarted = true;
    }

    inline int split(void)
    {
      assert(timerStarted == true);
      int ret = gettimeofday(&timeSplit, NULL);
      assert(ret == 0);
      return ((timeSplit.tv_sec - timeStart.tv_sec) * 1000 + (timeSplit.tv_usec - timeStart.tv_usec) / 1000);
    }

    inline int stopms(void)
    {
      assert(timerStarted == true);
      int ret = gettimeofday(&timeStop, NULL);
      assert(ret == 0);
      timerStarted = false;
      return (intervalms_ = (timeStop.tv_sec - timeStart.tv_sec) * 1000 + (timeStop.tv_usec - timeStart.tv_usec) / 1000);
    }

    inline int stopus(void)
    {
      assert(timerStarted == true);
      int ret = gettimeofday(&timeStop, NULL);
      assert(ret == 0);
      timerStarted = false;
      return (intervalus_ = (timeStop.tv_sec - timeStart.tv_sec) * 1000000 + (timeStop.tv_usec - timeStart.tv_usec));
    }
    
    inline int intervalms(void) const
    {
      return intervalms_;
    }
    
    inline int intervalus(void) const
    {
      return intervalus_;
    }
  };
}

#endif // TIMER_H_INL
