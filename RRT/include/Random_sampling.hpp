#ifndef _RANDOM_SAMPLING_
#define _RANDOM_SAMPLING_

#include "utils.hpp"

namespace Utils {
  template <class T>
  class RandomSampler {
  private:
    double alpha;
    double beta;
  public:
    RandomSampler() : alpha(0.1), beta(0.2) {}
    RandomSampler(double alpha, double beta) {
      this->alpha = alpha;
      this->beta = beta;
    }
    Point<T>& LineTo(Point<T>& goal, Point<T>& closest);
    Point<T>& Uniform();
    Point<T>& Ellipsis(Point<T>& start, Point<T>& goal);
  };

}  // namespace Utils

#endif
