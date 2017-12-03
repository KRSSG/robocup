// This file contains the class CS(CriticalSection)

#ifndef CRITICAL_SECTION_HPP
#define CRITICAL_SECTION_HPP

#ifdef WIN32
# include <winsock2.h>
#else
# include <pthread.h>
# include <errno.h>
# include <string.h>
# include <stdio.h>
#endif // !WIN32

namespace Util
{
  class CS
  {
  private:
#ifdef WIN32
    CRITICAL_SECTION cs;
#else
    pthread_mutex_t mutex;
#endif // !WIN32

  public:
    inline CS()
    {
#ifdef WIN32
      InitializeCriticalSection(&cs);
#else
      if (pthread_mutex_init(&mutex, NULL) != 0)
      {
        fprintf(stderr, "Failed to initialize mutex: %s", strerror(errno));
      }
#endif // !WIN32
    }

    inline ~CS()
    {
#ifdef WIN32
      DeleteCriticalSection(&cs);
      #else
      if (pthread_mutex_destroy(&mutex) != 0)
      {
        fprintf(stderr, "Failed to destroy mutex: %s", strerror(errno));
      }
#endif // WIN32
    }

    inline void enter()
    {
#ifdef WIN32
      EnterCriticalSection(&cs);
#else
      if (pthread_mutex_lock(&mutex) != 0)
      {
        fprintf(stderr, "Failed to lock mutex: %s", strerror(errno));
      }
#endif // !WIN32

    }

    inline void leave()
    {
#ifdef WIN32
      LeaveCriticalSection(&cs);
#else
      if (pthread_mutex_unlock(&mutex) != 0)
      {
        fprintf(stderr, "Failed to unlock mutex: %s", strerror(errno));
      }
#endif // !WIN32
    }

    inline void tryEnter()
    {
#ifdef WIN32
      TryEnterCriticalSection(&cs);
#else
      if (pthread_mutex_trylock(&mutex) != 0)
      {
        fprintf(stderr, "Failed trying to lock mutex: %s", strerror(errno));
      }
#endif // !WIN32
    }
  }; // class CS
} // namespace Util

#endif // CRITICAL_SECTION_HPP
