#pragma once

#include <optional>
#include <functional>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

namespace conway {

#if defined( __EMSCRIPTEN_PTHREADS__ )

class ThreadPool {
public:

  ThreadPool() {

    for ( size_t where = 1, end = std::thread::hardware_concurrency(); where < end; ++where ) {

      threads_.emplace_back( [&]() {

        worker();

      } );
    }
  }

  ~ThreadPool() {

    exit_ = true;

    condition_.notify_all();

    for ( std::thread& thread : threads_ ) {

      thread.join();
    }
  }


  /** Simple parallel for, not re-entrant */
  template < typename FunctionType >
  void parallel_for( size_t start, size_t end, FunctionType function, size_t threadStride = 2, size_t minParallelBatch = 4 ) {

    if ( ( end - start ) < minParallelBatch ) {

      for ( size_t where = start; where < end; ++where ) {

        function( where );
      }

      return;
    }

    std::function< void ( size_t ) > wrapper( function );

    {
      std::unique_lock< std::mutex > latch( lock_ );

      // A worker from the previous round may still be inside work() with a
      // stale end_/iteration_ - it would grab indices from the reset counter
      // against the old bound and call the old (dangling) function. Wait for
      // all workers to leave work() before publishing the new round.
      join_.wait( latch, [&]() { return activeWorkers_.load() == 0; } );

      complete_     = start;
      threadStride_ = threadStride;
      end_          = end;
      counter_      = start;

      iteration_ = &wrapper;
    }

    condition_.notify_all();

    work();

    {
      std::unique_lock<std::mutex> latch( lock_ );

      join_.wait(
        latch,
        [&]() {
          return complete_.load() >= end && activeWorkers_.load() == 0;
        } );
    }

    iteration_ = nullptr;
  }

  static ThreadPool& instance() { 

    if ( !instance_.has_value() ) {

      instance_.emplace();
    }

    return instance_.value();
  }

private:

  void worker() {

    while ( !exit_.load() ) {

      {
        std::unique_lock<std::mutex> latch( lock_ );

        condition_.wait( latch, [&]() { return exit_ || counter_.load() < end_.load(); } );

        if ( exit_ ) {
          return;
        }

        activeWorkers_.fetch_add( 1 );

        // Release the pool mutex while running iterations - holding it here
        // serialised the workers, leaving at most one of them (plus the
        // calling thread) doing real work.
      }

      work();

      activeWorkers_.fetch_sub( 1 );

      // Acquire/release the mutex between the state change and the notify so
      // the waiter in parallel_for cannot check its predicate, miss this
      // update, and then block forever on a notify that already fired.
      { std::lock_guard< std::mutex > guard( lock_ ); }

      join_.notify_one();
    }
  }

  void work() {

    size_t stride = threadStride_.load();

    while ( true ) {

      // Re-read the bound every pass: relying on a value cached before the
      // fetch_add lets a straggler overrun a smaller follow-up round.
      size_t end    = end_.load();
      size_t cursor = counter_.fetch_add( stride );

      if ( cursor >= end ) {
        break;
      }

      size_t cursorEnd = std::min( cursor + stride, end );

      while ( cursor < cursorEnd ) {

        (*iteration_)( cursor );

        ++cursor;

        if ( ( ++complete_ ) >= end ) {

          // Fence (see worker()) so the join waiter cannot miss the final
          // completion.
          { std::lock_guard< std::mutex > guard( lock_ ); }

          join_.notify_one();
        }
      }
    }
  }
  
  std::atomic< size_t > threadStride_ = 0;
  std::atomic< size_t > end_ = 0;
  std::atomic< size_t > counter_ = 0;
  std::atomic< size_t > complete_ = 0;
  std::atomic< size_t > activeWorkers_ = 0;

  std::atomic< bool > exit_ = false;

  std::mutex lock_;
  std::condition_variable condition_;
  std::condition_variable join_;

  std::vector< std::thread > threads_;

  std::function< void ( size_t ) >* iteration_ = nullptr;

  static std::optional< ThreadPool > instance_;

};

#else

class ThreadPool {
  public:
  
    ThreadPool() {
    }
  
    ~ThreadPool() {
    }
  
  
    /** Simple parallel for, not re-entrant */
    template < typename FunctionType > 
    void parallel_for( size_t start, size_t end, FunctionType function, [[maybe_unused]]size_t threadStride = 1 ) {
  
      for ( size_t where = start; where < end; ++where ) {

        function( where );
      }
    }
  
    static ThreadPool& instance() { 
  
      if ( !instance_.has_value() ) {
  
        instance_.emplace();
      }
  
      return instance_.value();
    }
  
  private:
    
    static std::optional< ThreadPool > instance_;
  
  };
  

#endif

}