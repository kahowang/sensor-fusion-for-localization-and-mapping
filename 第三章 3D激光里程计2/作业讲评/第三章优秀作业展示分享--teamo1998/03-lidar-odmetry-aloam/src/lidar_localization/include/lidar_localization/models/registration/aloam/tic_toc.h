#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_TIC_TOC_H_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_TIC_TOC_H_

#include <ctime>
#include <cstdlib>
#include <chrono>
namespace lidar_localization {
class TicToc
{
  public:
    TicToc()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};
}
#endif