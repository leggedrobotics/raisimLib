//
// Created by jhwangbo on 18. 1. 1.
//

#ifndef RAISIM_MESSAGE_LOGGER_HPP
#define RAISIM_MESSAGE_LOGGER_HPP

#include <chrono>
#include <ostream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <functional>

namespace raisim {

constexpr int RSEVERITY_INFO = 0;
constexpr int RSEVERITY_WARN = 1;
constexpr int RSEVERITY_FATAL = 2;


class RaiSimMsg {
 public:
  void stream(const char *file, const int line, std::stringstream &msg, int severity) {
    time_t currentTime;
    struct tm tm_time;

    time(&currentTime);                   // Get the current time
    localtime_r(&currentTime, &tm_time);  // Convert the current time to the local time

    const char *filename_start = file;
    const char *filename = filename_start;
    while (*filename != '\0')
      filename++;
    while ((filename != filename_start) && (*(filename - 1) != '/'))
      filename--;

    std::stringstream printout;

    std::string color;

    switch (severity) {
      case 0:
        color = "";
        break;
      case 1:
        color = "\033[33m";
        break;
      case 2:
        color = "\033[1;31m";
        break;
    }

    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    printout << "[" << std::setfill('0')
             << std::put_time(std::localtime(&in_time_t), "%Y:%m:%d:%X")<< ' '
             << std::setfill(' ')
             << filename
             << ':' << line << "] " << color << msg.str() << "\033[0m\n";

    std::cout << printout.str();

    if (severity == RSEVERITY_FATAL)
      fatalCallback_();
  }

  static void setFatalCallback(std::function<void()> fatalCallback) {
    fatalCallback_ = fatalCallback;
  }

 private:
  std::stringstream log;
  static std::function<void()> fatalCallback_;
};


}

#endif //RAISIM_MESSAGE_LOGGER_HPP
