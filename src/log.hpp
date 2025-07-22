#ifndef __LOG_HPP__
#define __LOG_HPP__

#include <iostream>
#include <string>

struct Logger {
  void Warn(std::string s) {
    std::cout << "[WARN] " << s << std::endl;
  }
  void Info(std::string s) {
    std::cout << "[INFO] " << s << std::endl;
  }
} log;

#endif
