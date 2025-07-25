#ifndef __LOG_HPP__
#define __LOG_HPP__

#include <iostream>
#include <string>

struct Logger {
  void Warn(std::string s) {
    std::cerr << "[WARN] " << s << std::endl;
  }
  void Info(std::string s) {
    std::cerr << "[INFO] " << s << std::endl;
  }
  void Debug(std::string s) {
    std::cerr << "[DEBUG] " << s << std::endl;
  }
  void Error(std::string s) {
    std::cout << "[ERROR] " << s << std::endl;
  }
} log;

#endif
