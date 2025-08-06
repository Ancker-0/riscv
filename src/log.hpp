#ifndef __LOG_HPP__
#define __LOG_HPP__

#include <iostream>
#include <string>

#define LOG 0

struct Logger {
  void Warn(std::string s) {
#if LOG
    std::cerr << "[WARN] " << s << std::endl;
#endif
  }
  void Info(std::string s) {
    std::cerr << "[INFO] " << s << std::endl;
  }
  void Debug(std::string s) {
#if LOG
    std::cerr << "[DEBUG] " << s << std::endl;
#endif
  }
  void Error(std::string s) {
    std::cerr << "[ERROR] " << s << std::endl;
  }
  std::string Red(std::string s) {
    return "\033[31m" + s + "\033[0m";
  }
} log;

#endif
