#ifndef __PREDICTOR_HPP__
#define __PREDICTOR_HPP__

struct Predictor {
  enum Type {
    DEFAULT
  };
  struct Ctx {
    Type type;
    int Qj, Qk;
    int PC, offset;
  };
  virtual bool operator()(Ctx ctx) = 0;
  virtual ~Predictor() = default;
};

struct NaivePredictor : public Predictor {
  ~NaivePredictor() override = default;
  bool operator()(Ctx ctx) override {
    return true;
  }
};

#endif