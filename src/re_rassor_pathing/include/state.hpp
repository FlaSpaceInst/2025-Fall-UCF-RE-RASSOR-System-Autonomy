#pragma once
#include <utility>
#include <cstdint>
#include <functional>

namespace dstarlite {


struct State {
  int x {0};
  int y {0};

  State() = default;
  State(int xi, int yi) : x(xi), y(yi) {}

  bool operator==(const State& o) const noexcept { return x == o.x && y == o.y; }
  bool operator!=(const State& o) const noexcept { return !(*this == o); }
  bool operator<(const State& o) const noexcept {
    return (x < o.x) || (x == o.x && y < o.y);
  }
};

struct Key {
  double k1 {0.0};
  double k2 {0.0};
  bool operator<(const Key& o) const noexcept {
    if (k1 < o.k1) return true;
    if (k1 > o.k1) return false;
    return k2 < o.k2;
  }
  bool operator==(const Key& o) const noexcept {
    return k1 == o.k1 && k2 == o.k2;
  }
};

struct StateHash {
  std::size_t operator()(const State& s) const noexcept {
    
    return (static_cast<std::size_t>(s.x) * 73856093u) ^
           (static_cast<std::size_t>(s.y) * 19349663u);
  }
};

} // namespace dstarlite
