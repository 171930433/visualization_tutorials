#pragma once
#include <cmath>

inline size_t s2us(const double t_s) { return static_cast<size_t>(round(t_s * 1.0e6)); }
inline size_t s2ms(const double t_s) { return static_cast<size_t>(round(t_s * 1.0e3)); }
inline double ms2s(const size_t t_ms) { return (t_ms / 1000.0); }
