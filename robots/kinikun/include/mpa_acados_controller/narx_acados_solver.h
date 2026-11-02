#pragma once
#include <vector>
#include <memory>
#include <cstdio>
#include <algorithm>
#include <cstring>

extern "C" {
#include "c_generated_code/acados_solver_narx_disc.h"
}

// ==========================================
// 診断ログのトグル
//  0: 失敗時だけ最小限
//  1: 0 + 失敗回の x(0), u(0), p(0)
//  2: 1 + 成功時もたまに要約
// ==========================================
#ifndef ACADOS_DIAG
#define ACADOS_DIAG 1
#endif

namespace mpa_acados_controller {

class NarxAcadosSolver {
public:
  NarxAcadosSolver();
  ~NarxAcadosSolver();

  bool init(double pmax);
  void setX0(const std::vector<double>& x0);
  void setParams(double theta_ref, double dz0);

  bool solve();                              // RTI 1回
  bool isOk() const;

  bool getU0(double &a, double &b) const;    // stage0 の u
  bool getControl(double &p1, double &p2) const;

  int nx() const { return NARX_DISC_NX; }
  int nu() const { return NARX_DISC_NU; }
  int N()  const { return NARX_DISC_N;  }

  // （任意）最後に設定した参照を外から見たい時
  double lastThetaRef() const { return last_theta_ref_; }
  double lastDz0()      const { return last_dz0_;      }

private:
  // ========== ヘルパ ==========
  static inline void printVec(const char* tag, const double* v, int n, int maxn=6) {
    int m = std::min(n, maxn);
    std::printf("%s[0:%d]:", tag, m);
    for (int i = 0; i < m; ++i) std::printf(" %.4g", v[i]);
    if (n > m) std::printf(" ... (n=%d)", n);
    std::printf("\n");
  }

  // 成否に関わらずステージ0の x,u を抜いて出す（必要時のみ呼ぶ）
  void dumpStage0() const;

private:
  narx_disc_solver_capsule *capsule_;
  double pmax_ = 0.7;

  // 診断用に保持（setParams で更新）
  double last_theta_ref_ = 0.0;
  double last_dz0_       = 0.0;
};

} // namespace mpa_acados_controller
