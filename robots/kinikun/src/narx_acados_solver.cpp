#include "mpa_acados_controller/narx_acados_solver.h"

namespace mpa_acados_controller {

NarxAcadosSolver::NarxAcadosSolver()
  : capsule_(nullptr), pmax_(0.7) {}

NarxAcadosSolver::~NarxAcadosSolver() {
  if (capsule_) {
    narx_disc_acados_free(capsule_);
    narx_disc_acados_free_capsule(capsule_);
    capsule_ = nullptr;
  }
}

bool NarxAcadosSolver::init(double pmax) {
  pmax_ = pmax;
  capsule_ = narx_disc_acados_create_capsule();
  if (!capsule_) return false;
  if (narx_disc_acados_create(capsule_) != 0) return false;

  // print_level を上げる（あなたのビルドで存在確認済みのフィールド）
  ocp_nlp_config *nlp_config = narx_disc_acados_get_nlp_config(capsule_);
  void *nlp_opts = narx_disc_acados_get_nlp_opts(capsule_);
  int print_level = 2;
  ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "print_level", &print_level);

#if ACADOS_DIAG >= 1
  std::printf("[acados] dims: N=%d nx=%d nu=%d (pmax=%.3f)\n",
              NARX_DISC_N, NARX_DISC_NX, NARX_DISC_NU, pmax_);
#endif
  return true;
}

void NarxAcadosSolver::setX0(const std::vector<double> &x0) {
  if (!capsule_) return;

  ocp_nlp_config *nlp_config = narx_disc_acados_get_nlp_config(capsule_);
  ocp_nlp_dims   *nlp_dims   = narx_disc_acados_get_nlp_dims(capsule_);
  ocp_nlp_in     *nlp_in     = narx_disc_acados_get_nlp_in(capsule_);
  ocp_nlp_out    *nlp_out    = narx_disc_acados_get_nlp_out(capsule_);

  const int nx = NARX_DISC_NX;
  const int N  = NARX_DISC_N;

  std::vector<double> x_init(nx, 0.0);
  const int copy_n = std::min<int>(nx, x0.size());
  if (copy_n > 0) std::memcpy(x_init.data(), x0.data(), sizeof(double)*copy_n);

  // 解ベクトルの初期推定 x（0..N 全ステージ）
  for (int stage = 0; stage <= N; ++stage)
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, stage, "x", (void*)x_init.data());

  // x(0)=x0 等式拘束（多くのテンプレートで nbx_0=nx 前提）
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", (void*)x_init.data());
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", (void*)x_init.data());

  // 入力の初期推定 u（0..N-1）
  const double ps0 = x_init[1];
  const double pd0 = x_init[2];
  double a0 = (ps0 + pd0) / (2.0 * pmax_);
  double b0 = (ps0 - pd0) / (2.0 * pmax_);
  a0 = std::min(1.0, std::max(0.0, a0));
  b0 = std::min(1.0, std::max(0.0, b0));
  double u_init[2] = {a0, b0};
  for (int stage = 0; stage < N; ++stage)
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, stage, "u", (void*)u_init);
}

void NarxAcadosSolver::setParams(double theta_ref, double dz0) {
  if (!capsule_) return;

  last_theta_ref_ = theta_ref;
  last_dz0_ = dz0;

  double p[2] = {theta_ref, dz0};
  const int N = NARX_DISC_N;
  for (int stage = 0; stage <= N; ++stage)
    narx_disc_acados_update_params(capsule_, stage, p, 2);
}

bool NarxAcadosSolver::solve() {
  if (!capsule_) return false;

  const int status = narx_disc_acados_solve(capsule_);
  if (status) {
    // 失敗時に最小限の診断
    ocp_nlp_solver *solver = narx_disc_acados_get_nlp_solver(capsule_);

    int sqp_iter  = 0;
    int qp_status = 0;
    ocp_nlp_get(solver, "sqp_iter",  &sqp_iter);
    ocp_nlp_get(solver, "qp_status", &qp_status);

    double inf_norm_res[4] = {0,0,0,0}; // [stat, eq, ineq, comp]
    ocp_nlp_get(solver, "inf_norm_res", inf_norm_res);

    std::printf("[acados] FAIL: status=%d sqp_iter=%d qp_status=%d | res[stat,eq,ineq,comp]=[%.2e, %.2e, %.2e, %.2e]\n",
                status, sqp_iter, qp_status, inf_norm_res[0], inf_norm_res[1], inf_norm_res[2], inf_norm_res[3]);

#if ACADOS_DIAG >= 1
    dumpStage0();
    std::printf("p(0)  [theta_ref,dz0]=[%.4g, %.4g]\n", last_theta_ref_, last_dz0_);
#endif
    return false;
  }

#if ACADOS_DIAG >= 2
  // 成功時も、たまにサマリ（ここでは毎回 1 行）
  std::printf("[acados] OK\n");
#endif
  return true;
}

bool NarxAcadosSolver::isOk() const {
  return (capsule_ != nullptr);
}

bool NarxAcadosSolver::getU0(double &a, double &b) const {
  if (!capsule_) return false;
  ocp_nlp_config *nlp_config = narx_disc_acados_get_nlp_config(capsule_);
  ocp_nlp_dims   *nlp_dims   = narx_disc_acados_get_nlp_dims(capsule_);
  ocp_nlp_out    *nlp_out    = narx_disc_acados_get_nlp_out(capsule_);

  double u0[NARX_DISC_NU] = {0.0};
  ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", u0);

  a = std::min(1.0, std::max(0.0, u0[0]));
  b = std::min(1.0, std::max(0.0, u0[1]));
  return true;
}

bool NarxAcadosSolver::getControl(double &p1, double &p2) const {
  double a=0.0, b=0.0;
  if (!getU0(a, b)) return false;
  const double ps = pmax_ * (a + b);
  const double pd = pmax_ * (a - b);
  p1 = 0.5 * (ps + pd);
  p2 = 0.5 * (ps - pd);
  return true;
}

void NarxAcadosSolver::dumpStage0() const {
  ocp_nlp_config *cfg  = narx_disc_acados_get_nlp_config(capsule_);
  ocp_nlp_dims   *dims = narx_disc_acados_get_nlp_dims(capsule_);
  ocp_nlp_out    *out  = narx_disc_acados_get_nlp_out(capsule_);

  const int nx = NARX_DISC_NX;
  const int nu = NARX_DISC_NU;

  std::vector<double> x0(nx, 0.0), u0(nu, 0.0);
  ocp_nlp_out_get(cfg, dims, out, 0, "x", x0.data());
  ocp_nlp_out_get(cfg, dims, out, 0, "u", u0.data());
  printVec("x(0) ", x0.data(), nx, 6);
  printVec("u(0) ", u0.data(), nu, 2);
}

} // namespace mpa_acados_controller
