#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NARX+z 1-step SLSQP コントローラのスタンドアロン・シミュレータ
- 入力:  narx_common_meta.npz（degree, lag_y, lag_u, delay, coef[], z_coef[], z_feat_names[], dt）
- 出力:  CSV（時系列: theta_ref, theta_pred, p1, p2, ps, pd, z, etc.）
- 目的関数/制約/履歴更新は ROS 版と同一

使い方例:
  python narx_sqp_sim.py \
    --meta /home/kan/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/kinikun/scripts/pressure_control/narx/narx_common_meta.npz \
    --steps 400 --theta0_deg 0 --target_deg 30 --step_t 0.5 \
    --p_ind_max 0.70 --reg_scale 0.9/4096 \
    --w_theta 10.0 --w_z 1.0 --w_u 0.1 \
    --u_delta_clip 0.10 \
    --dt_override 0.01 \
    --csv out_sim.csv --plot
"""

import os, math, argparse, csv
import numpy as np
import matplotlib


# ---- deps (SciPy, scikit-learn 必須) ----
try:
    from sklearn.preprocessing import PolynomialFeatures
except Exception:
    raise RuntimeError("scikit-learn が必要です（PolynomialFeatures）。")

try:
    from scipy.optimize import minimize
except Exception:
    raise RuntimeError("SciPy が必要です（SLSQP 最適化）。")

def clamp(x, lo, hi): return max(lo, min(hi, x))
def ps_pd_to_p12(ps, pd): return 0.5*(ps+pd), 0.5*(ps-pd)

def z_features(ps, pd, names):
    vals = {
        '1': 1.0,
        'ps': float(ps),
        'pd': float(pd),
        'ps*pd': float(ps)*float(pd),
        'ps^2': float(ps)*float(ps),
        'pd^2': float(pd)*float(pd),
    }
    try:
        return np.array([vals[nm] for nm in names], float)
    except KeyError as e:
        raise RuntimeError(f"z_feat_names に未知の名前があります: {e}. names={names}")

class NarxSqpSim:
    def __init__(self, meta_npz_path, p_ind_max, w_theta, w_z, w_u, u_delta_clip, reg_scale,
                 theta0_deg, dt_override=None, verbose_every=50):
        if not os.path.exists(meta_npz_path):
            raise RuntimeError(f"meta_npz_path が見つかりません: {meta_npz_path}")
        meta = np.load(meta_npz_path, allow_pickle=True)

        self.degree  = int(meta["degree"])
        self.lag_y   = int(meta["lag_y"])
        self.lag_u   = int(meta["lag_u"])
        self.delay   = int(meta["delay"])
        self.coef    = np.array(meta["coef"], dtype=float).reshape(-1)
        self.dt_meta = float(meta["dt"])
        self.z_coef  = np.array(meta["z_coef"], dtype=float).reshape(-1)
        self.z_names = [str(s) for s in meta["z_feat_names"]]

        if self.z_coef.shape[0] != len(self.z_names):
            raise RuntimeError(f"z_coef の次元不一致: len(z_coef)={self.z_coef.shape[0]} vs len(z_feat_names)={len(self.z_names)}")

        self.n_in = self.lag_y + 2*self.lag_u
        self.pf = PolynomialFeatures(degree=self.degree, include_bias=True)
        self.pf.fit(np.zeros((1, self.n_in), float))

        # パラメータ
        self.p_ind_max    = float(p_ind_max)
        self.w_theta      = float(w_theta)
        self.w_z          = float(w_z)
        self.w_u          = float(w_u)
        self.u_delta_clip = float(u_delta_clip)
        self.reg_scale    = float(reg_scale)
        self.verbose_every= int(verbose_every)

        # 状態（履歴は現在値から繰り返し）
        self.theta_now_deg = float(theta0_deg)
        self.theta_ref_deg = float(theta0_deg)
        self.y_hist = [self.theta_now_deg]*max(1, self.lag_y)
        self.u_hist = [[0.0, 0.0]]*max(1, self.lag_u+self.delay)
        self.ps_prev = 0.05
        self.pd_prev = 0.0

        # 時間刻み
        self.dt = float(dt_override) if (dt_override is not None) else self.dt_meta

        print(f"[Sim] loaded {meta_npz_path} | deg={self.degree} lag_y={self.lag_y} lag_u={self.lag_u} delay={self.delay} dt={self.dt:.4f}s")
        print(f"[Sim] bounds: 0<=p1,p2<= {self.p_ind_max:.3f} MPa | scale={self.reg_scale:.6f}")

    # NARX 予測（同一）
    def _narx_predict_deg(self, ps, pd):
        feats = []
        feats.extend(self.y_hist[:self.lag_y])
        start = self.delay
        for k in range(start, start+self.lag_u):
            ps_k, pd_k = self.u_hist[k]
            feats.extend([ps_k, pd_k])
        x  = np.array(feats, float).reshape(1, -1)
        xp = self.pf.transform(x)
        return float(np.dot(xp.reshape(-1), self.coef.reshape(-1)))

    # z 値（同一）
    def _z_value(self, ps, pd):
        return float(np.dot(z_features(ps, pd, self.z_names), self.z_coef))
    
    def _narx_rollout_deg(self, ps, pd, steps=None):
        # ↓ ここを delay → delay+1 に
        if steps is None: steps = max(1, int(self.delay) + 1)
        y_hist = list(self.y_hist)
        u_hist = [list(u) for u in self.u_hist]
        th_next = y_hist[0]
        for _ in range(steps):
            feats = []
            feats.extend(y_hist[:self.lag_y])
            start = self.delay
            for k in range(start, start + self.lag_u):
                ps_k, pd_k = u_hist[k]
                feats.extend([ps_k, pd_k])
            x = np.array(feats, float).reshape(1, -1)
            xp = self.pf.transform(x)
            th_next = float(np.dot(xp.reshape(-1), self.coef.reshape(-1)))
            y_hist = [th_next] + y_hist[:-1]
            u_hist = [[ps, pd]] + u_hist[:-1]
        return th_next
    # 目的関数（同一）
    def _objective(self, u):
        ps, pd = float(u[0]), float(u[1])
        # ↓ ここも delay → delay+1 に
        th_pred_d = self._narx_rollout_deg(ps, pd, steps=max(1, self.delay + 1))
        e_th = th_pred_d - self.theta_ref_deg
        zval = self._z_value(ps, pd)
        du2  = (ps - self.ps_prev)**2 + (pd - self.pd_prev)**2
        return self.w_theta*(e_th*e_th) + self.w_z*(zval*zval) + self.w_u*du2

    # 制約（同一）
    def _bounds(self):
        ps_hi = 2.0*self.p_ind_max
        return [(0.0, ps_hi), (-ps_hi, ps_hi)]

    def _ineq_constraints(self):
        return [{"type":"ineq", "fun": lambda u: float(u[0] - abs(u[1]))}]  # |pd| <= ps

    # 1 ステップ最適化（同一＋安全クリップ）
    def _solve_once(self):
        u0 = np.array([self.ps_prev, self.pd_prev], float)
        res = minimize(self._objective, u0, method="SLSQP",
                       bounds=self._bounds(), constraints=self._ineq_constraints(),
                       options=dict(maxiter=50, ftol=1e-9, disp=False))
        ps, pd = (u0 if not res.success else res.x)
        # 1周期の変化クリップ
        ps = self.ps_prev + clamp(ps - self.ps_prev, -self.u_delta_clip, self.u_delta_clip)
        pd = self.pd_prev + clamp(pd - self.pd_prev, -self.u_delta_clip, self.u_delta_clip)
        # 個別圧へ＆クリップ
        p1, p2 = ps_pd_to_p12(ps, pd)
        p1 = clamp(p1, 0.0, self.p_ind_max)
        p2 = clamp(p2, 0.0, self.p_ind_max)
        # 再合成（安全側）
        ps = p1 + p2
        pd = p1 - p2
        return ps, pd, p1, p2, res.success

    def _update_hist(self, ps, pd, theta_deg):
        self.y_hist = [theta_deg] + self.y_hist[:-1]
        self.u_hist = [[ps, pd]] + self.u_hist[:-1]

# ===== NarxSqpSim 内に追加/置換 =====

    def _debug_sensitivity(self, eps: float = 1e-3) -> None:
        """
        d-step 先の予測角度 θ_pred_d の ps/pd 感度を有限差分で計算して表示する。
        現在の内部状態 (ps_prev, pd_prev) まわりで評価する。
        """
        steps = max(1, self.delay + 1)
        base = self._narx_rollout_deg(self.ps_prev, self.pd_prev, steps=steps)

        dps_p = self._narx_rollout_deg(self.ps_prev + eps, self.pd_prev, steps=steps) - base
        dps_m = self._narx_rollout_deg(self.ps_prev - eps, self.pd_prev, steps=steps) - base
        dpd_p = self._narx_rollout_deg(self.ps_prev, self.pd_prev + eps, steps=steps) - base
        dpd_m = self._narx_rollout_deg(self.ps_prev, self.pd_prev - eps, steps=steps) - base

        dth_dps = (dps_p - dps_m) / (2.0 * eps)
        dth_dpd = (dpd_p - dpd_m) / (2.0 * eps)

        print("[DEBUG] sensitivities: dθ/dps≈{:.6f}  dθ/dpd≈{:.6f}  (at ps={:.3f}, pd={:.3f})"
            .format(dth_dps, dth_dpd, self.ps_prev, self.pd_prev))


    def _objective_value(self, ps: float, pd: float) -> float:
        """
        与えた (ps,pd) に対する目的関数 J を返す（制約違反は大罰）。
        J = wθ*(θ_pred_d - θ_ref)^2 + wz*z^2 + wu*||Δu||^2
        """
        # 制約: 0 <= p1,p2 <= p_ind_max  ⇔  0<=ps<=2*p_ind_max, |pd|<=ps
        if ps < 0.0 or ps > 2.0 * self.p_ind_max or abs(pd) > ps:
            return 1e9

        steps = max(1, self.delay + 1)
        th_pred_d = self._narx_rollout_deg(ps, pd, steps=steps)
        e_th = th_pred_d - self.theta_ref_deg
        zval = self._z_value(ps, pd)
        du2 = (ps - self.ps_prev) ** 2 + (pd - self.pd_prev) ** 2

        return self.w_theta * (e_th * e_th) + self.w_z * (zval * zval) + self.w_u * du2


    def _debug_grid_search(self, n: int = 8) -> tuple:
        """
        粗いグリッドで J を走査し最良 (ps,pd) を探すデバッガ。
        戻り値: (ps_best, pd_best, J_best)
        """
        ps_max = 2.0 * self.p_ind_max
        best = (None, None, float("inf"))

        for i in range(n + 1):
            ps = ps_max * i / n
            # 制約 |pd| <= ps を満たすように -ps..ps を等分
            for j in range(-n, n + 1):
                pd = ps * j / n
                val = self._objective_value(ps, pd)
                if val < best[2]:
                    best = (ps, pd, val)

        print("[DEBUG] grid best: ps={:.3f} pd={:.3f}  J={:.3f}".format(best[0], best[1], best[2]))
        return best


    def step_once(self, theta_ref_deg: float) -> dict:
        """
        1 ステップ進める。必要に応じて初回ターゲット印加時に感度/粗探索を実行。
        """
        self.theta_ref_deg = float(theta_ref_deg)

        # --- デバッグ: ステップ入力が入った最初のフレームで一度だけ可視化 ---
        if (abs(self.theta_ref_deg - self.theta_now_deg) > 1e-6) and (self.ps_prev == 0.0) and (self.pd_prev == 0.0):
            # 感度の表示（u への依存が本当にあるか）
            self._debug_sensitivity(eps=1e-3)
            # 粗探索で J 地形をざっくり確認（最良が (0,0) ならモデル依存が弱い可能性）
            self._debug_grid_search(n=8)

        # --- 最適化（SLSQP） ---
        ps, pd, p1, p2, ok = self._solve_once()

        # 1-step 予測（ログ/出力用）と z 値
        th_pred = self._narx_predict_deg(ps, pd)
        zval = self._z_value(ps, pd)

        # 内部更新（ここでは “実角度＝予測角度” として閉ループ同定的に更新）
        self._update_hist(ps, pd, th_pred)
        self.ps_prev, self.pd_prev = ps, pd

        # 参考: d-step 先の角度も併記すると効き具合が見やすい
        th_pred_d = self._narx_rollout_deg(self.ps_prev, self.pd_prev, steps=max(1, self.delay + 1))

        return {
            "theta_ref_deg": self.theta_ref_deg,
            "theta_pred_deg": th_pred,
            "theta_pred_d_deg": th_pred_d,
            "p1_MPa": p1, "p2_MPa": p2,
            "ps_MPa": ps, "pd_MPa": pd,
            "z": zval, "ok": bool(ok),
            "mpa_cmd_x": p1 * self.reg_scale,
            "mpa_cmd_y": p2 * self.reg_scale,
        }

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--meta", required=True, help="narx_common_meta.npz のパス")
    ap.add_argument("--steps", type=int, default=400)
    ap.add_argument("--theta0_deg", type=float, default=0.0)
    ap.add_argument("--target_deg", type=float, default=30.0, help="ステップで与える目標角度[deg]")
    ap.add_argument("--step_t", type=float, default=0.5, help="ステップ入力を開始する時刻[s]")
    ap.add_argument("--dt_override", type=float, default=None, help="NPZのdtを上書き（省略でNPZのdt）")
    ap.add_argument("--p_ind_max", type=float, default=0.70)
    ap.add_argument("--w_theta", type=float, default=10.0)
    ap.add_argument("--w_z", type=float, default=1.0)
    ap.add_argument("--w_u", type=float, default=0.1)
    ap.add_argument("--u_delta_clip", type=float, default=0.10)
    ap.add_argument("--reg_scale", type=float, default=(4096/0.9), help="圧力→コマンド変換のスケール（逆数をデフォルトに）")
    ap.add_argument("--csv", default="narx_sqp_sim.csv")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--verbose_every", type=int, default=50)
    # argparse 追加
    ap.add_argument("--save_png", default=None, help="プロットをPNG保存して終了（ウィンドウを開かない）")
    ap.add_argument("--plot_show", action="store_true", help="ウィンドウ表示（ブロッキング）")
    args = ap.parse_args()

    sim = NarxSqpSim(
        meta_npz_path=args.meta,
        p_ind_max=args.p_ind_max,
        w_theta=args.w_theta, w_z=args.w_z, w_u=args.w_u,
        u_delta_clip=args.u_delta_clip, reg_scale=args.reg_scale,
        theta0_deg=args.theta0_deg,
        dt_override=args.dt_override,
        verbose_every=args.verbose_every
    )

    # ターゲット生成（t>=step_t で target_deg へステップ）
    dt = sim.dt
    t = np.arange(args.steps)*dt
    theta_ref_series = np.where(t >= float(args.step_t), float(args.target_deg), float(args.theta0_deg))

    rows = []
    for i in range(args.steps):
        out = sim.step_once(theta_ref_series[i])
        out["t"] = float(t[i])
        rows.append(out)
        if (i % sim.verbose_every) == 0:
            print(f"[{i:04d}] t={t[i]:.3f}s | θ_ref={out['theta_ref_deg']:.2f} θ_pred={out['theta_pred_deg']:.2f} "
                  f"| p1={out['p1_MPa']:.3f} p2={out['p2_MPa']:.3f} (cmd=({out['mpa_cmd_x']:.6f},{out['mpa_cmd_y']:.6f})) "
                  f"| z={out['z']:.4f} | ok={out['ok']}")

    # CSV 保存
    with open(args.csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t","theta_ref_deg","theta_pred_deg","p1_MPa","p2_MPa","ps_MPa","pd_MPa","z","ok","mpa_cmd_x","mpa_cmd_y"])
        for r in rows:
            w.writerow([r["t"], r["theta_ref_deg"], r["theta_pred_deg"], r["p1_MPa"], r["p2_MPa"],
                        r["ps_MPa"], r["pd_MPa"], r["z"], int(r["ok"]), r["mpa_cmd_x"], r["mpa_cmd_y"]])
    print(f"[Sim] wrote CSV -> {args.csv}")

    # main() の最後の if args.plot: ブロックを以下に置き換え
    if args.save_png or args.plot_show:
        try:
            # ウィンドウを出さない場合はAggバックエンドに切替
            if args.save_png and not args.plot_show:
                matplotlib.use("Agg")
            import matplotlib.pyplot as plt

            th_ref = [r["theta_ref_deg"] for r in rows]
            th_prd = [r["theta_pred_deg"] for r in rows]
            p1 = [r["p1_MPa"] for r in rows]
            p2 = [r["p2_MPa"] for r in rows]

            plt.figure()
            plt.plot(t, th_ref, label="theta_ref [deg]")
            plt.plot(t, th_prd, label="theta_pred [deg]", linestyle="--")
            # 圧力の簡易可視化（同じ軸に薄く載せるならこれでOK）
            plt.plot(t, p1, label="p1 [MPa]")
            plt.plot(t, p2, label="p2 [MPa]")

            plt.xlabel("time [s]"); plt.ylabel("angle / pressure")
            plt.legend(); plt.title("NARX-SQP simulation (angle + pressure)")

            if args.save_png:
                plt.savefig(args.save_png, bbox_inches="tight", dpi=200)
                print(f"[Sim] saved plot -> {args.save_png}")
                plt.close()
            if args.plot_show:
                plt.show()
        except Exception as e:
            print(f"[Sim] plot skipped: {e}")

if __name__ == "__main__":
    main()
