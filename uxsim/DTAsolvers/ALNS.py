"""
Mixed-Control ALNS with Periodic Injection & Convergence Check
==============================================================

概要
----
- デフォルトの小近傍（kflip / swap / shuffle）で自動探索（auto）
- n ステップごとにユーザー定義の候補を**注入**（external）
- 収束判定（停滞 + 低受理率 + 低温度）で早期停止（オプション）
- Initially coded by GPT-5, refined by human and Claude Code. Unfortunately, this code is unnecessarily redundant and not in my style, but I confirm it works.

主API
-----
init_alns(x, obj, **params) -> ALNSState
    状態を初期化（温度・適応重み・ログなど）。
auto_step(state) -> StepLog
    自動近傍で 1 ステップ。
run_auto(state, n, time_limit_s=None) -> list[StepLog]
    自動近傍で n ステップ連続実行。
external_step(state, x_cand, move='external', changed_idx=None,
              accept_policy='metropolis'|'greedy'|'always') -> (accepted, StepLog)
    外部候補 1 つを注入し、受理判定・状態更新・ロギング。
run_auto_until_converged(state, *, max_steps=10000, batch=50, time_limit_s=None,
                         check_every=100, check_fn=default_convergence_check,
                         inject_every=None, inject_fn=None, stop_on_converge=True)
    自動探索を収束まで回すランナー。**inject_every ステップごとに inject_fn を呼び**、
    返された候補を external_step で評価・適用する。

その他ユーティリティ
--------------------
should_shake(state) -> bool
    停滞（patience 超過）したか。
make_shake_candidate(state, base='best'|'current', k=None) -> (x, idx)
    強摂動候補（参考実装）。
do_shake(state, base='best', k=None) -> StepLog
    強摂動を適用してログ追加（“再起動”。デフォルトでは自動では呼ばない）。
get_current_x(state) / get_best_x(state)
    現在解 / 最良解のコピー取得。
evaluate_candidates(state, xs) -> list[float]
    複数候補のユーザー向き（sense反映）評価。
default_convergence_check(state, **opts) -> (bool, reason)
    収束判定（停滞＋低受理率＋低温度の合成ルール）。
finalize(state, stop_reason='') -> (x_star, RunLog)
    最良解とログを返す。

"""

from __future__ import annotations
from dataclasses import dataclass, field, asdict
from typing import Callable, Sequence, List, Optional, Dict, Any, Tuple, Iterable, Union
import math
import random
import time

import numpy as np


# -------------------------
# ログ構造体
# -------------------------
@dataclass
class StepLog:
    """
    1 ステップ（自動/外部/Shake）のログ。

    Attributes
    ----------
    iter : int
        ステップ番号（1 始まり）。
    move : str
        適用した近傍または注入ラベル（例："kflip", "swap", "external", "shake(k)"}）。
    changed_idx : list of int or None
        変更したインデックス（分からなければ None/[] 可）。
    delta_obj : float
        内部向き最小化での差分（cand - cur）。
    accepted : bool
        候補が採択されたか。
    obj_val : float
        ステップ後の現解の目的値（ユーザー向き：sense を反映）。
    best_obj : float
        ステップ後の最良目的値（ユーザー向き）。
    temperature : float
        当該ステップの温度。
    source : {"auto","external","shake"}
        ステップ種別。
    """
    iter: int
    move: str
    changed_idx: Optional[List[int]]
    changed_idxs: Optional[int]
    delta_obj: float
    accepted: bool
    obj_val: float
    best_obj: float
    temperature: float
    source: str


@dataclass
class RunLog:
    """
    実行全体のログとメタ情報。

    Attributes
    ----------
    steps : list of StepLog
        ステップログ。
    best_x : sequence
        最良解（ユーザー向き）。
    best_obj : float
        最良目的値（ユーザー向き）。
    seed : int or None
        乱数シード。
    stop_reason : str
        停止理由。
    operator_stats : dict
        自動近傍の統計（weight, uses, best_improves）。
    params : dict
        実行パラメータのスナップショット。
    """
    steps: List[StepLog] = field(default_factory=list)
    best_x: Sequence[Any] = field(default_factory=list)
    best_obj: float = float("inf")
    seed: Optional[int] = None
    stop_reason: str = ""
    operator_stats: Dict[str, Dict[str, float]] = field(default_factory=dict)
    params: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        """辞書化。"""
        return asdict(self)

    def to_dataframe(self):
        """
        ステップログを pandas.DataFrame で返す（pandas 無しなら list[dict]）。
        """
        try:
            import pandas as pd
            return pd.DataFrame([s.__dict__ for s in self.steps])
        except Exception:
            return [s.__dict__ for s in self.steps]


# -------------------------
# 内部ユーティリティ
# -------------------------
def _copy_vec(x: Sequence[Any]) -> List[Any]:
    if isinstance(x, np.ndarray):
        return x.copy().tolist()
    return list(x)

def _rng_choice(rng: random.Random, seq: Sequence[Any]) -> Any:
    return seq[rng.randrange(len(seq))]

def _infer_domains_from_x(x: Sequence[Any]) -> List[List[Any]]:
    uniq = sorted(set(x), key=lambda v: (str(type(v)), str(v)))
    return [uniq[:] for _ in range(len(x))]


# -------------------------
# 状態
# -------------------------
@dataclass
class ALNSState:
    """
    ALNS の進行状態を保持する。

    Parameters（主なフィールド）
    ----------------------------
    sense : {"min","max"}
        最小化/最大化（内部では最小化に統一）。
    obj : callable
        ユーザーの目的関数 `obj(x)->float`。
    domains : list of sequence
        各位置の許容値候補。
    move_set : list of {"kflip","swap","shuffle"}
        自動ステップで使用する近傍の集合。
    k_max : int
        kflip で一度に変更する最大位置数。
    T0, Tend : float
        温度スケジュールの初期・終端。
    total_iters : int
        温度スケジュールの地平（分母）。後から変更可。
    adapt_eta : float
        近傍重みの EMA 係数。
    patience : int
        ベスト更新がこれだけ無いと停滞扱い。
    shake_k : int
        強摂動の変更数。
    auto_shake : bool
        停滞時に自動で do_shake() を入れるか（デフォルト False）。
    additional_info : dict
        UXsimから渡される追加情報
        "departure_times": 0-1で正規化された車両ごとの出発時刻


    状態フィールド
    --------------
    x_cur, x_best : list
        現在解 / 最良解。
    cur, best : float
        内部向き（最小化）の目的値。
    it, last_best_it : int
        総ステップ数 / 直近ベスト更新ステップ。
    op_* : dict
        自動近傍の適応重みや統計。
    steps : list of StepLog
        ログ。
    """
    # パラメータ
    sense: str
    obj: Callable[[Sequence[Any]], float]
    domains: List[Sequence[Any]]
    move_set: List[str]
    k_max: int
    T0: float
    Tend: float
    total_iters: int
    adapt_eta: float
    reward_improve_best: float
    reward_improve: float
    reward_accept_worse: float
    patience: int
    shake_k: int
    auto_shake: bool
    rng: random.Random
    seed: Optional[int]
    params: Dict[str, Any]
    additional_info: Any

    # 状態
    x_cur: List[Any]
    x_best: List[Any]
    cur: float
    best: float
    it: int = 0
    last_best_it: int = 0

    # 適応
    op_weights: Dict[str, float] = field(default_factory=dict)
    op_scores: Dict[str, float] = field(default_factory=dict)
    op_uses: Dict[str, int] = field(default_factory=dict)
    op_best_improves: Dict[str, int] = field(default_factory=dict)

    # ログ
    steps: List[StepLog] = field(default_factory=list)


# -------------------------
# 初期化
# -------------------------
def init_alns(
    x: Sequence[Any],
    obj: Callable[[Sequence[Any]], float],
    *,
    sense: str = "min",
    domains: Optional[List[Sequence[Any]]] = None,
    move_set: Optional[List[str]] = None,  # ["kflip","swap","shuffle"]
    k_max: int = 5,
    T0: Optional[float] = None,
    Tend: float = 1e-3,
    total_iters: int = 10000,
    adapt_eta: float = 0.2,
    reward_improve_best: float = 5.0,
    reward_improve: float = 2.0,
    reward_accept_worse: float = 0.5,
    patience: int = 1000,
    shake_k: Optional[int] = None,
    auto_shake: bool = False,  # ← デフォルトOFF
    seed: Optional[int] = 42,
    additional_info: Any,
) -> ALNSState:
    """
    ALNS 状態を初期化する。

    See Also
    --------
    run_auto, run_auto_until_converged
    """
    if sense not in ("min", "max"):
        raise ValueError("sense must be 'min' or 'max'")

    rng = random.Random(seed)
    if domains is None:
        domains = _infer_domains_from_x(x)
    n = len(x)
    if any(len(d) == 0 for d in domains):
        raise ValueError("domains に空の位置があります。")

    if move_set is None:
        move_set = ["kflip", "swap", "shuffle"]
    for m in move_set:
        if m not in ("kflip", "swap", "shuffle"):
            raise ValueError(f"未知の move: {m}")

    if shake_k is None:
        shake_k = max(1, n // 3)

    x_cur = _copy_vec(x)

    def _eval(xx: Sequence[Any]) -> float:
        v = obj(xx)
        return -v if sense == "max" else v

    cur = _eval(x_cur)
    best = cur
    x_best = _copy_vec(x_cur)

    # T0 自動推定
    if T0 is None:
        diffs = []
        for _ in range(min(10, 100)):
            xx = _copy_vec(x_cur)
            k = rng.randint(1, min(k_max, n))
            idx = rng.sample(range(n), k)
            for j in idx:
                choices = [c for c in domains[j] if c != xx[j]]
                if choices:
                    xx[j] = _rng_choice(rng, choices)
            diffs.append(max(0.0, _eval(xx) - cur))
        scale = (sorted(diffs)[len(diffs)//2] if diffs else 1.0) or 1.0
        T0 = max(1e-6, 3.0 * scale)

    op_weights = {m: 1.0 for m in move_set}
    op_scores = {m: 0.0 for m in move_set}
    op_uses = {m: 0 for m in move_set}
    op_best_improves = {m: 0 for m in move_set}

    additional_info = additional_info

    params = dict(
        sense=sense, move_set=move_set, k_max=k_max, T0=T0, Tend=Tend,
        total_iters=total_iters, adapt_eta=adapt_eta,
        reward_improve_best=reward_improve_best, reward_improve=reward_improve,
        reward_accept_worse=reward_accept_worse, patience=patience,
        shake_k=shake_k, auto_shake=auto_shake, seed=seed
    )

    return ALNSState(
        sense=sense, obj=obj, domains=domains, move_set=move_set,
        k_max=k_max, T0=T0, Tend=Tend, total_iters=total_iters,
        adapt_eta=adapt_eta, reward_improve_best=reward_improve_best,
        reward_improve=reward_improve, reward_accept_worse=reward_accept_worse,
        patience=patience, shake_k=shake_k, auto_shake=auto_shake,
        rng=rng, seed=seed, params=params,
        x_cur=x_cur, x_best=x_best, cur=cur, best=best,
        it=0, last_best_it=0,
        op_weights=op_weights, op_scores=op_scores,
        op_uses=op_uses, op_best_improves=op_best_improves,
        steps=[],
        additional_info=additional_info
    )


# -------------------------
# 温度・自動近傍
# -------------------------
def _temperature(state: ALNSState, it_next: Optional[int] = None) -> float:
    it = state.it if it_next is None else it_next
    it = max(0, min(it, max(1, state.total_iters)))
    return state.T0 * ((state.Tend / state.T0) ** (it / max(1, state.total_iters)))

def _apply_move_auto(state: ALNSState, move: str, base: Sequence[Any]) -> Tuple[List[int], List[Any]]:
    n = len(base)
    xx = _copy_vec(base)
    rng = state.rng

    #時間の矢を考えて，最初は出発時刻が早いほどkflip/swapしやすい
    #TODO: shuffleはこれを全く考慮しない．うまいヒューリスティクスがあれば使う
    weights = 1-state.additional_info["departure_times"]*((state.total_iters-state.it)/state.total_iters)
    weights = np.maximum(1/len(weights)/100, weights)

    if move == "kflip":
        k = rng.randint(1, min(state.k_max, n))
        idx = random.choices(range(n), weights=weights/sum(weights), k=k)
        for j in idx:
            choices = [c for c in state.domains[j] if c != xx[j]]
            if choices:
                xx[j] = _rng_choice(rng, choices)
        return idx, xx

    elif move == "swap":
        if n < 2:
            return [], xx
        
        i = random.choices(range(n), weights=weights/sum(weights), k=1)[0]
        j = random.choices(range(n), weights=weights/sum(weights), k=1)[0]

        xx[i], xx[j] = xx[j], xx[i]
        return [i, j], xx

    elif move == "shuffle":
        if n < 2:
            return [], xx
        i, j = sorted(rng.sample(range(n), 2))
        seg = xx[i:j+1]
        rng.shuffle(seg)
        xx[i:j+1] = seg
        return list(range(i, j+1)), xx

    else:
        raise ValueError(f"未知の move: {move}")

def _pick_operator(state: ALNSState) -> str:
    total = sum(state.op_weights[m] for m in state.move_set)
    r = state.rng.random() * total
    s = 0.0
    for m in state.move_set:
        s += state.op_weights[m]
        if r <= s:
            return m
    return state.move_set[-1]

def _eval_internal(state: ALNSState, x: Sequence[Any]) -> float:
    v = state.obj(x)
    return -v if state.sense == "max" else v

def _maybe_update_weights(state: ALNSState, op: str, *, improved_best: bool, accepted: bool, delta: float):
    if op not in state.op_weights:
        return
    if improved_best:
        state.op_scores[op] += state.reward_improve_best
    elif accepted and delta < 0:
        state.op_scores[op] += state.reward_improve
    elif accepted and delta > 0:
        state.op_scores[op] += state.reward_accept_worse

    w_old = state.op_weights[op]
    state.op_weights[op] = (1 - state.adapt_eta) * w_old + state.adapt_eta * max(1e-6, state.op_scores[op])
    state.op_scores[op] *= 0.5

def auto_step(state: ALNSState) -> StepLog:
    """
    デフォルト近傍で 1 ステップ進める。
    """
    op = _pick_operator(state)
    state.op_uses[op] += 1

    changed_idx, x_cand = _apply_move_auto(state, op, state.x_cur)
    cand = _eval_internal(state, x_cand)
    delta = cand - state.cur
    T = _temperature(state, it_next=state.it + 1)

    accept = (delta <= 0) or (random.random() < math.exp(-delta / max(T, 1e-12)))
    if accept:
        state.x_cur, state.cur = x_cand, cand

    improved_best = False
    if state.cur < state.best:
        state.best = state.cur
        state.x_best = _copy_vec(state.x_cur)
        improved_best = True
        state.last_best_it = state.it + 1
        state.op_best_improves[op] += 1

    _maybe_update_weights(state, op, improved_best=improved_best, accepted=accept, delta=delta)

    state.it += 1
    log = StepLog(
        iter=state.it, move=op, changed_idx=changed_idx, changed_idxs=len(changed_idx), delta_obj=delta, accepted=accept,
        obj_val=(state.cur if state.sense == "min" else -state.cur),
        best_obj=(state.best if state.sense == "min" else -state.best),
        temperature=T, source="auto"
    )
    state.steps.append(log)

    if state.auto_shake and should_shake(state):
        do_shake(state, base="best", k=state.shake_k)

    return log

def run_auto(state: ALNSState, n: int = 100, time_limit_s: Optional[float] = None) -> List[StepLog]:
    """
    自動近傍で連続 `n` ステップ実行。
    """
    start_it = state.it
    t0 = time.time()
    while state.it - start_it < n:
        if time_limit_s is not None and (time.time() - t0) >= time_limit_s:
            break
        auto_step(state)
    return state.steps[start_it:state.it]


# -------------------------
# 外部注入
# -------------------------
def external_step(
    state: ALNSState,
    x_cand: Sequence[Any],
    *,
    move: str = "external",
    changed_idx: Optional[List[int]] = None,
    accept_policy: str = "metropolis",  # "metropolis" | "greedy" | "always"
) -> Tuple[bool, StepLog]:
    """
    外部で作成した候補解を 1 ステップとして注入。

    Parameters
    ----------
    x_cand : sequence
        候補解。
    move : str, optional
        ログ用ラベル。
    changed_idx : list of int, optional
        変更インデックス。Noneの場合は自動計算される。
    accept_policy : {'metropolis','greedy','always'}, optional
        受理方針。

    Returns
    -------
    accepted : bool
        受理されたか。
    log : StepLog
        追加ログ。
    """
    # changed_idxが指定されていない場合、自動的に計算
    if changed_idx is None:
        changed_idx = []
        for i in range(len(x_cand)):
            if i >= len(state.x_cur) or state.x_cur[i] != x_cand[i]:
                changed_idx.append(i)
    
    cand = _eval_internal(state, x_cand)
    delta = cand - state.cur
    T = _temperature(state, it_next=state.it + 1)

    if accept_policy == "metropolis":
        accepted = (delta <= 0) or (random.random() < math.exp(-delta / max(T, 1e-12)))
    elif accept_policy == "greedy":
        accepted = (delta <= 0)
    elif accept_policy == "always":
        accepted = True
    else:
        raise ValueError("accept_policy must be 'metropolis', 'greedy', or 'always'")

    if accepted:
        state.x_cur = _copy_vec(x_cand)
        state.cur = cand

    improved_best = False
    if state.cur < state.best:
        state.best = state.cur
        state.x_best = _copy_vec(state.x_cur)
        improved_best = True
        state.last_best_it = state.it + 1

    state.it += 1
    log = StepLog(
        iter=state.it, move=move, changed_idx=changed_idx, changed_idxs=len(changed_idx), delta_obj=delta,
        accepted=accepted,
        obj_val=(state.cur if state.sense == "min" else -state.cur),
        best_obj=(state.best if state.sense == "min" else -state.best),
        temperature=T, source="external"
    )
    state.steps.append(log)

    if state.auto_shake and should_shake(state):
        do_shake(state, base="best", k=state.shake_k)

    return accepted, log


# -------------------------
# Shake（任意）
# -------------------------
def should_shake(state: ALNSState) -> bool:
    """停滞（patience 超過）したか。"""
    return (state.it - state.last_best_it) >= state.patience

def make_shake_candidate(state: ALNSState, *, base: str = "best", k: Optional[int] = None) -> Tuple[List[Any], List[int]]:
    """強摂動候補を作る（参考）。"""
    if k is None:
        k = state.shake_k
    xx = _copy_vec(state.x_best if base == "best" else state.x_cur)
    n = len(xx)
    idx = state.rng.sample(range(n), min(k, n))
    for j in idx:
        choices = [c for c in state.domains[j] if c != xx[j]]
        if choices:
            xx[j] = _rng_choice(state.rng, choices)
    return xx, idx

def do_shake(state: ALNSState, *, base: str = "best", k: Optional[int] = None) -> StepLog:
    """強摂動を適用してログ追加（再起動）。"""
    x_cand, idx = make_shake_candidate(state, base=base, k=k)
    def _eval(xx): 
        v = state.obj(xx)
        return -v if state.sense == "max" else v
    T = _temperature(state, it_next=state.it + 1)
    state.x_cur = _copy_vec(x_cand)
    state.cur = _eval(state.x_cur)
    state.it += 1
    log = StepLog(
        iter=state.it, move=f"shake({len(idx)})", changed_idx=idx, changed_idxs=len(idx), delta_obj=0.0,
        accepted=True,
        obj_val=(state.cur if state.sense == "min" else -state.cur),
        best_obj=(state.best if state.sense == "min" else -state.best),
        temperature=T, source="shake",
    )
    state.steps.append(log)
    return log


# -------------------------
# 収束判定 & ランナー
# -------------------------
def default_convergence_check(
    state: ALNSState,
    *,
    window: int = 500,
    stall_iters: Optional[int] = None,
    min_accept_rate: float = 0.005,
    min_best_improve: float = 1e-8,
    temp_threshold: Optional[float] = None,
    sources: Tuple[str, ...] = ("auto",),
) -> Tuple[bool, str]:
    """
    “停滞＋低受理率＋低温度”の合成ルールで収束を判定。

    Returns
    -------
    converged : bool
    reason : str
    """
    if stall_iters is None:
        stall_iters = state.patience
    if temp_threshold is None:
        temp_threshold = max(state.Tend * 1.05, state.T0 * 1e-4)

    steps = state.steps
    if len(steps) < window:
        return (False, "warmup")

    window_steps = [s for s in steps if s.source in sources][-window:]
    if not window_steps:
        return (False, "no-steps-in-window")

    accept_rate = sum(1 for s in window_steps if s.accepted) / len(window_steps)
    best_start = window_steps[0].best_obj
    best_end = window_steps[-1].best_obj
    best_improve = abs(best_start - best_end)
    no_improve_long = (state.it - state.last_best_it) >= stall_iters
    T = _temperature(state)

    if (no_improve_long and accept_rate <= min_accept_rate and best_improve <= min_best_improve):
        return True, f"stalled:{stall_iters}, low-accept:{accept_rate:.4f}, tiny-improve:{best_improve:.3g}"
    if T <= temp_threshold and best_improve <= min_best_improve:
        return True, f"frozen:T={T:.3g}<=thr, tiny-improve:{best_improve:.3g}"
    return False, "not-converged"


def run_auto_until_converged(
    state: ALNSState,
    *,
    max_steps: int = 10_000,
    batch: int = 50,
    time_limit_s: Optional[float] = None,
    check_every: int = 100,
    check_fn: Callable[..., Tuple[bool, str]] = default_convergence_check,
    stop_on_converge: bool = True,
    # ↓↓↓ 追加：n ステップごとにユーザー注入（例：n=200）
    inject_every: Optional[int] = None,
    inject_fn: Optional[
        Callable[[ALNSState],
                 Optional[Union[
                     Sequence[Any],
                     Tuple[Sequence[Any], str],
                     Tuple[Sequence[Any], str, Optional[List[int]]],
                     Tuple[Sequence[Any], str, Optional[List[int]], str],
                 ]]]
    ] = None,
    inject_accept_policy: str = "metropolis",
) -> Tuple[bool, str, int]:
    """
    自動探索を “収束まで” 実行するランナー。
    **inject_every** ステップごとに **inject_fn(state)** を呼び、
    返された候補を external_step で評価します。

    Parameters
    ----------
    inject_every : int or None, optional
        何ステップごとに注入を試みるか（例：200）。
        None の場合、注入は行わない。
    inject_fn : callable or None, optional
        `inject_fn(state)` は以下のいずれかを返せます：
            - x_cand
            - (x_cand, move)
            - (x_cand, move, accept_policy)
        None を返した場合は「今回注入なし」と解釈。
        changed_idxは自動的に計算される。
    inject_accept_policy : {'metropolis','greedy','always'}, optional
        inject_fn が accept_policy を返さなかった場合のデフォルト。

    Returns
    -------
    converged : bool
    reason : str
    steps_done : int
    """
    start_it = state.it
    t0 = time.time()
    next_inject_at = state.it + (inject_every or 0)

    def _maybe_inject():
        nonlocal next_inject_at
        if inject_every is None or inject_fn is None:
            return
        if state.it < next_inject_at:
            return
        payload = inject_fn(state)
        if payload is None:
            # スキップ（次回はさらに inject_every 後）
            next_inject_at = state.it + inject_every
            return
        # payload の解釈
        if isinstance(payload, tuple):
            if len(payload) == 1:
                x_cand = payload[0]
                move = "external"
                policy = inject_accept_policy
            elif len(payload) == 2:
                x_cand, move = payload
                policy = inject_accept_policy
            else:
                # 3つ以上の要素がある場合、3番目はaccept_policy
                x_cand, move, policy = payload[:3]
        else:
            x_cand = payload
            move = "external"
            policy = inject_accept_policy

        # changed_idxは常にNoneを渡して自動計算させる
        external_step(state, x_cand, move=move, changed_idx=None, accept_policy=policy)
        next_inject_at = state.it + inject_every

    while state.it - start_it < max_steps:
        if time_limit_s is not None and (time.time() - t0) >= time_limit_s:
            return False, "time-limit", state.it - start_it

        # 自動を小さく刻む：注入タイミングを細かく拾うため
        todo = min(batch, max_steps - (state.it - start_it))
        run_auto(state, n=todo)

        # 必要なら注入
        _maybe_inject()

        # 収束チェック
        if (state.it - start_it) % check_every == 0:
            converged, reason = check_fn(state)
            if converged:
                state.params["converged"] = True
                state.params["converged_reason"] = reason
                if stop_on_converge:
                    return True, reason, state.it - start_it

    return False, "step-budget", state.it - start_it


# -------------------------
# 取得・評価・完了
# -------------------------
def get_current_x(state: ALNSState) -> List[Any]:
    """現在解のコピーを返す。"""
    return _copy_vec(state.x_cur)

def get_best_x(state: ALNSState) -> List[Any]:
    """最良解のコピーを返す。"""
    return _copy_vec(state.x_best)

def evaluate_candidates(state: ALNSState, xs: Iterable[Sequence[Any]]) -> List[float]:
    """複数候補をユーザー向きの値（sense を反映）で返す。"""
    vals = []
    for x in xs:
        v = state.obj(x)
        vals.append(v if state.sense == "min" else -v)
    return vals

def finalize(state: ALNSState, stop_reason: str = "") -> Tuple[Sequence[Any], RunLog]:
    """実行を締め、最良解とログを返す。"""
    run = RunLog(
        steps=list(state.steps),
        best_x=_copy_vec(state.x_best),
        best_obj=(state.best if state.sense == "min" else -state.best),
        seed=state.seed,
        stop_reason=stop_reason,
        operator_stats={
            m: dict(weight=state.op_weights[m], uses=state.op_uses[m], best_improves=state.op_best_improves[m])
            for m in state.move_set
        },
        params=dict(state.params),
    )
    return _copy_vec(state.x_best), run


# -------------------------
# デモ（__main__）
# -------------------------
if __name__ == "__main__":
    # 例：各位置 ∈ {0,1,2,3}、重み付き合計を target に合わせる
    w = [1.12,3.13,2.141,7.11,4.111,6.1111,5.11111,9.111111]
    target = 70
    x0 = [0,0,0,0,0,0,0,0]
    domains = [[0,1,2,3] for _ in x0]

    def obj_example(x1):
        return abs(sum(wj*xj for wj, xj in zip(w, x1)) - target)

    # 初期化（自動shakeはOFF）
    state = init_alns(
        x0, obj_example,
        domains=domains,
        move_set=["kflip","swap","shuffle"],
        k_max=3,
        total_iters=1000,
        auto_shake=True,   # 収束後に壊さない
        seed=0,
    )

    # ---- ユーザー注入関数：inject_every ステップごとに呼ばれる ----
    def inject_fn_example(st):
        """
        重い位置（wが大）のインデックスを重点的に微摂動する簡易destroy+repair。
        - 返り値は (x_cand, move_label, changed_idx, accept_policy) の省略可タプル。
        """
        cur = get_current_x(st)
        top_idx = sorted(range(len(w)), key=lambda i: w[i], reverse=True)[:3]
        cand = _copy_vec(cur)
        rng = random.Random(123 + st.it)
        for i in top_idx:
            choices = [v for v in domains[i] if v != cand[i]]
            if not choices:
                continue
            cand[i] = rng.choice(choices)
        # "greedy" にすると悪化は採用しない＝非破壊注入にもできる
        return (cand, "custom-heavy-weights", "metropolis")

    # 収束まで回す。inject_every ステップごとに inject_fn_example を呼ぶ
    converged, reason, done = run_auto_until_converged(
        state,
        max_steps=2000,
        batch=50,
        check_every=100,
        inject_every=200,          # ← 指定：n=200
        inject_fn=inject_fn_example,
        stop_on_converge=True,
    )

    x_star, run = finalize(state, stop_reason="converged" if converged else "budget")

    try:
        df = run.to_dataframe()
        df.drop(columns=["changed_idx"])
        print(df.to_string())
    except Exception:
        pass

    print("converged:", converged, "reason:", reason, "steps:", done)
    print("best obj:", run.best_obj)
    print("best x  :", x_star)
    print("op stats:", run.operator_stats)