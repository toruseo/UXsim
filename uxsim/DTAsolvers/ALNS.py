"""
Adaptive Large Neighborhood Search (ALNS) for DSO

- Initially coded by GPT-5, refined by human and Claude Code. 
- Customizable minimum implementation
- Includes domain-knowledge-based heuristics
"""

from __future__ import annotations
from dataclasses import dataclass, field, asdict
from typing import Callable, Sequence, List, Optional, Dict, Any, Tuple, Iterable
import math
import random
import warnings
import numpy as np
from pprint import pprint
import time

from ..Utilities import estimate_congestion_externality_route

# -------------------------
# ログ構造体
# -------------------------
@dataclass
class StepLog:
    """ALNSステップのログ情報.
    
    Attributes
    ----------
    iter : int
        イテレーション番号
    move : str
        実行されたムーブ ("D:<destroy>+R:<repair>")
    changed_idx : Optional[List[int]]
        変更されたインデックスのリスト
    changed_idxs : Optional[int]
        変更されたインデックスの数
    delta_obj : float
        内部最小化差分 (cand - cur)
    accepted : bool
        ムーブが受理されたかどうか
    obj_val : float
        ユーザー向き目的関数値（sense反映）
    best_obj : float
        ユーザー向き最良目的関数値
    temperature : float
        現在の温度
    source : str
        ステップのソース ("auto")
    """
    iter: int
    move: str                          # "D:<destroy>+R:<repair>"
    changed_idx: Optional[List[int]]
    changed_idxs: Optional[int]
    delta_obj: float                   # 内部最小化差分 cand - cur
    accepted: bool
    obj_val: float                     # ユーザー向き（sense 反映）
    best_obj: float                    # ユーザー向き
    temperature: float
    source: str                        # "auto"

@dataclass
class RunLog:
    """ALNS実行全体のログ情報.
    
    Attributes
    ----------
    steps : List[StepLog]
        各ステップのログリスト
    best_x : Sequence[Any]
        最良解
    best_obj : float
        最良目的関数値
    seed : Optional[int]
        使用された乱数シード
    stop_reason : str
        停止理由
    operator_stats : Dict[str, Any]
        オペレータの統計情報 {"destroy":..., "repair":...}
    params : Dict[str, Any]
        使用されたパラメータ
    
    Methods
    -------
    to_dict()
        辞書形式に変換
    to_dataframe()
        pandasデータフレームに変換（pandasが必要）
    """
    steps: List[StepLog] = field(default_factory=list)
    best_x: Sequence[Any] = field(default_factory=list)
    best_obj: float = float("inf")
    seed: Optional[int] = None
    stop_reason: str = ""
    operator_stats: Dict[str, Any] = field(default_factory=dict)   # {"destroy":..., "repair":...}
    params: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    def to_dataframe(self):
        try:
            import pandas as pd
            return pd.DataFrame([s.__dict__ for s in self.steps])
        except Exception:
            return [s.__dict__ for s in self.steps]

# -------------------------
# 内部ユーティリティ
# -------------------------
def _copy_vec(x: Sequence[Any]) -> List[Any]:
    """ベクトルをコピー.
    
    numpy配列の場合はリストに変換してコピー。
    
    Parameters
    ----------
    x : Sequence[Any]
        コピー元のベクトル
    
    Returns
    -------
    List[Any]
        コピーされたリスト
    """
    if isinstance(x, np.ndarray):
        return x.copy().tolist()
    return list(x)

def _rng_choice(rng: random.Random, seq: Sequence[Any]) -> Any:
    """乱数生成器を使ってシーケンスから要素をランダム選択.
    
    Parameters
    ----------
    rng : random.Random
        乱数生成器
    seq : Sequence[Any]
        選択元のシーケンス
    
    Returns
    -------
    Any
        選択された要素
    """
    return seq[rng.randrange(len(seq))]

def _infer_domains_from_x(x: Sequence[Any]) -> List[List[Any]]:
    """解ベクトルから定義域を自動推定.
    
    各位置の値として、xに含まれる全てのユニーク値を
    使用可能と仮定。
    
    Parameters
    ----------
    x : Sequence[Any]
        推定元の解ベクトル
    
    Returns
    -------
    List[List[Any]]
        各位置の定義域
    """
    uniq = sorted(set(x), key=lambda v: (str(type(v)), str(v)))
    return [uniq[:] for _ in range(len(x))]

# -------------------------
# 状態
# -------------------------
@dataclass
class ALNSState:
    """ALNS実行状態.
    
    アルゴリズムの全状態を保持するデータクラス。
    
    Attributes
    ----------
    sense : str
        最適化の方向 ("min" または "max")
    obj : Callable[[Sequence[Any]], float]
        目的関数
    domains : List[Sequence[Any]]
        各変数の定義域
    destroy_set : List[str]
        使用する破壊オペレータの名前リスト
    repair_set : List[str]
        使用する修復オペレータの名前リスト
    k_min : int
        破壊規模の最小値
    k_max : int
        破壊規模の最大値
    k_cur : int
        破壊規模の現在値
    T0 : float
        初期温度
    Tend : float
        終了温度
    total_iters : int
        総イテレーション数
    segment_len : int
        適応更新のセグメント長
    adapt_eta : float
        適応更新の学習率
    reward_improve_best : float
        最良解改善時の報酬
    reward_improve : float
        解改善時の報酬
    reward_accept_worse : float
        悪化解受理時の報酬
    rng : random.Random
        乱数生成器
    seed : Optional[int]
        乱数シード
    params : Dict[str, Any]
        パラメータ辞書
    additional_info : Dict[str, Any]
        追加情報
    x_cur : List[Any]
        現在解
    x_best : List[Any]
        最良解
    cur : float
        現在解の目的関数値（内部用）
    best : float
        最良解の目的関数値（内部用）
    it : int
        現在のイテレーション番号
    last_best_it : int
        最後に最良解が更新されたイテレーション番号
    d_weights : Dict[str, float]
        破壊オペレータの重み
    d_scores : Dict[str, float]
        破壊オペレータのスコア
    d_uses : Dict[str, int]
        破壊オペレータの使用回数
    d_best_improves : Dict[str, int]
        破壊オペレータによる最良解改善回数
    r_weights : Dict[str, float]
        修復オペレータの重み
    r_scores : Dict[str, float]
        修復オペレータのスコア
    r_uses : Dict[str, int]
        修復オペレータの使用回数
    r_best_improves : Dict[str, int]
        修復オペレータによる最良解改善回数
    seg_countdown : int
        次のセグメント更新までのカウントダウン
    steps : List[StepLog]
        ステップログのリスト
    """
    # パラメータ
    sense: str
    obj: Callable[[Sequence[Any]], float]
    domains: List[Sequence[Any]]

    # ---- 破壊/修復（可変オペレータ集合） ----
    destroy_set: List[str]
    repair_set: List[str]

    # 破壊規模
    k_min: int
    k_max: int
    k_cur: int

    # 焼きなまし
    T0: float
    Tend: float
    total_iters: int

    # 適応（セグメント更新）
    segment_len: int
    adapt_eta: float
    reward_improve_best: float
    reward_improve: float
    reward_accept_worse: float
    always_accept: bool


    rng: random.Random
    seed: Optional[int]
    params: Dict[str, Any]
    additional_info: Dict[str, Any]

    # 状態
    x_cur: List[Any]
    x_best: List[Any]
    cur: float
    best: float
    it: int = 0
    last_best_it: int = 0

    # 破壊/修復の適応重み
    d_weights: Dict[str, float] = field(default_factory=dict)
    d_scores: Dict[str, float] = field(default_factory=dict)
    d_uses: Dict[str, int] = field(default_factory=dict)
    d_best_improves: Dict[str, int] = field(default_factory=dict)

    r_weights: Dict[str, float] = field(default_factory=dict)
    r_scores: Dict[str, float] = field(default_factory=dict)
    r_uses: Dict[str, int] = field(default_factory=dict)
    r_best_improves: Dict[str, int] = field(default_factory=dict)

    seg_countdown: int = 0

    # ログ
    steps: List[StepLog] = field(default_factory=list)

# -------------------------
# 破壊オペレータ実装
#   シグネチャ: (state, base) -> (removed_idx: list[int], partial_x: list[Any] with None on removed)
# -------------------------

def destroy_random(state: ALNSState, base: Sequence[Any]) -> Tuple[List[int], List[Any]]:
    """ランダムにk個の要素を選んで破壊.
    
    Parameters
    ----------
    state : ALNSState
        ALNS状態
    base : Sequence[Any]
        基本となる解
    
    Returns
    -------
    Tuple[List[int], List[Any]]
        (削除されたインデックス, 部分解) のタプル
        部分解の削除位置はNoneに設定される
    """
    n = len(base)
    k = state.rng.randint(state.k_min, min(state.k_cur, n))
    removed = state.rng.sample(range(n), k)
    xx = _copy_vec(base)
    for i in removed:
        xx[i] = None
    return removed, xx

def destroy_segment(state: ALNSState, base: Sequence[Any]) -> Tuple[List[int], List[Any]]:
    """連続したセグメントを破壊.
    
    Parameters
    ----------
    state : ALNSState
        ALNS状態
    base : Sequence[Any]
        基本となる解
    
    Returns
    -------
    Tuple[List[int], List[Any]]
        (削除されたインデックス, 部分解) のタプル
        部分解の削除位置はNoneに設定される
    """
    n = len(base)
    if n < 2:
        return destroy_random(state, base)
    i = state.rng.randrange(n)
    j = state.rng.randrange(n)
    if i > j:
        i, j = j, i
    length = j - i + 1
    if length < state.k_min:
        j = min(n - 1, i + state.k_min - 1)
    if (j - i + 1) > state.k_cur:
        j = i + state.k_cur - 1
    removed = list(range(i, j + 1))
    xx = _copy_vec(base)
    for t in removed:
        xx[t] = None
    return removed, xx

def destroy_early_departure(state: ALNSState, base: Sequence[Any]) -> Tuple[List[int], List[Any]]:
    """出発時刻の早い車両を優先で重み付き確率でk個の要素を選んで破壊.
        
    Parameters
    ----------
    state : ALNSState
        ALNS状態
    base : Sequence[Any]
        基本となる解
    
    Returns
    -------
    Tuple[List[int], List[Any]]
        (削除されたインデックス, 部分解) のタプル
        部分解の削除位置はNoneに設定される
    """
    n = len(base)
    
    #出発時刻の早いものを選択しやすい
    w = (1 - state.additional_info.get("departure_times"))
    w = np.maximum(1/100000, w-0.3)
    
    k = state.rng.randint(state.k_min, min(state.k_cur, n))
    probs = (w / w.sum())
    removed = list(np.random.choice(np.arange(n), size=k, replace=False, p=probs))
    xx = _copy_vec(base)
    for i in removed:
        xx[i] = None
    return removed, xx

def destroy_late_departure(state: ALNSState, base: Sequence[Any]) -> Tuple[List[int], List[Any]]:
    """出発時刻の遅い車両を優先で重み付き確率でk個の要素を選んで破壊.
        
    Parameters
    ----------
    state : ALNSState
        ALNS状態
    base : Sequence[Any]
        基本となる解
    
    Returns
    -------
    Tuple[List[int], List[Any]]
        (削除されたインデックス, 部分解) のタプル
        部分解の削除位置はNoneに設定される
    """
    n = len(base)
    
    #出発時刻の遅いものを選択しやすい
    w = state.additional_info.get("departure_times")
    w = np.maximum(1/100000, w-0.3)
    
    k = state.rng.randint(state.k_min, min(state.k_cur, n))
    probs = (w / w.sum())
    removed = list(np.random.choice(np.arange(n), size=k, replace=False, p=probs))
    xx = _copy_vec(base)
    for i in removed:
        xx[i] = None
    return removed, xx


def destroy_congested_link(state: ALNSState, base: Sequence[Any]) -> Tuple[List[int], List[Any]]:
    """混雑リンクを走行した車両を選んで破壊
        
    Parameters
    ----------
    state : ALNSState
        ALNS状態
    base : Sequence[Any]
        基本となる解
    
    Returns
    -------
    Tuple[List[int], List[Any]]
        (削除されたインデックス, 部分解) のタプル
        部分解の削除位置はNoneに設定される
    """
    n = len(base)

    removed = []

    #混雑リンクを選択
    W = state.additional_info.get("W")
    df_l = W.analyzer.link_to_pandas()
    
    delay_ranking_ratio = 0.05
    delay_ranking_min_n = 3

    most_delayed_links = list(df_l.sort_values(by='delay_ratio', ascending=False)[:max([int(len(df_l)*delay_ranking_ratio),delay_ranking_min_n])]["link"])
    
    k = state.rng.randint(state.k_min, min(state.k_cur, n))
    for _ in range(k):
        link_selected_name = np.random.choice(most_delayed_links)
        link_selected = W.get_link(link_selected_name)

        veh_selected = np.random.choice(list(link_selected.vehicles_enter_log.values()))
        removed.append(veh_selected.id)

    xx = _copy_vec(base)
    for i in removed:
        xx[i] = None
    return removed, xx


# 破壊オペレータ一覧
_DESTROY_IMPLS: Dict[str, Callable[[ALNSState, Sequence[Any]], Tuple[List[int], List[Any]]]] = {
    "random": destroy_random,
    "segment": destroy_segment,
    "early_dep": destroy_early_departure,
    "late_dep": destroy_late_departure,
    "congest_link": destroy_congested_link,
}

# 破壊オペレータの初期重み（選択確率に比例）
_DESTROY_INITIAL_WEIGHTS: Dict[str, float] = {
    "random": 1.0,
    "segment": 1.0,
    "early_dep": 5.0,
    "late_dep": 0.2,
    "congest_link": 1.0,
}

# -------------------------
# 修復オペレータ実装
#   シグネチャ: (state, base_cur, partial_x, removed_idx) -> (x_filled: list[Any], changed_idx: list[int])
# -------------------------
def _eval_internal(state: ALNSState, x: Sequence[Any]) -> float:
    """内部用の目的関数評価.
    
    最大化問題の場合は符号を反転して最小化として扱う。
    
    Parameters
    ----------
    state : ALNSState
        ALNS状態
    x : Sequence[Any]
        評価する解
    
    Returns
    -------
    float
        内部目的関数値（常に最小化）
    """
    v = state.obj(x)
    return -v if state.sense == "max" else v

def _repair_random(state: ALNSState, base_cur: Sequence[Any], partial_x: Sequence[Any], removed: List[int]) -> Tuple[List[Any], List[int]]:
    """ランダムに値を選んで修復.
    
    Parameters
    ----------
    state : ALNSState
        ALNS状態
    base_cur : Sequence[Any]
        現在の基本解
    partial_x : Sequence[Any]
        部分解（削除位置はNone）
    removed : List[int]
        削除されたインデックス
    
    Returns
    -------
    Tuple[List[Any], List[int]]
        (修復された解, 変更されたインデックス) のタプル
    """
    x = _copy_vec(partial_x)
    rng = state.rng
    for i in removed:
        cur_val = base_cur[i]
        choices = [c for c in state.domains[i]]
        if len(choices) > 1:
            choices = [c for c in choices if c != cur_val] or choices
        x[i] = _rng_choice(rng, choices)
    return x, removed[:]

def _repair_minimum_cost(state: ALNSState, base_cur: Sequence[Any], partial_x: Sequence[Any], removed: List[int]) -> Tuple[List[Any], List[int]]:
    """各車両の総コストに基づく貪欲法で修復.
    
    各位置で最も良い値を選択。
    
    Parameters
    ----------
    state : ALNSState
        ALNS状態
    base_cur : Sequence[Any]
        現在の基本解
    partial_x : Sequence[Any]
        部分解（削除位置はNone）
    removed : List[int]
        削除されたインデックス
    
    Returns
    -------
    Tuple[List[Any], List[int]]
        (修復された解, 変更されたインデックス) のタプル
    """
    W = state.additional_info.get("W")
    veh_keys = list(W.VEHICLES.keys())
    
    x = _copy_vec(base_cur)
    changed = []
    
    for j in removed:
        orig = x[j]
        veh = W.VEHICLES[veh_keys[j]]
        t = veh.departure_time_in_second
        o, d = veh.orig.name, veh.dest.name

        cost_min = float("inf")
        for v in state.domains[j]:
            x[j] = v
            route = W.defRoute(W.dict_od_to_routes[o, d][v])
            ext = estimate_congestion_externality_route(W, route, t)
            private_cost = route.actual_travel_time(t)
            cost = private_cost+ext
            
            if cost < cost_min:
                cost_min = cost
                best_val = v
        x[j] = best_val
        if x[j] != orig:
            changed.append(j)
    
    return x, changed


# 修復オペレータ一覧
_REPAIR_IMPLS: Dict[str, Callable[[ALNSState, Sequence[Any], Sequence[Any], List[int]], Tuple[List[Any], List[int]]]] = {
    "random": _repair_random,
    "min_cost": _repair_minimum_cost,
}

# 修復オペレータの初期重み（選択確率に比例）
_REPAIR_INITIAL_WEIGHTS: Dict[str, float] = {
    "random": 1.0,
    "min_cost": 1.0,
}

# -------------------------
# 初期化
# -------------------------
def init_alns(
    x: Sequence[Any],
    obj: Callable[[Sequence[Any]], float],
    *,
    sense: str = "min",
    domains: Optional[List[Sequence[Any]]] = None,

    # destroy/repair：指定しなければ辞書キーを自動採用
    destroy_set: Optional[List[str]] = None,
    repair_set: Optional[List[str]] = None,

    # 破壊規模
    k_min: int = None,
    k_max: int = 5,

    # 焼きなまし
    p0: float=0.2, #目標初期エラー
    trials: int=50, #初期試行回数
    T0: Optional[float] = None,
    Tend: float = 1e-3,
    total_iters: int = 2000,
    always_accept: False,

    # 適応（セグメント更新）
    segment_len: int = 50,
    adapt_eta: float = 0.2,
    reward_improve_best: float = 5.0,
    reward_improve: float = 2.0,
    reward_accept_worse: float = 0.5,

    seed: Optional[int] = 42,

    # 追加情報（任意）
    additional_info: Optional[Dict[str, Any]] = None,
) -> ALNSState:
    """ALNSアルゴリズムを初期化.
    
    Parameters
    ----------
    x : Sequence[Any]
        初期解
    obj : Callable[[Sequence[Any]], float]
        目的関数
    sense : str, default="min"
        最適化方向 ("min" または "max")
    domains : Optional[List[Sequence[Any]]], default=None
        各変数の定義域。Noneの場合はxから自動推定
    destroy_set : Optional[List[str]], default=None
        使用する破壊オペレータ名。Noneの場合は全て使用
    repair_set : Optional[List[str]], default=None
        使用する修復オペレータ名。Noneの場合は全て使用
    k_min : int, default=1
        破壊規模の最小値
    k_max : int, default=5
        破壊規模の最大値
    T0 : Optional[float], default=None
        初期温度。Noneの場合は自動推定
    Tend : float, default=1e-3
        終了温度
    total_iters : int, default=2000
        総イテレーション数
    segment_len : int, default=50
        適応更新のセグメント長
    adapt_eta : float, default=0.2
        適応更新の学習率
    reward_improve_best : float, default=5.0
        最良解改善時の報酬
    reward_improve : float, default=2.0
        解改善時の報酬
    reward_accept_worse : float, default=0.5
        悪化解受理時の報酬
    seed : Optional[int], default=42
        乱数シード
    additional_info : Optional[Dict[str, Any]], default=None
        追加情報（例: departure_timesなど）
    
    Returns
    -------
    ALNSState
        初期化されたALNS状態
    
    Raises
    ------
    ValueError
        senseが"min"または"max"でない場合
        domainsに空の位置がある場合
        destroy_setまたはrepair_setが空の場合
        未登録のオペレータが指定された場合
    """
    if sense not in ("min", "max"):
        raise ValueError("sense must be 'min' or 'max'")

    rng = random.Random(seed)

    if domains is None:
        domains = _infer_domains_from_x(x)
    n = len(x)
    if any(len(d) == 0 for d in domains):
        raise ValueError("domains is invalid")

    # 利用可能オペレータ一覧（辞書キー）
    available_destroy = list(_DESTROY_IMPLS.keys())
    available_repair  = list(_REPAIR_IMPLS.keys())

    if destroy_set is None:
        destroy_set = available_destroy[:]
    if repair_set is None:
        repair_set = available_repair[:]

    if not destroy_set:
        raise ValueError("destroy_set is empty")
    if not repair_set:
        raise ValueError("repair_set is empty")

    unknown_d = [m for m in destroy_set if m not in _DESTROY_IMPLS]
    unknown_r = [m for m in repair_set  if m not in _REPAIR_IMPLS]
    if unknown_d:
        raise ValueError(f"destroy_set is invalid")
    if unknown_r:
        raise ValueError(f"repair_set is invalid")

    if k_min is None:
        k_min = k_max
    k_min = max(1, min(k_min, n))
    k_max = max(k_min, min(k_max, n))
    k_cur = k_max

    x_cur = _copy_vec(x)
    additional_info = additional_info or {}

    def _eval(xx: Sequence[Any]) -> float:
        v = obj(xx)
        return -v if sense == "max" else v

    cur = _eval(x_cur)
    best = cur
    x_best = _copy_vec(x_cur)

    # T0 自動推定
    if T0 is None:
        # diffs = []
        # trials = min(20, 100)
        # for _ in range(trials):
        #     xx = _copy_vec(x_cur)
        #     k = rng.randint(k_min, min(k_max, n))
        #     idx = rng.sample(range(n), k)
        #     for j in idx:
        #         choices = [c for c in domains[j] if c != xx[j]]
        #         if choices:
        #             xx[j] = _rng_choice(rng, choices)
        #     diffs.append(max(0.0, _eval(xx) - cur))
        # scale = (sorted(diffs)[len(diffs)//2] if diffs else 1.0) or 1.0
        # T0 = max(1e-6, 3.0 * scale)
            
        deltas = []
        if total_iters > trials:
            total_iters -= trials
        else:
            trials = int(total_iters*0.5)
            total_iters -= trials
            warnings.warn("total_iters is too small for initial T0 estimation")
            
        for _ in range(trials):
            xx = _copy_vec(x_cur)
            k = rng.randint(k_min, min(k_max, n))
            idx = rng.sample(range(n), k)
            for j in idx:
                choices = [c for c in domains[j] if c != xx[j]]
                if choices:
                    xx[j] = _rng_choice(rng, choices)
            d = _eval(xx) - cur
            if d > 0:               # 悪化のみ収集
                deltas.append(d)
        if not deltas:
            deltas = [1.0]
        med = sorted(deltas)[len(deltas)//2]
        p0 = p0                 # 初期に悪化を割合p0前後で受理したい：かなりずれる．．．
        T0 = max(1e-6, med / max(1e-12, math.log(1.0/p0)))

    # 適応初期値（初期重みは定数から取得、定義されていなければ1.0）
    d_weights = {m: _DESTROY_INITIAL_WEIGHTS.get(m, 1.0) for m in destroy_set}
    d_scores  = {m: 0.0 for m in destroy_set}
    d_uses    = {m: 0   for m in destroy_set}
    d_best_improves = {m: 0 for m in destroy_set}

    r_weights = {m: _REPAIR_INITIAL_WEIGHTS.get(m, 1.0) for m in repair_set}
    r_scores  = {m: 0.0 for m in repair_set}
    r_uses    = {m: 0   for m in repair_set}
    r_best_improves = {m: 0 for m in repair_set}

    params = dict(
        sense=sense,
        destroy_set=destroy_set, repair_set=repair_set,
        available_destroy=available_destroy, available_repair=available_repair,
        k_min=k_min, k_max=k_max, k_cur=k_cur,
        T0=T0, Tend=Tend, total_iters=total_iters,
        segment_len=segment_len, adapt_eta=adapt_eta,
        reward_improve_best=reward_improve_best, reward_improve=reward_improve,
        reward_accept_worse=reward_accept_worse,
        seed=seed
    )

    return ALNSState(
        sense=sense, obj=obj, domains=domains,
        destroy_set=destroy_set, repair_set=repair_set,
        k_min=k_min, k_max=k_max,k_cur=k_cur,
        always_accept=always_accept,
        T0=T0, Tend=Tend, total_iters=total_iters,
        segment_len=segment_len, adapt_eta=adapt_eta,
        reward_improve_best=reward_improve_best, reward_improve=reward_improve,
        reward_accept_worse=reward_accept_worse,
        rng=rng, seed=seed, params=params,
        additional_info=additional_info,
        x_cur=x_cur, x_best=x_best, cur=cur, best=best,
        it=0, last_best_it=0,
        d_weights=d_weights, d_scores=d_scores, d_uses=d_uses, d_best_improves=d_best_improves,
        r_weights=r_weights, r_scores=r_scores, r_uses=r_uses, r_best_improves=r_best_improves,
        seg_countdown=segment_len,
        steps=[]
    )

# -------------------------
# 温度・ルーレット・重み更新
# -------------------------
def _temperature(state: ALNSState, it_next: Optional[int] = None) -> float:
    """焼きなまし温度を計算.
    
    指数的に減少する温度スケジュール。
    
    Parameters
    ----------
    state : ALNSState
        ALNS状態
    it_next : Optional[int], default=None
        次のイテレーション番号。Noneの場合は現在のitを使用
    
    Returns
    -------
    float
        現在の温度
    """
    it = state.it if it_next is None else it_next
    it = max(0, min(it, max(1, state.total_iters)))
    return state.T0 * ((state.Tend / state.T0) ** (it / max(1, state.total_iters)))

def _roulette_pick(rng: random.Random, weights: Dict[str, float]) -> str:
    """ルーレット選択.
    
    重みに比例した確率でキーを選択。
    
    Parameters
    ----------
    rng : random.Random
        乱数生成器
    weights : Dict[str, float]
        各キーの重み
    
    Returns
    -------
    str
        選択されたキー
    """
    total = sum(max(1e-12, w) for w in weights.values())
    r = rng.random() * total
    s = 0.0
    last = None
    for k, w in weights.items():
        s += max(1e-12, w)
        last = k
        if r <= s:
            return k
    return last or next(iter(weights))

def _segment_update_weights(state: ALNSState):
    """セグメント終了時にオペレータ重みを更新.
    
    スコアと使用回数に基づいて適応的に重みを調整。
    
    Parameters
    ----------
    state : ALNSState
        ALNS状態
    """
    if state.seg_countdown > 0:
        return
    for m in state.destroy_set:
        uses = max(1, state.d_uses[m])
        target = max(1e-6, state.d_scores[m] / uses)
        w_old = state.d_weights[m]
        state.d_weights[m] = (1 - state.adapt_eta) * w_old + state.adapt_eta * target
        state.d_scores[m] = 0.0
        state.d_uses[m] = 0
    for m in state.repair_set:
        uses = max(1, state.r_uses[m])
        target = max(1e-6, state.r_scores[m] / uses)
        w_old = state.r_weights[m]
        state.r_weights[m] = (1 - state.adapt_eta) * w_old + state.adapt_eta * target
        state.r_scores[m] = 0.0
        state.r_uses[m] = 0
    state.seg_countdown = state.segment_len

def _reward_step(state: ALNSState, d: str, r: str, *, improved_best: bool, accepted: bool, delta: float):
    """ステップ結果に基づいてオペレータに報酬を付与.
    
    Parameters
    ----------
    state : ALNSState
        ALNS状態
    d : str
        使用された破壊オペレータ名
    r : str
        使用された修復オペレータ名
    improved_best : bool
        最良解が改善されたか
    accepted : bool
        ムーブが受理されたか
    delta : float
        目的関数値の差分
    """
    if improved_best:
        state.d_scores[d] += state.reward_improve_best
        state.r_scores[r] += state.reward_improve_best
    elif accepted and delta < 0:
        state.d_scores[d] += state.reward_improve
        state.r_scores[r] += state.reward_improve
    elif accepted and delta > 0:
        state.d_scores[d] += state.reward_accept_worse
        state.r_scores[r] += state.reward_accept_worse

# -------------------------
# 自動ステップ（ALNS）
# -------------------------
def auto_step(state: ALNSState) -> StepLog:
    """ALNSの1ステップを実行.
    
    破壊・修復オペレータを選択し、現在解を変更、
    焼きなましに基づいて受理判定を行う。
    
    Parameters
    ----------
    state : ALNSState
        ALNS状態
    
    Returns
    -------
    StepLog
        実行されたステップのログ
    """
    # kを計算
    temp_cur = _temperature(state, state.it)
    state.k_cur = int((state.k_max-state.k_min)*temp_cur/state.T0 + state.k_min)
    if state.k_cur < state.k_min:
        state.k_cur = state.k_min
    if state.k_cur > state.k_max:
        state.k_cur = state.k_max

    # オペレータ選択
    d = _roulette_pick(state.rng, state.d_weights)
    r = _roulette_pick(state.rng, state.r_weights)
    state.d_uses[d] += 1
    state.r_uses[r] += 1

    # 破壊
    removed, partial = _DESTROY_IMPLS[d](state, state.x_cur)
    # 修復
    x_cand, changed_idx = _REPAIR_IMPLS[r](state, state.x_cur, partial, removed)

    # 評価・受理
    cand = _eval_internal(state, x_cand)
    delta = cand - state.cur
    T = _temperature(state, it_next=state.it + 1)

    accept = (((delta <= 0) or (random.random() < math.exp(-delta / max(T, 1e-12)))) or state.always_accept) and len(changed_idx) > 0
    if accept:
        state.x_cur, state.cur = x_cand, cand

    improved_best = False
    if state.cur < state.best:
        state.best = state.cur
        state.x_best = _copy_vec(state.x_cur)
        improved_best = True
        state.last_best_it = state.it + 1
        state.d_best_improves[d] += 1
        state.r_best_improves[r] += 1

    _reward_step(state, d, r, improved_best=improved_best, accepted=accept, delta=delta)

    state.it += 1
    state.seg_countdown -= 1
    _segment_update_weights(state)

    log = StepLog(
        iter=state.it, move=f"D:{d}+R:{r}", changed_idx=changed_idx, changed_idxs=len(changed_idx),
        delta_obj=delta, accepted=accept,
        obj_val=(state.cur if state.sense == "min" else -state.cur),
        best_obj=(state.best if state.sense == "min" else -state.best),
        temperature=T, source="auto"
    )
    state.steps.append(log)

    return log

def run_auto(state: ALNSState, n: int = 100, time_limit_s: Optional[float] = None) -> List[StepLog]:
    """複数ステップを自動実行.
    
    Parameters
    ----------
    state : ALNSState
        ALNS状態
    n : int, default=100
        実行するステップ数
    time_limit_s : Optional[float], default=None
        時間制限（秒）。Noneの場合は制限なし
    
    Returns
    -------
    List[StepLog]
        実行されたステップのログリスト
    """
    start_it = state.it
    t0 = time.time()
    while state.it - start_it < n:
        if time_limit_s is not None and (time.time() - t0) >= time_limit_s:
            break
        auto_step(state)
    return state.steps[start_it:state.it]

# -------------------------
# 取得・評価・完了
# -------------------------
def get_current_x(state: ALNSState) -> List[Any]:
    """現在解を取得.
    
    Parameters
    ----------
    state : ALNSState
        ALNS状態
    
    Returns
    -------
    List[Any]
        現在解のコピー
    """
    return _copy_vec(state.x_cur)

def get_best_x(state: ALNSState) -> List[Any]:
    """最良解を取得.
    
    Parameters
    ----------
    state : ALNSState
        ALNS状態
    
    Returns
    -------
    List[Any]
        最良解のコピー
    """
    return _copy_vec(state.x_best)

# def evaluate_candidates(state: ALNSState, xs: Iterable[Sequence[Any]]) -> List[float]:
#     """複数の候補解を評価.
    
#     Parameters
#     ----------
#     state : ALNSState
#         ALNS状態
#     xs : Iterable[Sequence[Any]]
#         評価する候補解のイテラブル
    
#     Returns
#     -------
#     List[float]
#         各候補解の目的関数値（ユーザー向き）
#     """
#     vals = []
#     for x in xs:
#         v = state.obj(x)
#         vals.append(v if state.sense == "min" else -v)
#     return vals

def finalize(state: ALNSState, stop_reason: str = "") -> Tuple[Sequence[Any], RunLog]:
    """ALNS実行を終了し、結果を取得.
    
    Parameters
    ----------
    state : ALNSState
        ALNS状態
    stop_reason : str, default=""
        停止理由
    
    Returns
    -------
    Tuple[Sequence[Any], RunLog]
        (最良解, 実行ログ) のタプル
    """
    run = RunLog(
        steps=list(state.steps),
        best_x=_copy_vec(state.x_best),
        best_obj=(state.best if state.sense == "min" else -state.best),
        seed=state.seed,
        stop_reason=stop_reason,
        operator_stats={
            "destroy": {
                m: dict(weight=state.d_weights[m], uses=state.d_uses[m], best_improves=state.d_best_improves[m])
                for m in state.destroy_set
            },
            "repair": {
                m: dict(weight=state.r_weights[m], uses=state.r_uses[m], best_improves=state.r_best_improves[m])
                for m in state.repair_set
            },
        },
        params=dict(state.params),
    )
    return _copy_vec(state.x_best), run

# -------------------------
# デモ（__main__）
# -------------------------
# if __name__ == "__main__":
#     # 例：各位置 ∈ {0,1,2,3}、重み付き合計を target に合わせる（制約なし）
#     random.seed(42)
#     n_elem = 20
#     w = [random.uniform(0,10) for _ in range(n_elem)]
#     target = sum(w)*1.5
#     x0 = [0 for _ in range(n_elem)]
#     domains = [[0,1,2,3] for _ in x0]

#     def obj_example(x1):
#         return abs(sum(wj*xj for wj, xj in zip(w, x1)) - target)

#     state = init_alns(
#         x0, obj_example,
#         domains=domains,
#         k_min=1, k_max=4,
#         total_iters=2000,
#         segment_len=50,
#         seed=0,
#         additional_info={"departure_times": np.linspace(0,1,len(x0))} #無意味な例
#     )

#     # 1000 ステップだけ走らせて終了
#     run_auto(state, n=1000)
#     x_star, run = finalize(state, stop_reason="done")

#     try:
#         df = run.to_dataframe()
#         print(df.to_string())
#     except Exception:
#         pass

#     print("best obj:", run.best_obj)
#     print("best x  :", x_star)
#     print("op stats:")
#     pprint(run.operator_stats)
