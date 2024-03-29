"""
Analyzer for a UXsim simulation result.
This module is automatically loaded when you import the `uxsim` module.
"""

import numpy as np
import matplotlib.pyplot as plt
import random, glob, os, csv, time
import pandas as pd
from PIL import Image, ImageDraw, ImageFont
from PIL.Image import Resampling
from tqdm.auto import tqdm
from collections import defaultdict as ddict
from importlib.resources import read_binary #according to official doc, this is also not recommended
import io
from scipy.sparse.csgraph import floyd_warshall

from .utils  import *

plt.rcParams["font.family"] = "monospace"
if "MS Gothic" in plt.rcParams["font.family"]:
    plt.rcParams["font.family"] = "MS Gothic"


class Analyzer:
    """
    Class for analyzing and visualizing a simulation result.
    """

    def __init__(s, W):
        """
        Create result analysis object.

        Parameters
        ----------
        W : object
            The world to which this belongs.
        """
        s.W = W

        os.makedirs(f"out{s.W.name}", exist_ok=True)

        #基礎統計量
        s.average_speed = 0
        s.average_speed_count = 0
        s.trip_completed = 0
        s.trip_all = 0
        s.total_travel_time = 0
        s.average_travel_time = 0

        #フラグ
        s.flag_edie_state_computed = 0
        s.flag_trajectory_computed = 0
        s.flag_pandas_convert = 0
        s.flag_od_analysis = 0

    def basic_analysis(s):
        """
        Analyze basic stats.
        """
        df = s.W.analyzer.od_to_pandas()

        s.trip_completed = np.sum(df["completed_trips"])
        s.trip_all = np.sum(df["total_trips"])

        if s.trip_completed:
            s.total_travel_time = np.sum(df["completed_trips"]*df["average_travel_time"])
            s.average_travel_time = s.total_travel_time/s.trip_completed
            s.total_delay = np.sum(df["completed_trips"]*(df["average_travel_time"]-df["free_travel_time"]))
            s.average_delay = s.total_delay/s.trip_completed
        else:
            s.total_travel_time = -1
            s.average_travel_time = -1
            s.total_delay = -1
            s.average_delay = -1


    def od_analysis(s):
        """
        Analyze OD-specific stats: number of trips, number of completed trips, free-flow travel time, average travel time, its std
        """
        if s.flag_od_analysis:
            return 0
        else:
            s.flag_od_analysis = 1

        s.od_trips = ddict(lambda: 0)
        s.od_trips_comp = ddict(lambda: 0)
        s.od_tt_free = ddict(lambda: 0)
        s.od_tt = ddict(lambda: [])
        s.od_tt_ave = ddict(lambda: 0)
        s.od_tt_std = ddict(lambda: 0)
        dn = s.W.DELTAN

        #自由旅行時間
        adj_mat_time = np.zeros([len(s.W.NODES), len(s.W.NODES)])
        for link in s.W.LINKS:
            i = link.start_node.id
            j = link.end_node.id
            if s.W.ADJ_MAT[i,j]:
                adj_mat_time[i,j] = link.length/link.u
                if link.capacity_in == 0: #流入禁止の場合は通行不可
                    adj_mat_time[i,j] = np.inf
            else:
                adj_mat_time[i,j] = np.inf
        dist = floyd_warshall(adj_mat_time)

        for veh in s.W.VEHICLES.values():
            o = veh.orig
            d = veh.dest
            s.od_trips[o,d] += dn
            if veh.travel_time != -1:
                s.od_trips_comp[o,d] += dn
                s.od_tt[o,d].append(veh.travel_time)
        for o,d in s.od_tt.keys():
            s.od_tt_ave[o,d] = np.average(s.od_tt[o,d])
            s.od_tt_std[o,d] = np.std(s.od_tt[o,d])
            s.od_tt_free[o,d] = dist[o.id, d.id]

    def link_analysis_coarse(s):
        """
        Analyze link-level coarse stats: traffic volume, remaining vehicles, free-flow travel time, average travel time, its std.
        """
        s.linkc_volume = ddict(lambda:0)
        s.linkc_tt_free = ddict(lambda:0)
        s.linkc_tt_ave = ddict(lambda:-1)
        s.linkc_tt_std = ddict(lambda:-1)
        s.linkc_remain = ddict(lambda:0)

        for l in s.W.LINKS:
            s.linkc_volume[l] = l.cum_departure[-1]
            s.linkc_remain[l] = l.cum_arrival[-1]-l.cum_departure[-1]
            s.linkc_tt_free[l] = l.length/l.u
            if s.linkc_volume[l]:
                s.linkc_tt_ave[l] = np.average([t for t in l.traveltime_actual if t>0])
                s.linkc_tt_std[l] = np.std([t for t in l.traveltime_actual if t>0])

    def compute_accurate_traj(s):
        """
        Generate more complete vehicle trajectories for each link by extrapolating recorded trajectories. It is assumed that vehicles are in free-flow travel at the end of the link.
        """
        if s.flag_trajectory_computed:
            return 0
        else:
            s.flag_trajectory_computed = 1

        for veh in s.W.VEHICLES.values():
            l_old = None
            for i in lange(veh.log_t):
                if veh.log_link[i] != -1:
                    l = s.W.get_link(veh.log_link[i])
                    if l_old != l:
                        l.tss.append([])
                        l.xss.append([])
                        l.cs.append(veh.color)
                        l.names.append(veh.name)

                    l_old = l
                    l.tss[-1].append(veh.log_t[i])
                    l.xss[-1].append(veh.log_x[i])

        for l in s.W.LINKS:
            #端部を外挿
            for i in lange(l.xss):
                if len(l.xss[i]):
                    if l.xss[i][0] != 0:
                        x_remain = l.xss[i][0]
                        if x_remain/l.u > s.W.DELTAT*0.01:
                            l.xss[i].insert(0, 0)
                            l.tss[i].insert(0, l.tss[i][0]-x_remain/l.u)
                    if l.length-l.u*s.W.DELTAT <= l.xss[i][-1] < l.length:
                        x_remain = l.length-l.xss[i][-1]
                        if x_remain/l.u > s.W.DELTAT*0.01:
                            l.xss[i].append(l.length)
                            l.tss[i].append(l.tss[i][-1]+x_remain/l.u)

    def compute_edie_state(s):
        """
        Compute Edie's traffic state for each link.
        """
        if s.flag_edie_state_computed:
            return 0
        else:
            s.flag_edie_state_computed = 1

        s.compute_accurate_traj()
        for l in s.W.LINKS:
            DELTAX = l.edie_dx
            DELTATE = l.edie_dt
            MAXX = l.length
            MAXT = s.W.TMAX

            dt = DELTATE
            dx = DELTAX
            tn = [[ddict(lambda: 0) for i in range(int(MAXX/DELTAX))] for j in range(int(MAXT/DELTATE))]
            dn = [[ddict(lambda: 0) for i in range(int(MAXX/DELTAX))] for j in range(int(MAXT/DELTATE))]

            l.k_mat = np.zeros([int(MAXT/DELTATE), int(MAXX/DELTAX)])
            l.q_mat = np.zeros([int(MAXT/DELTATE), int(MAXX/DELTAX)])
            l.v_mat = np.zeros([int(MAXT/DELTATE), int(MAXX/DELTAX)])

            for v in lange(l.xss):
                for i in lange(l.xss[v][:-1]):
                    i0 = l.names[v]
                    x0 = l.xss[v][i]
                    x1 = l.xss[v][i+1]
                    t0 = l.tss[v][i]
                    t1 = l.tss[v][i+1]
                    if t1-t0 != 0:
                        v0 = (x1-x0)/(t1-t0)
                    else:
                        #compute_accurate_traj()の外挿で極稀にt1=t0になったのでエラー回避（もう起きないはずだが念のため）
                        v0 = 0

                    tt = int(t0//dt)
                    xx = int(x0//dx)

                    if v0 > 0:
                        #残り
                        xl0 = dx-x0%dx
                        xl1 = x1%dx
                        tl0 = xl0/v0
                        tl1 = xl1/v0

                        if tt < int(MAXT/DELTATE) and xx < int(MAXX/DELTAX):
                            if xx == x1//dx:
                                #(x,t)
                                dn[tt][xx][i0] += x1-x0
                                tn[tt][xx][i0] += t1-t0
                            else:
                                #(x+n, t)
                                jj = int(x1//dx-xx+1)
                                for j in range(jj):
                                    if xx+j < int(MAXX/DELTAX):
                                        if j == 0:
                                            dn[tt][xx+j][i0] += xl0
                                            tn[tt][xx+j][i0] += tl0
                                        elif j == jj-1:
                                            dn[tt][xx+j][i0] += xl1
                                            tn[tt][xx+j][i0] += tl1
                                        else:
                                            dn[tt][xx+j][i0] += dx
                                            tn[tt][xx+j][i0] += dx/v0
                    else:
                        if tt < int(MAXT/DELTATE) and xx < int(MAXX/DELTAX):
                            dn[tt][xx][i0] += 0
                            tn[tt][xx][i0] += t1-t0

            for i in lange(tn):
                for j in lange(tn[i]):
                    t = list(tn[i][j].values())*s.W.DELTAN
                    d = list(dn[i][j].values())*s.W.DELTAN
                    l.tn_mat[i,j] = sum(t)
                    l.dn_mat[i,j] = sum(d)
                    l.k_mat[i,j] = l.tn_mat[i,j]/DELTATE/DELTAX
                    l.q_mat[i,j] = l.dn_mat[i,j]/DELTATE/DELTAX
            with np.errstate(invalid="ignore"):
                l.v_mat = l.q_mat/l.k_mat
            l.v_mat = np.nan_to_num(l.v_mat, nan=l.u)

    @catch_exceptions_and_warn()
    def print_simple_stats(s, force_print=False):
        """
        Prints basic statistics of simulation result.

        Parameters
        ----------
        force_print : bool, optional
            print the stats regardless of the value of `print_mode`
        """
        s.W.print("results:")
        s.W.print(f" average speed:\t {s.average_speed:.1f} m/s")
        s.W.print(" number of completed trips:\t", s.trip_completed, "/", len(s.W.VEHICLES)*s.W.DELTAN)
        if s.trip_completed > 0:
            s.W.print(f" average travel time of trips:\t {s.average_travel_time:.1f} s")
            s.W.print(f" average delay of trips:\t {s.average_delay:.1f} s")
            s.W.print(f" delay ratio:\t\t\t {s.average_delay/s.average_travel_time:.3f}")

        if force_print == 1 and s.W.print_mode == 0:
            print("results:")
            print(f" average speed:\t {s.average_speed:.1f} m/s")
            print(" number of completed trips:\t", s.trip_completed, "/", len(s.W.VEHICLES)*s.W.DELTAN)
            if s.trip_completed > 0:
                print(f" average travel time of trips:\t {s.average_travel_time:.1f} s")
                print(f" average delay of trips:\t {s.average_delay:.1f} s")
                print(f" delay ratio:\t\t\t {s.average_delay/s.average_travel_time:.3f}")

    def comp_route_travel_time(s, t, route):
        pass

    @catch_exceptions_and_warn()
    def time_space_diagram_traj(s, links=None, figsize=(12,4), plot_signal=True, xlim=None, ylim=None):
        """
        Draws the time-space diagram of vehicle trajectories for vehicles on specified links.

        Parameters
        ----------
        links : list of link or link, optional
            The names of the links for which the time-space diagram is to be plotted.
            If None, the diagram is plotted for all links in the network. Default is None.
        figsize : tuple of int, optional
            The size of the figure to be plotted, default is (12,4).
        plot_signal : bool, optional
            Plot the downstream signal red light.
        """

        #リンク車両軌跡の時空間図
        s.W.print(" drawing trajectories...")
        s.compute_accurate_traj()

        #対象がlistであればOKで，単一な場合にはlistに変換する．未指定であれば全部にする．
        if links == None:
            links = s.W.LINKS
        try:
            iter(links)
            if type(links) == str:
                links = [links]
        except TypeError:
            links = [links]

        for lll in tqdm(links, disable=(s.W.print_mode==0)):
            l = s.W.get_link(lll)

            plt.figure(figsize=figsize)
            plt.title(l)
            for i in range(len(l.xss)):
                plt.plot(l.tss[i], l.xss[i], c=l.cs[i], lw=0.5)
            if plot_signal:
                signal_log = [i*s.W.DELTAT for i in lange(l.end_node.signal_log) if (l.end_node.signal_log[i] not in l.signal_group and len(l.end_node.signal)>1)]
                plt.plot(signal_log, [l.length for i in lange(signal_log)], "r.")
            plt.xlabel("time (s)")
            plt.ylabel("space (m)")
            if xlim == None:
                plt.xlim([0, s.W.TMAX])
            else:
                plt.xlim(xlim)
            if ylim == None:
                plt.ylim([0, l.length])
            else:
                plt.ylim(ylim)
            plt.grid()
            plt.tight_layout()
            if s.W.save_mode:
                plt.savefig(f"out{s.W.name}/tsd_traj_{l.name}.png")
            if s.W.show_mode:
                plt.show()
            else:
                plt.close("all")

    @catch_exceptions_and_warn()
    def time_space_diagram_density(s, links=None, figsize=(12,4), plot_signal=True, xlim=None, ylim=None):
        """
        Draws the time-space diagram of traffic density on specified links.

        Parameters
        ----------
        links : list of link or link, optional
            The names of the links for which the time-space diagram is to be plotted.
            If None, the diagram is plotted for all links in the network. Default is None.
        figsize : tuple of int, optional
            The size of the figure to be plotted, default is (12,4).
        plot_signal : bool, optional
            Plot the downstream signal red light.
        """

        #リンク密度の時空間図
        s.W.print(" drawing traffic states...")
        s.compute_edie_state()

        #対象がlistであればOKで，単一な場合にはlistに変換する．未指定であれば全部にする．
        if links == None:
            links = s.W.LINKS
        try:
            iter(links)
            if type(links) == str:
                links = [links]
        except TypeError:
            links = [links]

        s.compute_edie_state()

        for lll in tqdm(links, disable=(s.W.print_mode==0)):
            l = s.W.get_link(lll)

            plt.figure(figsize=figsize)
            plt.title(l)
            plt.imshow(l.k_mat.T, origin="lower", aspect="auto",
                extent=(0, int(s.W.TMAX/l.edie_dt)*l.edie_dt, 0, int(l.length/l.edie_dx)*l.edie_dx),
                interpolation="none", vmin=0, vmax=1/l.delta, cmap="inferno")
            if plot_signal:
                signal_log = [i*s.W.DELTAT for i in lange(l.end_node.signal_log) if (l.end_node.signal_log[i] not in l.signal_group and len(l.end_node.signal)>1)]
                plt.plot(signal_log, [l.length for i in lange(signal_log)], "r.")
            plt.colorbar().set_label("density (veh/m)")
            plt.xlabel("time (s)")
            plt.ylabel("space (m)")
            if xlim == None:
                plt.xlim([0, s.W.TMAX])
            else:
                plt.xlim(xlim)
            if ylim == None:
                plt.ylim([0, l.length])
            else:
                plt.ylim(ylim)
            plt.tight_layout()

            if s.W.save_mode:
                plt.savefig(f"out{s.W.name}/tsd_k_{l.name}.png")
            if s.W.show_mode:
                plt.show()
            else:
                plt.close("all")

    @catch_exceptions_and_warn()
    def time_space_diagram_traj_links(s, linkslist, figsize=(12,4), plot_signal=True, xlim=None, ylim=None):
        """
        Draws the time-space diagram of vehicle trajectories for vehicles on concective links.

        Parameters
        ----------
        linkslist : list of link or list of list of link
            The names of the concective links for which the time-space diagram is to be plotted.
        figsize : tuple of int, optional
            The size of the figure to be plotted, default is (12,4).
        plot_signal : bool, optional
            Plot the signal red light.
        """
        #複数リンクの連続した車両軌跡の時空間図
        s.W.print(" drawing trajectories in consecutive links...")
        s.compute_accurate_traj()

        #リンクリストのリストであればそのまま，そうでなければリスト化
        try:
            iter(linkslist[0])
            if type(linkslist[0]) == str:
                linkslist = [linkslist]
        except TypeError:
            linkslist = [linkslist]

        for links in linkslist:
            linkdict = {}
            d = 0
            for ll in links:
                l = s.W.get_link(ll)
                linkdict[l] = d
                d += l.length

            plt.figure(figsize=figsize)
            for ll in links:
                l = s.W.get_link(ll)
                for i in range(len(l.xss)):
                    plt.plot(l.tss[i], np.array(l.xss[i])+linkdict[l], c=l.cs[i], lw=0.5)
                if plot_signal:
                    signal_log = [i*s.W.DELTAT for i in lange(l.end_node.signal_log) if (l.end_node.signal_log[i] not in l.signal_group and len(l.end_node.signal)>1)]
                    plt.plot(signal_log, [l.length+linkdict[l] for i in lange(signal_log)], "r.")
            for l in linkdict.keys():
                plt.plot([0, s.W.TMAX], [linkdict[l], linkdict[l]], "k--", lw=0.7)
                plt.plot([0, s.W.TMAX], [linkdict[l]+l.length, linkdict[l]+l.length], "k--", lw=0.7)
                plt.text(0, linkdict[l]+l.length/2, l.name, va="center", c="b")
                plt.text(0, linkdict[l], l.start_node.name, va="center", c="g")
                plt.text(0, linkdict[l]+l.length, l.end_node.name, va="center", c="g")
            plt.xlabel("time (s)")
            plt.ylabel("space (m)")
            plt.xlim([0, s.W.TMAX])
            if xlim == None:
                plt.xlim([0, s.W.TMAX])
            else:
                plt.xlim(xlim)
            if ylim == None:
                pass
            else:
                plt.ylim(ylim)
            plt.grid()
            plt.tight_layout()
            if s.W.save_mode:
                plt.savefig(f"out{s.W.name}/tsd_traj_links_{'-'.join([s.W.get_link(l).name for l in links])}.png")
            if s.W.show_mode:
                plt.show()
            else:
                plt.close("all")

    @catch_exceptions_and_warn()
    def cumulative_curves(s, links=None, figsize=(6,4)):
        """
        Plots the cumulative curves and travel times for the provided links.

        Parameters
        ----------
        links : list or object, optional
            A list of links or a single link for which the cumulative curves and travel times are to be plotted.
            If not provided, the cumulative curves and travel times for all the links in the network will be plotted.
        figsize : tuple of int, optional
            The size of the figure to be plotted. Default is (6, 4).

        Notes
        -----
        This method plots the cumulative curves for vehicle arrivals and departures, as well as the instantaneous and actual travel times.
        The plots are saved to the directory `out<W.name>` with the filename format `cumulative_curves_<link.name>.png`.
        """

        #対象がlistであればOKで，単一な場合にはlistに変換する．未指定であれば全部にする．
        if links == None:
            links = s.W.LINKS
        try:
            iter(links)
            if type(links) == str:
                flag_single = 1
        except TypeError:
            links = [links]

        for link in links:
            link = s.W.get_link(link)

            fig, ax1 = plt.subplots(figsize=figsize)
            plt.title(link)
            ax2 = ax1.twinx()

            ax1.plot(range(0, s.W.TMAX, s.W.DELTAT), link.cum_arrival, "r-", label="arrival")
            ax1.plot(range(0, s.W.TMAX, s.W.DELTAT), link.cum_departure, "b-", label="departure")

            ax2.plot(range(0, s.W.TMAX, s.W.DELTAT), link.traveltime_instant, ".", c="gray", ms=0.5, label="instantaneous")
            ax2.plot(range(0, s.W.TMAX, s.W.DELTAT), link.traveltime_actual, "k.", ms=1, label="actual")

            ax1.set_ylim(ymin=0)
            ax2.set_ylim(ymin=0)

            ax1.set_xlabel("time(s)")
            ax1.set_ylabel("cumlative count (veh)")
            ax2.set_ylabel("travel time (s)")

            ax1.legend(loc="upper left")
            ax2.legend(loc="upper right")

            ax1.grid()
            plt.tight_layout()
            if s.W.save_mode:
                plt.savefig(f"out{s.W.name}/cumlative_curves_{link.name}.png")
            if s.W.show_mode:
                plt.show()
            else:
                plt.close("all")

    @catch_exceptions_and_warn()
    def network(s, t=None, detailed=1, minwidth=0.5, maxwidth=12, left_handed=1, tmp_anim=0, figsize=(6,6), network_font_size=4, node_size=2):
        """
        Visualizes the entire transportation network and its current traffic conditions.

        Parameters
        ----------
        t : float, optional
            The current time for which the traffic conditions are visualized.
        detailed : int, optional
            Determines the level of detail in the visualization.
            If set to 1, the link internals (cell) are displayed in detail.
            If set to 0, the visualization is simplified to link-level. Default is 1.
        minwidth : float, optional
            The minimum width of the link visualization. Default is 0.5.
        maxwidth : float, optional
            The maximum width of the link visualization. Default is 12.
        left_handed : int, optional
            If set to 1, the left-handed traffic system (e.g., Japan, UK) is used. If set to 0, the right-handed one is used. Default is 1.
        tmp_anim : int, optional
            If set to 1, the visualization will be saved as a temporary animation frame. Default is 0.
        figsize : tuple of int, optional
            The size of the figure to be plotted. Default is (6, 6).
        network_font_size : int, optional
            The font size for the network labels. Default is 4.
        node_size : int, optional
            The size of the nodes in the visualization. Default is 2.

        Notes
        -----
        This method visualizes the entire transportation network and its current traffic conditions.
        The visualization provides information on vehicle density, velocity, link names, node locations, and more.
        The plots are saved to the directory `out<W.name>` with filenames depending on the `detailed` and `t` parameters.
        """
        s.compute_edie_state()

        plt.figure(figsize=figsize)
        plt.subplot(111, aspect="equal")
        plt.title(f"t = {t :>8} (s)")
        for n in s.W.NODES:
            plt.plot(n.x, n.y, "ko", ms=node_size, zorder=10)
            if network_font_size > 0:
                plt.text(n.x, n.y, n.name, c="g", horizontalalignment="center", verticalalignment="top", zorder=20, fontsize=network_font_size)
        for l in s.W.LINKS:
            x1, y1 = l.start_node.x, l.start_node.y
            x2, y2 = l.end_node.x, l.end_node.y
            vx, vy = (y1-y2)*0.05, (x2-x1)*0.05
            if not left_handed:
                vx, vy = -vx, -vy
            if detailed:
                #詳細モード
                xsize = l.k_mat.shape[1]-1
                c = ["k" for i in range(xsize)]
                lw = [1 for i in range(xsize)]
                for i in range(xsize):
                    try:
                        k = l.k_mat[int(t/l.edie_dt), i+1]
                        v = l.v_mat[int(t/l.edie_dt), i+1]
                    except:
                        warnings.warn(f"invalid time {t} is specified for network visualization", UserWarning)
                        return -1
                    lw[i] = k*l.delta*(maxwidth-minwidth)+minwidth
                    c[i] = plt.colormaps["viridis"](v/l.u)
                xmid = [((xsize-i)*x1+(i+1)*x2)/(xsize+1)+vx for i in range(xsize)]
                ymid = [((xsize-i)*y1+(i+1)*y2)/(xsize+1)+vy for i in range(xsize)]
                plt.plot([x1]+xmid+[x2], [y1]+ymid+[y2], "k--", lw=0.25, zorder=5)
                for i in range(xsize-1):
                    plt.plot([xmid[i], xmid[i+1]], [ymid[i], ymid[i+1]], c=c[i], lw=lw[i], zorder=6)
                if network_font_size > 0:
                    plt.text(xmid[int(len(xmid)/2)], ymid[int(len(xmid)/2)], l.name, c="b", zorder=20, fontsize=network_font_size)
            else:
                #簡略モード
                k = (l.cum_arrival[int(t/s.W.DELTAT)]-l.cum_departure[int(t/s.W.DELTAT)])/l.length
                v = l.length/l.traveltime_instant[int(t/s.W.DELTAT)]
                width = k*l.delta*(maxwidth-minwidth)+minwidth
                c = plt.colormaps["viridis"](v/l.u)
                xmid1, ymid1 = (2*x1+x2)/3+vx, (2*y1+y2)/3+vy
                xmid2, ymid2 = (x1+2*x2)/3+vx, (y1+2*y2)/3+vy
                plt.plot([x1, xmid1, xmid2, x2], [y1, ymid1, ymid2, y2], "k--", lw=0.25, zorder=5)
                plt.plot([x1, xmid1, xmid2, x2], [y1, ymid1, ymid2, y2], c=c, lw=width, zorder=6, solid_capstyle="butt")
                if network_font_size > 0:
                    plt.text(xmid1, ymid1, l.name, c="b", zorder=20, fontsize=network_font_size)
        maxx = max([n.x for n in s.W.NODES])
        minx = min([n.x for n in s.W.NODES])
        maxy = max([n.y for n in s.W.NODES])
        miny = min([n.y for n in s.W.NODES])
        buffx, buffy = (maxx-minx)/10, (maxy-miny)/10
        if buffx == 0:
            buffx = buffy
        if buffy == 0:
            buffy = buffx
        plt.xlim([minx-buffx, maxx+buffx])
        plt.ylim([miny-buffy, maxy+buffy])
        plt.tight_layout()
        if tmp_anim:
            plt.savefig(f"out{s.W.name}/tmp_anim_{t}.png")
            plt.close("all")
        else:
            if s.W.save_mode:
                plt.savefig(f"out{s.W.name}/network{detailed}_{t}.png")
            if s.W.show_mode:
                plt.show()
            else:
                plt.close("all")

    @catch_exceptions_and_warn()
    def network_pillow(s, t=None, detailed=1, minwidth=0.5, maxwidth=12, left_handed=1, tmp_anim=0, figsize=6, network_font_size=20, node_size=2, image_return=0):
        """
        Visualizes the entire transportation network and its current traffic conditions. Faster implementation using Pillow.

        Parameters
        ----------
        t : float, optional
            The current time for which the traffic conditions are visualized.
        detailed : int, optional
            Determines the level of detail in the visualization.
            If set to 1, the link internals (cell) are displayed in detail.
            If set to 0, the visualization is simplified to link-level. Default is 1.
        minwidth : float, optional
            The minimum width of the link visualization. Default is 0.5.
        maxwidth : float, optional
            The maximum width of the link visualization. Default is 12.
        left_handed : int, optional
            If set to 1, the left-handed traffic system (e.g., Japan, UK) is used. If set to 0, the right-handed one is used. Default is 1.
        tmp_anim : int, optional
            If set to 1, the visualization will be saved as a temporary animation frame. Default is 0.
        figsize : tuple of int, optional
            The size of the figure to be plotted. Default is (6, 6).
        network_font_size : int, optional
            The font size for the network labels. Default is 4.
        node_size : int, optional
            The size of the nodes in the visualization. Default is 2.

        Notes
        -----
        This method visualizes the entire transportation network and its current traffic conditions.
        The visualization provides information on vehicle density, velocity, link names, node locations, and more.
        The plots are saved to the directory `out<W.name>` with filenames depending on the `detailed` and `t` parameters.
        """

        maxx = max([n.x for n in s.W.NODES])
        minx = min([n.x for n in s.W.NODES])
        maxy = max([n.y for n in s.W.NODES])
        miny = min([n.y for n in s.W.NODES])

        scale = 2
        try:
            coef = figsize*100*scale/(maxx-minx)
        except:
            coef = figsize[0]*100*scale/(maxx-minx)
        maxx *= coef
        minx *= coef
        maxy *= coef
        miny *= coef
        minwidth *= scale
        maxwidth *= scale

        buffer = (maxx-minx)/10
        maxx += buffer
        minx -= buffer
        maxy += buffer
        miny -= buffer

        img = Image.new("RGBA", (int(maxx-minx), int(maxy-miny)), (255, 255, 255, 255))
        draw = ImageDraw.Draw(img)
        font_data = read_binary('uxsim.files', 'HackGen-Regular.ttf') 
        font_file_like = io.BytesIO(font_data)
        font = ImageFont.truetype(font_file_like, int(network_font_size))

        def flip(y):
            return img.size[1]-y

        for l in s.W.LINKS:
            x1, y1 = l.start_node.x*coef-minx, l.start_node.y*coef-miny
            x2, y2 = l.end_node.x*coef-minx, l.end_node.y*coef-miny
            vx, vy = (y1-y2)*0.05, (x2-x1)*0.05
            if not left_handed:
                vx, vy = -vx, -vy
            k = (l.cum_arrival[int(t/s.W.DELTAT)]-l.cum_departure[int(t/s.W.DELTAT)])/l.length
            v = l.length/l.traveltime_instant[int(t/s.W.DELTAT)]
            width = k*l.delta*(maxwidth-minwidth)+minwidth
            c = plt.colormaps["viridis"](v/l.u)
            xmid1, ymid1 = (2*x1+x2)/3+vx, (2*y1+y2)/3+vy
            xmid2, ymid2 = (x1+2*x2)/3+vx, (y1+2*y2)/3+vy
            draw.line([(x1, flip(y1)), (xmid1, flip(ymid1)), (xmid2, flip(ymid2)), (x2, flip(y2))], fill=(int(c[0]*255), int(c[1]*255), int(c[2]*255)), width=int(width), joint="curve")

            if network_font_size > 0:
                draw.text((xmid1, flip(ymid1)), l.name, font=font, fill="blue", anchor="mm")

        for n in s.W.NODES:
            if network_font_size > 0:
                draw.text(((n.x)*coef-minx, flip((n.y)*coef-miny)), n.name, font=font, fill="green", anchor="mm")
                draw.text(((n.x)*coef-minx, flip((n.y)*coef-miny)), n.name, font=font, fill="green", anchor="mm")

        font_data = read_binary('uxsim.files', 'HackGen-Regular.ttf') 
        font_file_like = io.BytesIO(font_data)
        font = ImageFont.truetype(font_file_like, int(30))
        draw.text((img.size[0]/2,20), f"t = {t :>8} (s)", font=font, fill="black", anchor="mm")

        img = img.resize((int((maxx-minx)/scale), int((maxy-miny)/scale)), resample=Resampling.LANCZOS)
        if image_return:
            return img
        elif tmp_anim:
            img.save(f"out{s.W.name}/tmp_anim_{t}.png")
        else:
            if s.W.save_mode:
                img.save(f"out{s.W.name}/network{detailed}_{t}.png")

    @catch_exceptions_and_warn()
    def show_simulation_progress(s):
        """
        Print simulation progress.
        """
        if s.W.print_mode:
            vehs = [l.density*l.length for l in s.W.LINKS]
            sum_vehs = sum(vehs)

            vs = [l.density*l.length*l.speed for l in s.W.LINKS]
            if sum_vehs > 0:
                avev = sum(vs)/sum_vehs
            else:
                avev = 0

            print(f"{s.W.TIME:>8.0f} s| {sum_vehs:>8.0f} vehs|  {avev:>4.1f} m/s| {time.time()-s.W.sim_start_time:8.2f} s", flush=True)

    @catch_exceptions_and_warn()
    def network_anim(s, animation_speed_inverse=10, detailed=0, minwidth=0.5, maxwidth=12, left_handed=1, figsize=(6,6), node_size=2, network_font_size=20, timestep_skip=24):
        """
        Generates an animation of the entire transportation network and its traffic states over time.

        Parameters
        ----------
        animation_speed_inverse : int, optional
            The inverse of the animation speed. A higher value will result in a slower animation. Default is 10.
        detailed : int, optional
            Determines the level of detail in the animation.
            If set to 1, the link internals (cell) are displayed in detail.
            Under some conditions, the detailed mode will produce inappropriate visualization.
            If set to 0, the visualization is simplified to link-level. Default is 0.
        minwidth : float, optional
            The minimum width of the link visualization in the animation. Default is 0.5.
        maxwidth : float, optional
            The maximum width of the link visualization in the animation. Default is 12.
        left_handed : int, optional
            If set to 1, the left-handed traffic system (e.g., Japan, UK) is used. If set to 0, the right-handed one is used. Default is 1.
        figsize : tuple of int, optional
            The size of the figures in the animation. Default is (6, 6).
        node_size : int, optional
            The size of the nodes in the animation. Default is 2.
        network_font_size : int, optional
            The font size for the network labels in the animation. Default is 20.
        timestep_skip : int, optional
            How many timesteps are skipped per frame. Large value means coarse and lightweight animation. Default is 8.

        Notes
        -----
        This method generates an animation visualizing the entire transportation network and its traffic conditions over time.
        The animation provides information on vehicle density, velocity, link names, node locations, and more.
        The generated animation is saved to the directory `out<W.name>` with a filename based on the `detailed` parameter.

        Temporary images used to create the animation are removed after the animation is generated.
        """
        s.W.print(" generating animation...")
        pics = []
        for t in tqdm(range(0, s.W.TMAX, s.W.DELTAT*timestep_skip), disable=(s.W.print_mode==0)):
            if int(t/s.W.LINKS[0].edie_dt) < s.W.LINKS[0].k_mat.shape[0]:
                if detailed:
                    #todo_later: 今後はこちらもpillowにする
                    s.network(int(t), detailed=detailed, minwidth=minwidth, maxwidth=maxwidth, left_handed=left_handed, tmp_anim=1, figsize=figsize, node_size=node_size, network_font_size=network_font_size)
                else:
                    s.network_pillow(int(t), detailed=detailed, minwidth=minwidth, maxwidth=maxwidth, left_handed=left_handed, tmp_anim=1, figsize=figsize, node_size=node_size, network_font_size=network_font_size)
                pics.append(Image.open(f"out{s.W.name}/tmp_anim_{t}.png"))
        pics[0].save(f"out{s.W.name}/anim_network{detailed}.gif", save_all=True, append_images=pics[1:], optimize=False, duration=animation_speed_inverse*timestep_skip, loop=0)
        for f in glob.glob(f"out{s.W.name}/tmp_anim_*.png"):
            os.remove(f)

    @catch_exceptions_and_warn()
    def network_fancy(s, animation_speed_inverse=10, figsize=6, sample_ratio=0.3, interval=5, network_font_size=0, trace_length=3, speed_coef=2):
        """
        Generates a visually appealing animation of vehicles' trajectories across the entire transportation network over time.

        Parameters
        ----------
        animation_speed_inverse : int, optional
            The inverse of the animation speed. A higher value will result in a slower animation. Default is 10.
        figsize : int or tuple of int, optional
            The size of the figures in the animation. Default is 6.
        sample_ratio : float, optional
            The fraction of vehicles to be visualized. Default is 0.3.
        interval : int, optional
            The interval at which vehicle positions are sampled. Default is 5.
        network_font_size : int, optional
            The font size for the network labels in the animation. Default is 0.
        trace_length : int, optional
            The length of the vehicles' trajectory trails in the animation. Default is 3.
        speed_coef : int, optional
            A coefficient that adjusts the animation speed. Default is 2.

        Notes
        -----
        This method generates a visually appealing animation that visualizes vehicles' trajectories across the transportation network over time.
        The animation provides information on vehicle positions, speeds, link names, node locations, and more, with Bezier curves used for smooth transitions.
        The generated animation is saved to the directory `out<W.name>` with a filename `anim_network_fancy.gif`.

        Temporary images used to create the animation are removed after the animation is generated.
        """

        s.W.print(" generating animation...")

        # ベジエ補間
        from scipy.interpolate import make_interp_spline

        #{t: ["xs":[], "ys":[], "v": v, "c":c]}
        draw_dict = ddict(lambda: [])

        maxx = max([n.x for n in s.W.NODES])
        minx = min([n.x for n in s.W.NODES])
        dcoef = (maxx-minx)/20

        for veh in s.W.VEHICLES.values():
            if random.random() > sample_ratio:
                continue
            ts = []
            xs = []
            ys = []
            vs = []
            dx = (random.random()-0.5)*dcoef
            dy = (random.random()-0.5)*dcoef
            for i in range(0, len(veh.log_t), interval):
                if veh.log_state[i] == "run":
                    link = veh.log_link[i]
                    x0 = link.start_node.x+dx
                    y0 = link.start_node.y+dy
                    x1 = link.end_node.x+dx
                    y1 = link.end_node.y+dy
                    alpha = veh.log_x[i]/link.length
                    ts.append(veh.log_t[i])
                    xs.append(x0*(1-alpha)+x1*alpha)
                    ys.append(y0*(1-alpha)+y1*alpha)
                    c = veh.color
            for i in range(0, len(veh.log_t)):
                if veh.log_state[i] == "run":
                    vs.append(veh.log_v[i]/veh.log_link[i].u)
            if len(ts) <= interval:
                continue

            # 点列
            points = np.array([xs, ys]).T

            # x, y 座標を取得
            x = points[:, 0]
            y = points[:, 1]

            # ベジエ曲線による補間
            t = np.linspace(0, 1, len(points))
            interp_size = len(ts)*interval
            t_smooth = np.linspace(0, 1, interp_size)
            bezier_spline = make_interp_spline(t, points, k=3)
            smooth_points = bezier_spline(t_smooth)
            for i in lange(t_smooth):
                ii = max(0, i-trace_length)
                if i < len(vs):
                    v = vs[i]
                else:
                    v = vs[-1]
                draw_dict[int(ts[0]+i*s.W.DELTAT)].append({
                    "xs": smooth_points[ii:i+1, 0],
                    "ys": smooth_points[ii:i+1, 1],
                    "c": veh.color,
                    "v": v
                })

        # 可視化
        maxx = max([n.x for n in s.W.NODES])
        minx = min([n.x for n in s.W.NODES])
        maxy = max([n.y for n in s.W.NODES])
        miny = min([n.y for n in s.W.NODES])

        scale = 2
        try:
            coef = figsize*100*scale/(maxx-minx)
        except:
            coef = figsize[0]*100*scale/(maxx-minx)
        maxx *= coef
        minx *= coef
        maxy *= coef
        miny *= coef

        buffer = (maxx-minx)/10
        maxx += buffer
        minx -= buffer
        maxy += buffer
        miny -= buffer

        pics = []
        for t in tqdm(range(int(s.W.TMAX*0), int(s.W.TMAX*1), s.W.DELTAT*speed_coef)):
            img = Image.new("RGBA", (int(maxx-minx), int(maxy-miny)), (255, 255, 255, 255))
            draw = ImageDraw.Draw(img)
            font_data = read_binary('uxsim.files', 'HackGen-Regular.ttf') 
            font_file_like = io.BytesIO(font_data)
            font = ImageFont.truetype(font_file_like, int(network_font_size))

            def flip(y):
                return img.size[1]-y

            for l in s.W.LINKS:
                x1, y1 = l.start_node.x*coef-minx, l.start_node.y*coef-miny
                x2, y2 = l.end_node.x*coef-minx, l.end_node.y*coef-miny
                draw.line([(x1, flip(y1)), (x2, flip(y2))], fill=(200,200,200), width=int(1), joint="curve")

                if network_font_size > 0:
                    draw.text(((x1+x2)/2, flip((y1+y2)/2)), l.name, font=font, fill="blue", anchor="mm")

            traces = draw_dict[t]
            for trace in traces:
                xs = trace["xs"]*coef-minx
                ys = trace["ys"]*coef-miny
                size = 3*(1-trace["v"])
                coords = [(l[0], flip(l[1])) for l in list(np.vstack([xs, ys]).T)]
                draw.line(coords,
                          fill=(int(trace["c"][0]*255), int(trace["c"][1]*255), int(trace["c"][2]*255)), width=2, joint="curve")
                draw.ellipse((xs[-1]-size, flip(ys[-1])-size, xs[-1]+size, flip(ys[-1])+size), fill=(int(trace["c"][0]*255), int(trace["c"][1]*255), int(trace["c"][2]*255)))
                #draw.line([(x1, flip(y1)), (xmid1, flip(ymid1)), (xmid2, flip(ymid2)), (x2, flip(y2))]

            font_data = read_binary('uxsim.files', 'HackGen-Regular.ttf') 
            font_file_like = io.BytesIO(font_data)
            font = ImageFont.truetype(font_file_like, int(30))
            draw.text((img.size[0]/2,20), f"t = {t :>8} (s)", font=font, fill="black", anchor="mm")

            img = img.resize((int((maxx-minx)/scale), int((maxy-miny)/scale)), resample=Resampling.LANCZOS)
            img.save(f"out{s.W.name}/tmp_anim_{t}.png")
            pics.append(Image.open(f"out{s.W.name}/tmp_anim_{t}.png"))

        pics[0].save(f"out{s.W.name}/anim_network_fancy.gif", save_all=True, append_images=pics[1:], optimize=False, duration=animation_speed_inverse*speed_coef, loop=0)

        for f in glob.glob(f"out{s.W.name}/tmp_anim_*.png"):
            os.remove(f)

    def compute_mfd(s, links=None):
        """
        Compute network average flow and density for MFD.
        """
        s.compute_edie_state()
        if links == None:
            links = s.W.LINKS
        links = [s.W.get_link(link) for link in links]
        links = frozenset(links)


        for i in range(len(s.W.Q_AREA[links])):
            tn = sum([l.tn_mat[i,:].sum() for l in s.W.LINKS if l in links])
            dn = sum([l.dn_mat[i,:].sum() for l in s.W.LINKS if l in links])
            an = sum([l.length*s.W.EULAR_DT for l in s.W.LINKS if l in links])
            s.W.K_AREA[links][i] = tn/an
            s.W.Q_AREA[links][i] = dn/an

    @catch_exceptions_and_warn()
    def macroscopic_fundamental_diagram(s, kappa=0.2, qmax=1, figtitle="", links=None, fname="", figsize=(4,4)):
        """
        Plots the Macroscopic Fundamental Diagram (MFD) for the provided links.

        Parameters
        ----------
        kappa : float, optional
            The maximum network average density for the x-axis of the MFD plot. Default is 0.2.
        qmax : float, optional
            The maximum network average flow for the y-axis of the MFD plot. Default is 1.
        links : list or object, optional
            A list of links or a single link for which the MFD is to be plotted.
            If not provided, the MFD for all the links in the network will be plotted.
        fname : str
            File name for saving (postfix). Default is "".
        figsize : tuple of int, optional
            The size of the figure to be plotted. Default is (4, 4).

        Notes
        -----
        This method plots the Macroscopic Fundamental Diagram (MFD) for the provided links.
        The MFD provides a relationship between the network average density and the network average flow.
        The plot is saved to the directory `out<W.name>` with the filename `mfd<fname>.png`.
        """

        if links == None:
            links = s.W.LINKS
        links = [s.W.get_link(link) for link in links]
        links = frozenset(links)
        s.compute_mfd(links)

        plt.figure(figsize=figsize)
        plt.title(f"{figtitle} (# of links: {len(links)})")
        plt.plot(s.W.K_AREA[links], s.W.Q_AREA[links], "ko-")
        plt.xlabel("network average density (veh/m)")
        plt.ylabel("network average flow (veh/s)")
        plt.xlim([0, kappa])
        plt.ylim([0, qmax])
        plt.grid()
        plt.tight_layout()
        if s.W.save_mode:
            plt.savefig(f"out{s.W.name}/mfd{fname}.png")
        if s.W.show_mode:
            plt.show()
        else:
            plt.close("all")

    @catch_exceptions_and_warn()
    def plot_vehicle_log(s, vehname):
        """
        Plots the driving link and speed for a single vehicle.

        Parameters
        ----------
        vehname : str
            The name of the vehicle for which the driving link and speed are to be plotted.

        Notes
        -----
        This method visualizes the speed profile and the links traversed by a specific vehicle over time.
        The speed is plotted on the primary y-axis, and the links are plotted on the secondary y-axis.
        The plot is saved to the directory `out<W.name>` with the filename `vehicle_<vehname>.png`.
        """
        veh = s.W.VEHICLES[vehname]

        fig, ax1 = plt.subplots()
        plt.title(f"vehicle: {veh.name}")
        ax1.fill_between(veh.log_t, 0, veh.log_v, color="c", zorder=10)
        ax1.set_ylabel('speed (m/s)', color='c')
        plt.ylim([0, None])
        plt.xlabel("time (s)")

        ax2 = ax1.twinx()
        vehlinks = [str(l.name) if l != -1 else "not in network" for l in veh.log_link]
        ax2.plot([veh.log_t[i] for i in lange(veh.log_t) if veh.log_state[i] != "home"], [vehlinks[i] for i in lange(vehlinks) if veh.log_state[i] != "home"], 'k-', zorder=20)
        ax2.grid()
        ax2.set_ylabel('link', color='k')
        plt.ylim([0, None])
        plt.tight_layout()

        if s.W.save_mode:
            plt.savefig(f"out{s.W.name}/vehicle_{vehname}.png")
        if s.W.show_mode:
            plt.show()
        else:
            plt.close("all")


    def vehicles_to_pandas(s):
        """
        Converts the vehicle travel logs to a pandas DataFrame.

        Returns
        -------
        pd.DataFrame
            A DataFrame containing the travel logs of vehicles, with the columns:
            - 'name': the name of the vehicle (platoon).
            - 'dn': the platoon size.
            - 'orig': the origin node of the vehicle's trip.
            - 'dest': the destination node of the vehicle's trip.
            - 't': the timestep.
            - 'link': the link the vehicle is on (or relevant status).
            - 'x': the position of the vehicle on the link.
            - 's': the spacing of the vehicle.
            - 'v': the speed of the vehicle.
        """
        if s.flag_pandas_convert == 0:
            s.flag_pandas_convert = 1

            out = [["name", "dn", "orig", "dest", "t", "link", "x", "s", "v"]]
            for veh in s.W.VEHICLES.values():
                for i in range(len(veh.log_t)):
                    if veh.log_state[i] in ("wait", "run", "end", "abort"):
                        if veh.log_link[i] != -1:
                            linkname = veh.log_link[i].name
                        else:
                            if veh.log_state[i] == "wait":
                                linkname = "waiting_at_origin_node"
                            elif veh.log_state[i] == "abort":
                                linkname = "trip_aborted"
                            else:
                                linkname = "trip_end"
                        out.append([veh.name, s.W.DELTAN, veh.orig.name, veh.dest.name, veh.log_t[i], linkname, veh.log_x[i], veh.log_s[i], veh.log_v[i]])
            s.df_vehicles = pd.DataFrame(out[1:], columns=out[0])
        return s.df_vehicles

    def log_vehicles_to_pandas(s):
        """
        same to `vehicles_to_pandas`, just for backward compatibility
        """
        return s.vehicles_to_pandas()

    def basic_to_pandas(s):
        """
        Converts the basic stats to a pandas DataFrame.

        Returns
        -------
        pd.DataFrame
        """
        out = [["total_trips", "completed_trips", "total_travel_time", "average_travel_time", "total_delay", "average_delay"], [s.trip_all, s.trip_completed, s.total_travel_time, s.average_travel_time, s.total_delay, s.average_delay]]

        s.df_basic = pd.DataFrame(out[1:], columns=out[0])
        return s.df_basic

    def od_to_pandas(s):
        """
        Converts the OD-specific analysis results to a pandas DataFrame.

        Returns
        -------
        pd.DataFrame
        """

        s.od_analysis()

        out = [["orig", "dest", "total_trips", "completed_trips", "free_travel_time", "average_travel_time", "stddiv_travel_time"]]
        for o,d in s.od_trips.keys():
            out.append([o.name, d.name, s.od_trips[o,d], s.od_trips_comp[o,d], s.od_tt_free[o,d], s.od_tt_ave[o,d], s.od_tt_std[o,d]])

        s.df_od = pd.DataFrame(out[1:], columns=out[0])
        return s.df_od

    def mfd_to_pandas(s, links=None):
        """
        Converts the MFD to a pandas DataFrame.

        Returns
        -------
        pd.DataFrame
        """
        if links == None:
            links = s.W.LINKS
        s.compute_mfd(links)
        links = [s.W.get_link(link) for link in links]
        links = frozenset(links)

        out = [["t", "network_k", "network_q"]]
        for i in lange(s.W.K_AREA):
            out.append([i*s.W.EULAR_DT, s.W.K_AREA[links][i], s.W.Q_AREA[links][i]])
        s.df_mfd = pd.DataFrame(out[1:], columns=out[0])
        return s.df_mfd

    def link_to_pandas(s):
        """
        Converts the link-level analysis results to a pandas DataFrame.

        Returns
        -------
        pd.DataFrame
        """
        s.link_analysis_coarse()

        out = [["link", "traffic_volume", "vehicles_remain", "free_travel_time", "average_travel_time", "stddiv_travel_time"]]
        for l in s.W.LINKS:
            out.append([l.name, s.linkc_volume[l], s.linkc_remain[l], s.linkc_tt_free[l], s.linkc_tt_ave[l], s.linkc_tt_std[l]])
        s.df_linkc = pd.DataFrame(out[1:], columns=out[0])
        return s.df_linkc

    def link_traffic_state_to_pandas(s):
        """
        Converts the traffic states in links to a pandas DataFrame.

        Returns
        -------
        pd.DataFrame
        """
        s.compute_edie_state()

        out = [["link", "t", "x", "delta_t", "delta_x", "q", "k", "v"]]
        for l in s.W.LINKS:
            for i in range(l.k_mat.shape[0]):
                for j in range(l.k_mat.shape[1]):
                    out.append([l.name, i*l.edie_dt, j*l.edie_dx, l.edie_dt, l.edie_dx, l.q_mat[i,j], l.k_mat[i,j], l.v_mat[i,j]])
        s.df_link_traffic_state = pd.DataFrame(out[1:], columns=out[0])
        return s.df_link_traffic_state

    def link_cumulative_to_pandas(s):
        """
        Converts the cumulative counts etc. in links to a pandas DataFrame.

        Returns
        -------
        pd.DataFrame
        """
        out = [["link", "t", "arrival_count", "departure_count", "actual_travel_time", "instantanious_travel_time"]]
        for link in s.W.LINKS:
            for i in range(s.W.TSIZE):
                out.append([link.name, i*s.W.DELTAT, link.cum_arrival[i], link.cum_departure[i], link.traveltime_actual[i], link.traveltime_instant[i]])
        s.df_link_cumulative = pd.DataFrame(out[1:], columns=out[0])
        return s.df_link_cumulative

    @catch_exceptions_and_warn()
    def output_data(s, fname=None):
        """
        Save all results to CSV files
        """
        if fname == None:
            fname = f"out{s.W.name}/data"
        s.basic_to_pandas().to_csv(fname+"_basic.csv", index=False)
        s.od_to_pandas().to_csv(fname+"_od.csv", index=False)
        s.mfd_to_pandas().to_csv(fname+"_mfd.csv", index=False)
        s.link_to_pandas().to_csv(fname+"_link.csv", index=False)
        s.link_traffic_state_to_pandas().to_csv(fname+"_link_traffic_state.csv", index=False)
        s.vehicles_to_pandas().to_csv(fname+"_vehicles.csv", index=False)
