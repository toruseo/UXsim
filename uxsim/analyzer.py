"""
Analyzer for a UXsim simulation result.
This module is automatically loaded when you import the `uxsim` module.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import glob, os, csv, time
import pandas as pd
from PIL import Image, ImageDraw, ImageFont
from PIL.Image import Resampling
from tqdm.auto import tqdm
from collections import defaultdict as ddict
from importlib.resources import files
from pathlib import Path
import io
from scipy.sparse.csgraph import floyd_warshall

from .utils import *

def load_font_data(font_path=None):
    if font_path is None:
        font_resource = files('uxsim.files').joinpath('HackGen-Regular.ttf')
        with font_resource.open('rb') as f:
            font_data = f.read()
    else:
        font_file = Path(font_path)
        with font_file.open('rb') as f:
            font_data = f.read()
    
    return font_data

# def load_font_data():
#     font_data = read_binary("uxsim.files", "HackGen-Regular.ttf")
#     return font_data

class Analyzer:
    """
    Class for analyzing and visualizing a simulation result.
    """

    def __init__(s, W, font_pillow=None, font_matplotlib=None):
        """
        Create result analysis object.

        Parameters
        ----------
        W : object
            The world to which this belongs.
        font_pillow : str, optional
            The path to the font file for Pillow. If not provided, the default font for English and Japanese is used.
        font_matplotlib : str, optional
            The font name for Matplotlib. If not provided, the default font for English and Japanese is used.
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

        # visualization data
        s.font_data = load_font_data(font_pillow)
        s.font_file_like = io.BytesIO(s.font_data)
        
        plt.rcParams["font.family"] = get_font_for_matplotlib(font_matplotlib)

    def basic_analysis(s):
        """
        Analyze basic stats.
        """
        df = s.W.analyzer.od_to_pandas()

        s.trip_completed = np.sum(df["completed_trips"])
        s.trip_all = np.sum(df["total_trips"])
        
        s.total_distance_traveled = np.sum(df["average_distance_traveled_per_veh"]*df["total_trips"])

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
        Analyze OD-specific stats: number of trips, number of completed trips, free-flow travel time, average travel time, its std, total distance traveled
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
        s.od_dist = ddict(lambda: [])
        s.od_dist_total = ddict(lambda: 0)
        s.od_dist_ave = ddict(lambda: 0)
        s.od_dist_std = ddict(lambda: 0)
        s.od_dist_min = ddict(lambda: 0)
        dn = s.W.DELTAN

        #自由旅行時間と最短距離
        adj_mat_time = np.zeros([len(s.W.NODES), len(s.W.NODES)])
        adj_mat_dist = np.zeros([len(s.W.NODES), len(s.W.NODES)])
        for link in s.W.LINKS:
            i = link.start_node.id
            j = link.end_node.id
            if s.W.ADJ_MAT[i,j]:
                adj_mat_time[i,j] = link.length/link.u
                adj_mat_dist[i,j] = link.length
                if link.capacity_in == 0: #流入禁止の場合は通行不可
                    adj_mat_time[i,j] = np.inf
                    adj_mat_dist[i,j] = np.inf
            else:
                adj_mat_time[i,j] = np.inf
                adj_mat_dist[i,j] = np.inf
        dist_time = floyd_warshall(adj_mat_time)
        dist_space = floyd_warshall(adj_mat_dist)

        for veh in s.W.VEHICLES.values():
            o = veh.orig
            d = veh.dest
            if d != None:
                s.od_trips[o,d] += dn

                veh_links = [rec[1] for rec in veh.log_t_link if hasattr(rec[1], "length")]
                veh_dist_traveled = sum([l.length for l in veh_links])
                if veh.state == "run":
                    veh_dist_traveled += veh.x
                veh.distance_traveled = veh_dist_traveled
                s.od_dist[o,d].append(veh.distance_traveled)

                if veh.travel_time != -1:
                    s.od_trips_comp[o,d] += dn
                    s.od_tt[o,d].append(veh.travel_time)
        for o,d in s.od_tt.keys():
            s.od_tt_ave[o,d] = np.average(s.od_tt[o,d])
            s.od_tt_std[o,d] = np.std(s.od_tt[o,d])
            s.od_tt_free[o,d] = dist_time[o.id, d.id]
            s.od_dist_total[o,d] = np.sum(s.od_dist[o,d])
            s.od_dist_min[o,d] = dist_space[o.id, d.id]
            s.od_dist_ave[o,d] = np.average(s.od_dist[o,d])
            s.od_dist_std[o,d] = np.std(s.od_dist[o,d])

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
        if s.W.vehicle_logging_timestep_interval != 1:
            warnings.warn("vehicle_logging_timestep_interval is not 1. The trajecotries are not exactly accurate.", LoggingWarning)

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
                        l.ls.append(veh.log_lane[i])
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
        s.W.print(" number of completed trips:\t", s.trip_completed, "/", s.trip_all)
        #s.W.print(" number of completed trips:\t", s.trip_completed, "/", len(s.W.VEHICLES)*s.W.DELTAN)
        if s.trip_completed > 0:
            s.W.print(f" total travel time:\t\t {s.total_travel_time:.1f} s")
            s.W.print(f" average travel time of trips:\t {s.average_travel_time:.1f} s")
            s.W.print(f" average delay of trips:\t {s.average_delay:.1f} s")
            s.W.print(f" delay ratio:\t\t\t {s.average_delay/s.average_travel_time:.3f}")
        s.W.print(f" total distance traveled:\t {s.total_distance_traveled:.1f} m")

        if force_print == 1 and s.W.print_mode == 0:
            print("results:")
            print(f" average speed:\t {s.average_speed:.1f} m/s")
            print(" number of completed trips:\t", s.trip_completed, "/", s.trip_all)
            #print(" number of completed trips:\t", s.trip_completed, "/", len(s.W.VEHICLES)*s.W.DELTAN)
            if s.trip_completed > 0:
                print(f" total travel time:\t\t {s.total_travel_time:.1f} s")
                print(f" average travel time of trips:\t {s.average_travel_time:.1f} s")
                print(f" average delay of trips:\t {s.average_delay:.1f} s")
                print(f" delay ratio:\t\t\t {s.average_delay/s.average_travel_time:.3f}")
            print(f" total distance traveled:\t {s.total_distance_traveled:.1f} m")


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
        if s.W.vehicle_logging_timestep_interval != 1:
            warnings.warn("vehicle_logging_timestep_interval is not 1. The plot is not exactly accurate.", LoggingWarning)

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

        for lll in links:
            s.time_space_diagram_traj_links(linkslist=[lll], figsize=figsize, plot_signal=plot_signal, xlim=xlim, ylim=ylim)
    
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
        if s.W.vehicle_logging_timestep_interval != 1:
            warnings.warn("vehicle_logging_timestep_interval is not 1. The plot is not exactly accurate.", LoggingWarning)

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
        if s.W.vehicle_logging_timestep_interval != 1:
            warnings.warn("vehicle_logging_timestep_interval is not 1. The plot is not exactly accurate.", LoggingWarning)
            
        #複数リンクの連続した車両軌跡の時空間図
        s.W.print(" drawing trajectories...")
        s.compute_accurate_traj()

        #リンクリストのリストであればそのまま，そうでなければリスト化
        try:
            iter(linkslist[0])
            if type(linkslist[0]) == str:
                linkslist = [linkslist]
        except TypeError:
            linkslist = [linkslist]

        for links in linkslist:
            linkdict = dict()
            d = 0
            for ll in links:
                l = s.W.get_link(ll)
                linkdict[l] = d
                d += l.length

            plt.figure(figsize=figsize)
            for ll in links:
                l = s.W.get_link(ll)
                for i in range(len(l.xss)):
                    lane_shift = l.ls[i]/l.number_of_lanes*s.W.DELTAT/2 #vehicle with the same lane is plotted slightly shifted
                    plt.plot(np.array(l.tss[i])+lane_shift, np.array(l.xss[i])+linkdict[l], "-", c=l.cs[i], lw=0.5)
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
                if len(links) == 1:
                    plt.savefig(f"out{s.W.name}/tsd_traj_{s.W.get_link(links[0]).name}.png")
                else:
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
    def network(s, t=None, detailed=1, state_variables="density_speed", minwidth=0.5, maxwidth=12, left_handed=1, tmp_anim=0, figsize=(6,6), network_font_size=12, node_size=2, legend=True, legend_outside=False):
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
        state_variables : str, optional
            Traffic state variables to be visualized. Default is "density_speed".
            The other option is "flow_delay". Anything other than "density_speed" is considered as "flow_delay" mode.
        minwidth : float, optional
            The minimum width of the link visualization. Default is 0.5.
        maxwidth : float, optional
            The maximum width of the link per lane visualization. Default is 12.
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
        legend : bool, optional
            If set to True, the legend will be displayed. Default is True.  
        legend_outside : bool, optional
            If set to True, the legend will be placed outside the plot. Default is False.

        Notes
        -----
        This method visualizes the entire transportation network and its current traffic conditions.
        The visualization provides information on vehicle density, velocity, link names, node locations, and more.
        The plots are saved to the directory `out<W.name>` with filenames depending on the `detailed` and `t` parameters.

        In the default mode (`state_variables="density_speed"`), the color of the links represents the traffic speed (lighter colors indicate higher speeds), and the width of the links represents the traffic density (thicker links indicate higher densities).Although this combination of density and speed is intuitive, they are strongly correlated, so it is not very informative. Thus alternatively, with `state_variables="flow_delay"` mode, the color of the links represents the traffic speed (lighter colors indicate higher speeds), and the width of the links represents the traffic flow (thicker links indicate higher flows).
        Specific meaning of the colors (truncated "jet" colormap):

        - dark blue: free-flow (delay=free_flow_speed/speed-1 < 10%)
        - red: very congested (delay > 90%)
        """
        s.compute_edie_state()

        plt.figure(figsize=figsize, facecolor='white')
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
                    if state_variables == "density_speed":
                        try:
                            k = l.k_mat[int(t/l.edie_dt), i+1]
                            v = l.v_mat[int(t/l.edie_dt), i+1]
                        except:
                            warnings.warn(f"invalid time {t} is specified for network visualization", UserWarning)
                            return -1
                        lw[i] = k*l.delta*(maxwidth*l.number_of_lanes-minwidth)+minwidth
                        c[i] = plt.colormaps["viridis"](v/l.u)
                    else: #"flow_delay" mode
                        try:
                            q = l.q_mat[int(t/l.edie_dt), i+1]
                            v = l.v_mat[int(t/l.edie_dt), i+1]
                        except:
                            warnings.warn(f"invalid time {t} is specified for network visualization", UserWarning)
                            return -1
                        lw[i] = q/l.capacity*(maxwidth*l.number_of_lanes-minwidth)+minwidth
                        pace_ratio = l.u/v
                        pace_max = 3.0
                        pace_min = 1.0
                        color_coef = (pace_ratio-pace_min)/(pace_max-pace_min)
                        if color_coef < 0.1:    #delay_ratio < 20%
                            color_coef = 0.1
                        if color_coef > 0.9:    #delay_ratio > 180%
                            color_coef = 0.9

                        c[i] = plt.colormaps["jet"](color_coef)

                xmid = [((xsize-i)*x1+(i+1)*x2)/(xsize+1)+vx for i in range(xsize)]
                ymid = [((xsize-i)*y1+(i+1)*y2)/(xsize+1)+vy for i in range(xsize)]
                plt.plot([x1]+xmid+[x2], [y1]+ymid+[y2], "k--", lw=0.25, zorder=5)
                for i in range(xsize-1):
                    plt.plot([xmid[i], xmid[i+1]], [ymid[i], ymid[i+1]], c=c[i], lw=lw[i], zorder=6)
                if network_font_size > 0:
                    plt.text(xmid[int(len(xmid)/2)], ymid[int(len(xmid)/2)], l.name, c="b", zorder=20, fontsize=network_font_size)
            else:
                #簡略モード
                if state_variables == "density_speed":
                    k = (l.cum_arrival[int(t/s.W.DELTAT)]-l.cum_departure[int(t/s.W.DELTAT)])/l.length
                    v = l.length/l.traveltime_instant[int(t/s.W.DELTAT)]
                    width = k*l.delta*(maxwidth*l.number_of_lanes-minwidth)+minwidth
                    c = plt.colormaps["viridis"](v/l.u)
                else: #"flow_delay" mode
                    k = (l.cum_arrival[int(t/s.W.DELTAT)]-l.cum_departure[int(t/s.W.DELTAT)])/l.length
                    v = l.length/l.traveltime_instant[int(t/s.W.DELTAT)]
                    q = k*v
                    width = q/l.capacity*(maxwidth*l.number_of_lanes-minwidth)+minwidth
                    pace_ratio = l.u/v
                                
                    pace_max = 2.0
                    pace_min = 1.0
                    color_coef = (pace_ratio-pace_min)/(pace_max-pace_min)
                    if color_coef < 0.1:    #delay_ratio < 10%
                        color_coef = 0.1
                    if color_coef > 0.9:    #delay_ratio > 90%
                        color_coef = 0.9

                    c = plt.colormaps["jet"](color_coef)

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
        
        if legend:
            # ヘッダー用のdummyアーティスト（凡例上はテキストだけを表示）
            if state_variables == "density_speed":
                dummy_speed = Line2D([], [], linestyle='', color='none', label="Speed")
                dummy_density = Line2D([], [], linestyle='', color='none', label="Density")

                speed_handles = [
                    Line2D([0], [0], color=plt.colormaps["viridis"](0.0), lw=(minwidth+maxwidth)/2, solid_capstyle="butt"),
                    Line2D([0], [0], color=plt.colormaps["viridis"](1.0), lw=(minwidth+maxwidth)/2, solid_capstyle="butt")
                ]
                speed_labels = ["0", "max"]

                density_handles = [
                    Line2D([0], [0], color='black', lw=minwidth, solid_capstyle="butt"),
                    Line2D([0], [0], color='black', lw=maxwidth, solid_capstyle="butt")
                ]
                density_labels = ["0", "max (1lane)"]

                handles = [dummy_speed] + speed_handles + [dummy_density] + density_handles
                labels  = [dummy_speed.get_label()] + speed_labels + [dummy_density.get_label()] + density_labels
            else:
                dummy_speed = Line2D([], [], linestyle='', color='none', label="Delay")
                dummy_volume = Line2D([], [], linestyle='', color='none', label="Flow")

                speed_handles = [
                    Line2D([0], [0], color=plt.colormaps["jet"](0.1), lw=(minwidth+maxwidth)/2, solid_capstyle="butt"),
                    Line2D([0], [0], color=plt.colormaps["jet"](0.5), lw=(minwidth+maxwidth)/2, solid_capstyle="butt"),
                    Line2D([0], [0], color=plt.colormaps["jet"](0.9), lw=(minwidth+maxwidth)/2, solid_capstyle="butt"),
                ]
                speed_labels = ["< 10%", "= 50%", "> 90%"]

                volume_handles = [
                    Line2D([0], [0], color='black', lw=minwidth, solid_capstyle="butt"),
                    Line2D([0], [0], color='black', lw=maxwidth, solid_capstyle="butt")
                ]
                volume_labels = ["0", "max"]

                handles = [dummy_speed] + speed_handles + [dummy_volume] + volume_handles
                labels  = [dummy_speed.get_label()] + speed_labels + [dummy_volume.get_label()] + volume_labels


            if not legend_outside:
                plt.legend(handles, labels, ncol=1, handlelength=2, columnspacing=1.0, loc='best', frameon=True)
            else:
                plt.legend(handles, labels, ncol=1, handlelength=2, columnspacing=1.0, frameon=True, loc='center left', bbox_to_anchor=(1, 0.5))



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
    def network_average(s, minwidth=0.5, maxwidth=12, left_handed=1, figsize=(6,6), network_font_size=12, node_size=2, legend=True, legend_outside=False):
        """
        Visualizes the average traffic conditions of the network.
        This function generates a network visualization where links are colored based on congestion levels (travel time ratio) and sized according to traffic volume.

        Parameters
        ----------
        s : Simulator
            Simulator object containing the network and simulation results.
        minwidth : float, optional
            Minimum width of links in the visualization. Default is 0.5.
        maxwidth : float, optional
            Maximum width of links in the visualization. Default is 12.
        left_handed : int, optional
            If 1, offsets the links to the left side of the road. If 0, to the right. Default is 1.
        figsize : tuple, optional
            Size of the figure (width, height) in inches. Default is (6, 6).
        network_font_size : int, optional
            Font size for node and link labels. If 0, no labels are shown. Default is 4.
        node_size : int, optional
            Size of the nodes in the visualization. Default is 2.
        legend : bool, optional
            Whether to show the legend. Default is True.
        legend_outside : bool, optional
            If True, places the legend outside the plot. Default is False.

        Returns
        -------
        None
            Saves the visualization to a file if save_mode is True and/or displays it if show_mode is True.

        Notes
        -----
        Links are colored according to the delay ratio as follows (using truncated "jet" colormap):

        - dark blue: free-flow (delay=free_flow_speed/speed-1 < 10%)
        - red: very congested (delay > 90%)
        """
        df = s.link_to_pandas()

        plt.figure(figsize=figsize)
        plt.subplot(111, aspect="equal")
        for n in s.W.NODES:
            plt.plot(n.x, n.y, "ko", ms=node_size, zorder=10)
            if network_font_size > 0:
                plt.text(n.x, n.y, n.name, c="g", horizontalalignment="center", verticalalignment="top", zorder=20, fontsize=network_font_size)
        volume_max = np.max(df["traffic_volume"])
        for l in s.W.LINKS:
            x1, y1 = l.start_node.x, l.start_node.y
            x2, y2 = l.end_node.x, l.end_node.y
            vx, vy = (y1-y2)*0.05, (x2-x1)*0.05
            if not left_handed:
                vx, vy = -vx, -vy
            
            volume = df[df["link"]==l.name]["traffic_volume"].values[0]
            pace_ratio = df[df["link"]==l.name]["average_travel_time"].values[0]/df[df["link"]==l.name]["free_travel_time"].values[0]

            width = volume/volume_max*maxwidth + minwidth
            
            pace_max = 2.0
            pace_min = 1.0
            color_coef = (pace_ratio-pace_min)/(pace_max-pace_min)
            if color_coef < 0.1:    #delay_ratio < 10%
                color_coef = 0.1
            if color_coef > 0.9:    #delay_ratio > 90%
                color_coef = 0.9

            c = plt.colormaps["jet"](color_coef)

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
        buffxy = max([(maxx-minx)/10, (maxy-miny)/10])
        plt.xlim([minx-buffxy, maxx+buffxy])
        plt.ylim([miny-buffxy, maxy+buffxy])

        if legend:
            # ヘッダー用のdummyアーティスト（凡例上はテキストだけを表示）
            dummy_speed = Line2D([], [], linestyle='', color='none', label="Delay")
            dummy_volume = Line2D([], [], linestyle='', color='none', label="Flow")

            speed_handles = [
                Line2D([0], [0], color=plt.colormaps["jet"](0.1), lw=(minwidth+maxwidth)/2, solid_capstyle="butt"),
                Line2D([0], [0], color=plt.colormaps["jet"](0.5), lw=(minwidth+maxwidth)/2, solid_capstyle="butt"),
                Line2D([0], [0], color=plt.colormaps["jet"](0.9), lw=(minwidth+maxwidth)/2, solid_capstyle="butt"),
            ]
            speed_labels = ["< 10%", "= 50%", "> 90%"]

            volume_handles = [
                Line2D([0], [0], color='black', lw=minwidth, solid_capstyle="butt"),
                Line2D([0], [0], color='black', lw=maxwidth, solid_capstyle="butt")
            ]
            volume_labels = ["0", "max"]

            handles = [dummy_speed] + speed_handles + [dummy_volume] + volume_handles
            labels  = [dummy_speed.get_label()] + speed_labels + [dummy_volume.get_label()] + volume_labels

            if not legend_outside:
                plt.legend(handles, labels, ncol=1, handlelength=2, columnspacing=1.0, loc='best', frameon=True)
            else:
                plt.legend(handles, labels, ncol=1, handlelength=2, columnspacing=1.0, frameon=True, loc='center left', bbox_to_anchor=(1, 0.5))


        plt.tight_layout()

        if s.W.save_mode:
            plt.savefig(f"out{s.W.name}/network_average.png")
        if s.W.show_mode:
            plt.show()
        else:
            plt.close("all")

    @catch_exceptions_and_warn()
    def network_pillow(s, t=None, detailed=1, state_variables="density_speed", minwidth=0.5, maxwidth=12, left_handed=1, tmp_anim=0, figsize=6, network_font_size=20, node_size=2, image_return=0, legend=True):
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
        state_variables : str, optional
            Traffic state variables to be visualized. Default is "density_speed".
            The other option are "flow_delay" and "density_flow". Anything other than "density_speed" or "density_flow" is considered as "flow_delay" mode.
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
        image_return : int, optional
            If set to 1, the function returns the image. Default is 0.
        legend : bool, optional
            If set to True, the legend will be displayed. Default is True.

        Notes
        -----
        This method visualizes the entire transportation network and its current traffic conditions.
        The visualization provides information on vehicle density, velocity, link names, node locations, and more.
        The plots are saved to the directory `out<W.name>` with filenames depending on the `detailed` and `t` parameters.
        
        In the default mode (`state_variables="density_speed"`), the color of the links represents the traffic speed (lighter colors indicate higher speeds), and the width of the links represents the traffic density (thicker links indicate higher densities).Although this combination of density and speed is intuitive, they are strongly correlated, so it is not very informative. Thus alternatively, with `state_variables="flow_delay"` mode, the color of the links represents the traffic speed (lighter colors indicate higher speeds), and the width of the links represents the traffic flow (thicker links indicate higher flows).
        Specific meaning of the colors (truncated "jet" colormap):

        - dark blue: free-flow (delay=free_flow_speed/speed-1 < 10%)
        - red: very congested (delay > 90%)

        "density_flow" is an experimental mode. Although reasonable, it is often difficult to recognize.
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
        
        lypad = 0
        if legend:
            lypad = buffer*1.5
            miny -= lypad
        img = Image.new("RGBA", (int(maxx-minx), int(maxy-miny)), (255, 255, 255, 255))
        draw = ImageDraw.Draw(img)
        
        font_data = load_font_data()
        font_file_like = io.BytesIO(font_data)
        if network_font_size > 0:
            font = ImageFont.truetype(font_file_like, int(network_font_size))

        def flip(y):
            return img.size[1]-y

        for l in s.W.LINKS:
            x1, y1 = l.start_node.x*coef-minx, l.start_node.y*coef-miny
            x2, y2 = l.end_node.x*coef-minx, l.end_node.y*coef-miny
            vx, vy = (y1-y2)*0.05, (x2-x1)*0.05
            if not left_handed:
                vx, vy = -vx, -vy
            if state_variables == "density_speed":
                k = (l.cum_arrival[int(t/s.W.DELTAT)]-l.cum_departure[int(t/s.W.DELTAT)])/l.length
                v = l.length/l.traveltime_instant[int(t/s.W.DELTAT)]
                width = k*l.delta*(maxwidth-minwidth)+minwidth
                c = plt.colormaps["viridis"](v/l.u)
            elif state_variables == "density_flow":
                k = (l.cum_arrival[int(t/s.W.DELTAT)]-l.cum_departure[int(t/s.W.DELTAT)])/l.length
                v = l.length/l.traveltime_instant[int(t/s.W.DELTAT)]
                width = k*l.delta*(maxwidth-minwidth)+minwidth
                q = k*v/l.capacity
                if q < 0:
                    q = 0
                if q > 1:
                    q = 1
                c = plt.colormaps["magma"](q)
            else: #"flow_delay" mode
                k = (l.cum_arrival[int(t/s.W.DELTAT)]-l.cum_departure[int(t/s.W.DELTAT)])/l.length
                v = l.length/l.traveltime_instant[int(t/s.W.DELTAT)]
                q = k*v
                width = q/l.capacity*(maxwidth-minwidth)+minwidth

                pace_ratio = l.u/v    #pace (inverse of speed) is more intuitive
                pace_max = 2.0
                pace_min = 1.0
                color_coef = (pace_ratio-pace_min)/(pace_max-pace_min)
                if color_coef < 0.1:    #delay_ratio < 10%
                    color_coef = 0.1
                if color_coef > 0.9:    #delay_ratio > 90%
                    color_coef = 0.9

                c = plt.colormaps["jet"](color_coef)

                
            xmid1, ymid1 = (2*x1+x2)/3+vx, (2*y1+y2)/3+vy
            xmid2, ymid2 = (x1+2*x2)/3+vx, (y1+2*y2)/3+vy
            draw.line([(x1, flip(y1)), (xmid1, flip(ymid1)), (xmid2, flip(ymid2)), (x2, flip(y2))], fill=(int(c[0]*255), int(c[1]*255), int(c[2]*255)), width=int(width), joint="curve")

            if network_font_size > 0:
                draw.text((xmid1, flip(ymid1)), l.name, font=font, fill="blue", anchor="mm")

        for n in s.W.NODES:
            if network_font_size > 0:
                draw.text(((n.x)*coef-minx, flip((n.y)*coef-miny)), n.name, font=font, fill="green", anchor="mm")
                draw.text(((n.x)*coef-minx, flip((n.y)*coef-miny)), n.name, font=font, fill="green", anchor="mm")

        font_data = load_font_data()
        font_file_like = io.BytesIO(font_data)
        font = ImageFont.truetype(font_file_like, int(30))
        draw.text((img.size[0]/2,20), f"t = {t :>8} (s)", font=font, fill="black", anchor="mm")

        if legend:
            
            lx00 = (maxx-minx)*0.25
            lx01 = (maxx-minx)*0.35
            lx10 = (maxx-minx)*0.65
            lx11 = (maxx-minx)*0.75
            ly0 = -buffer-miny
            ly1 = -buffer-lypad*0.35-miny
            ly2 = -buffer-lypad*0.7-miny

            lny = flip(-buffer+lypad*0.2-miny)
            lsy = flip(-buffer-lypad*0.9-miny)
            lex = (maxx-minx)*0.15
            lwx = (maxx-minx)*0.87
            
            if state_variables == "density_speed":
                c1 = tuple(int(c*255) for c in plt.colormaps["viridis"](1.0))[:3]
                c2 = tuple(int(c*255) for c in plt.colormaps["viridis"](0.0))[:3]

                draw.text(((lx00+lx01)/2, flip(ly0)), "color: speed", font=font, fill="black", anchor="mm")
                draw.line([(lx00, flip(ly1)), (lx01, flip(ly1))], fill=c1, width=int((maxwidth-minwidth)/2))
                draw.line([(lx00, flip(ly2)), (lx01, flip(ly2))], fill=c2, width=int((maxwidth-minwidth)/2))            
                draw.text((lx01+10, flip(ly1)), "max", font=font, fill="black", anchor="lm")
                draw.text((lx01+10, flip(ly2)), "0", font=font, fill="black", anchor="lm")

                draw.text(((lx10+lx11)/2, flip(ly0)), "width: density", font=font, fill="black", anchor="mm")    
                draw.line([(lx10, flip(ly1)), (lx11, flip(ly1))], fill="black", width=int(minwidth))
                draw.line([(lx10, flip(ly2)), (lx11, flip(ly2))], fill="black", width=int(maxwidth))            
                draw.text((lx11+10, flip(ly1)), "0", font=font, fill="black", anchor="lm")
                draw.text((lx11+10, flip(ly2)), "max", font=font, fill="black", anchor="lm")

            elif state_variables == "density_flow":
                c1 = tuple(int(c*255) for c in plt.colormaps["magma"](1.0))[:3]
                c2 = tuple(int(c*255) for c in plt.colormaps["magma"](0.0))[:3]

                draw.text(((lx00+lx01)/2, flip(ly0)), "color: flow", font=font, fill="black", anchor="mm")
                draw.line([(lx00, flip(ly1)), (lx01, flip(ly1))], fill=c1, width=int((maxwidth-minwidth)/2))
                draw.line([(lx00, flip(ly2)), (lx01, flip(ly2))], fill=c2, width=int((maxwidth-minwidth)/2))            
                draw.text((lx01+10, flip(ly1)), "max", font=font, fill="black", anchor="lm")
                draw.text((lx01+10, flip(ly2)), "0", font=font, fill="black", anchor="lm")

                draw.text(((lx10+lx11)/2, flip(ly0)), "width: density", font=font, fill="black", anchor="mm")    
                draw.line([(lx10, flip(ly1)), (lx11, flip(ly1))], fill="black", width=int(minwidth))
                draw.line([(lx10, flip(ly2)), (lx11, flip(ly2))], fill="black", width=int(maxwidth))            
                draw.text((lx11+10, flip(ly1)), "0", font=font, fill="black", anchor="lm")
                draw.text((lx11+10, flip(ly2)), "max", font=font, fill="black", anchor="lm")

            
            else:    #"flow_delay" mode
                c1 = tuple(int(c*255) for c in plt.colormaps["jet"](0.1))[:3]
                c2 = tuple(int(c*255) for c in plt.colormaps["jet"](0.9))[:3]

                draw.text(((lx00+lx01)/2, flip(ly0)), "color: delay", font=font, fill="black", anchor="mm")
                draw.line([(lx00, flip(ly1)), (lx01, flip(ly1))], fill=c1, width=int((maxwidth-minwidth)/2))
                draw.line([(lx00, flip(ly2)), (lx01, flip(ly2))], fill=c2, width=int((maxwidth-minwidth)/2))            
                draw.text((lx01+10, flip(ly1)), "< 10%", font=font, fill="black", anchor="lm")
                draw.text((lx01+10, flip(ly2)), "> 90%", font=font, fill="black", anchor="lm")

                draw.text(((lx10+lx11)/2, flip(ly0)), "width: flow", font=font, fill="black", anchor="mm")    
                draw.line([(lx10, flip(ly1)), (lx11, flip(ly1))], fill="black", width=int(minwidth))
                draw.line([(lx10, flip(ly2)), (lx11, flip(ly2))], fill="black", width=int(maxwidth))            
                draw.text((lx11+10, flip(ly1)), "0", font=font, fill="black", anchor="lm")
                draw.text((lx11+10, flip(ly2)), "max", font=font, fill="black", anchor="lm")
            draw.line([(lwx, lny), (lex, lny), (lex, lsy), (lwx, lsy), (lwx, lny)], fill="black", width=1)

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
    def network_anim(s, animation_speed_inverse=10, detailed=0, state_variables="density_speed", minwidth=0.5, maxwidth=12, left_handed=1, figsize=(6,6), node_size=2, network_font_size=20, timestep_skip=24, file_name=None, legend=True):
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
        state_variables : str, optional
            Traffic state variables to be visualized. Default is "density_speed".
            The other option is "flow_delay". Anything other than "density_speed" is considered as "flow_delay" mode.
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
        file_name : str, optional
            The name of the file to which the animation is saved. It overrides the defauld name. Default is None.
        legend : bool, optional
            If set to True, the legend will be displayed. Default is True.  

        Notes
        -----
        This method generates an animation visualizing the entire transportation network and its traffic conditions over time.
        The animation provides information on vehicle density, velocity, link names, node locations, and more.
        The generated animation is saved to the directory `out<W.name>` with a filename based on the `detailed` parameter.

        Temporary images used to create the animation are removed after the animation is generated.
        
        In the default mode (`state_variables="density_speed"`), the color of the links represents the traffic speed (lighter colors indicate higher speeds), and the width of the links represents the traffic density (thicker links indicate higher densities).Although this combination of density and speed is intuitive, they are strongly correlated, so it is not very informative. Thus alternatively, with `state_variables="flow_delay"` mode, the color of the links represents the traffic speed (lighter colors indicate higher speeds), and the width of the links represents the traffic flow (thicker links indicate higher flows).
        Specific meaning of the colors (truncated "jet" colormap):

        - dark blue: free-flow (delay=free_flow_speed/speed-1 < 10%)
        - red: very congested (delay > 90%)
        """
        s.W.print(" generating animation...")
        pics = []
        for t in tqdm(range(0, s.W.TMAX, s.W.DELTAT*timestep_skip), disable=(s.W.print_mode==0)):
            if int(t/s.W.LINKS[0].edie_dt) < s.W.LINKS[0].k_mat.shape[0]:
                if detailed:
                    #todo_later: 今後はこちらもpillowにする
                    s.network(int(t), detailed=detailed, state_variables=state_variables, minwidth=minwidth, maxwidth=maxwidth, left_handed=left_handed, tmp_anim=1, figsize=figsize, node_size=node_size, network_font_size=network_font_size, legend=legend)
                    pics.append(Image.open(f"out{s.W.name}/tmp_anim_{t}.png"))
                else:
                    img_ret = s.network_pillow(int(t), detailed=detailed, state_variables=state_variables, minwidth=minwidth, maxwidth=maxwidth, left_handed=left_handed, tmp_anim=1, figsize=figsize, node_size=node_size, network_font_size=network_font_size, image_return=True, legend=legend)
                    pics.append(img_ret)
                
        fname = f"out{s.W.name}/anim_network{detailed}.gif"
        if file_name != None:
            fname = file_name
        pics[0].save(fname, save_all=True, append_images=pics[1:], optimize=False, duration=animation_speed_inverse*timestep_skip, loop=0)
        for f in glob.glob(f"out{s.W.name}/tmp_anim_*.png"):
            os.remove(f)

    @catch_exceptions_and_warn()
    def network_fancy(s, animation_speed_inverse=10, figsize=6, sample_ratio=0.3, interval=5, network_font_size=0, trace_length=3, speed_coef=2, file_name=None, antialiasing=True):
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
        file_name : str, optional
            The name of the file to which the animation is saved. It overrides the defauld name. Default is None.
        antialiasing : bool, optional
            If set to True, antialiasing is applied to the animation. Default is True.

        Notes
        -----
        This method generates a visually appealing animation that visualizes vehicles' trajectories across the transportation network over time.
        The animation provides information on vehicle positions, speeds, link names, node locations, and more, with Bezier curves used for smooth transitions.
        The generated animation is saved to the directory `out<W.name>` with a filename `anim_network_fancy.gif`.

        Temporary images used to create the animation are removed after the animation is generated.
        """
        if s.W.vehicle_logging_timestep_interval != 1:
            warnings.warn("vehicle_logging_timestep_interval is not 1. The animation is not exactly accurate.", LoggingWarning)

        s.W.print(" generating animation...")

        # ベジエ補間
        from scipy.interpolate import make_interp_spline

        #{t: ["xs":[], "ys":[], "v": v, "c":c]}
        draw_dict = ddict(lambda: [])

        maxx = max([n.x for n in s.W.NODES])
        minx = min([n.x for n in s.W.NODES])
        dcoef = (maxx-minx)/20

        for veh in s.W.VEHICLES.values():
            if s.W.rng.random() > sample_ratio:
                continue
            ts = []
            xs = []
            ys = []
            vs = []
            dx = (s.W.rng.random()-0.5)*dcoef
            dy = (s.W.rng.random()-0.5)*dcoef
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

        if antialiasing:
            scale = 2
        else:
            scale = 1
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
                
            if network_font_size > 0:
                font = ImageFont.truetype(s.font_file_like, int(network_font_size))

            def flip(y):
                return img.size[1]-y

            for l in s.W.LINKS:
                x1, y1 = l.start_node.x*coef-minx, l.start_node.y*coef-miny
                x2, y2 = l.end_node.x*coef-minx, l.end_node.y*coef-miny
                n_lane = l.number_of_lanes
                draw.line([(x1, flip(y1)), (x2, flip(y2))], fill=(200,200,200), width=int(n_lane*scale), joint="curve")

                if network_font_size > 0:
                    draw.text(((x1+x2)/2, flip((y1+y2)/2)), l.name, font=font, fill="blue", anchor="mm")

            traces = draw_dict[t]
            for trace in traces:
                xs = trace["xs"]*coef-minx
                ys = trace["ys"]*coef-miny
                size = 1.5*(1-trace["v"])*scale
                coords = [(l[0], flip(l[1])) for l in list(np.vstack([xs, ys]).T)]
                try:
                    draw.line(coords,
                            fill=(int(trace["c"][0]*255), int(trace["c"][1]*255), int(trace["c"][2]*255)), width=scale, joint="curve")
                    draw.ellipse((xs[-1]-size, flip(ys[-1])-size, xs[-1]+size, flip(ys[-1])+size), 
                            fill=(int(trace["c"][0]*255), int(trace["c"][1]*255), int(trace["c"][2]*255)))
                except:
                    pass
                #draw.line([(x1, flip(y1)), (xmid1, flip(ymid1)), (xmid2, flip(ymid2)), (x2, flip(y2))]

            font_file_like = io.BytesIO(s.font_data) #for unknown reason, s.font_file_like cannot be resued
            font = ImageFont.truetype(font_file_like, int(30))
            draw.text((img.size[0]/2,20), f"t = {t :>8} (s)", font=font, fill="black", anchor="mm")

            if antialiasing:
                img = img.resize((int((maxx-minx)/scale), int((maxy-miny)/scale)), resample=Resampling.LANCZOS)
                
            byte_stream = io.BytesIO()
            img.save(byte_stream, format='PNG')
            byte_stream.seek(0)
            pics.append(Image.open(byte_stream))

            #img.save(f"out{s.W.name}/tmp_anim_{t}.png")
            #pics.append(Image.open(f"out{s.W.name}/tmp_anim_{t}.png"))

        fname = f"out{s.W.name}/anim_network_fancy.gif"
        if file_name != None:
            fname = file_name
        pics[0].save(fname, save_all=True, append_images=pics[1:], optimize=False, duration=animation_speed_inverse*speed_coef, loop=0)

        # for f in glob.glob(f"out{s.W.name}/tmp_anim_*.png"):
        #     os.remove(f)

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
        if s.W.vehicle_logging_timestep_interval != 1:
            warnings.warn("vehicle_logging_timestep_interval is not 1. The plot is not exactly accurate.", LoggingWarning)
        
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


    @catch_exceptions_and_warn()
    def plot_vehicles_log(s, vehnamelist):
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
        if s.W.vehicle_logging_timestep_interval != 1:
            warnings.warn("vehicle_logging_timestep_interval is not 1. The plot is not exactly accurate.", LoggingWarning)
        
        vehs = [s.W.VEHICLES[vehname] for vehname in vehnamelist]

        plt.figure()
        for veh in vehs:
            vehlinks = [str(l.name) if l != -1 else "not in network" for l in veh.log_link]
            plt.plot([veh.log_t[i] for i in lange(veh.log_t) if veh.log_state[i] != "home"], [vehlinks[i] for i in lange(vehlinks) if veh.log_state[i] != "home"], c=veh.color, label=veh.name)
        plt.grid()
        plt.ylabel('link')
        plt.legend()
        plt.ylim([0, None])
        plt.tight_layout()

        if s.W.save_mode:
            plt.savefig(f"out{s.W.name}/vehicles_{vehnamelist}.png")
        if s.W.show_mode:
            plt.show()
        else:
            plt.close("all")


    def vehicles_to_pandas(s):
        """
        Compute the detailed vehicle travel logs and return as a pandas DataFrame.

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
        if s.W.vehicle_logging_timestep_interval != 1:
            warnings.warn("vehicle_logging_timestep_interval is not 1. The output data is not exactly accurate.", LoggingWarning)

        if s.flag_pandas_convert == 0:
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
                        veh_dest_name = None
                        if veh.dest != None:
                            veh_dest_name = veh.dest.name
                        out.append([veh.name, s.W.DELTAN, veh.orig.name, veh_dest_name, veh.log_t[i], linkname, veh.log_x[i], veh.log_s[i], veh.log_v[i]])
            s.df_vehicles = pd.DataFrame(out[1:], columns=out[0])

            s.flag_pandas_convert = 1
        return s.df_vehicles

    def log_vehicles_to_pandas(s):
        """
        same to `vehicles_to_pandas`, just for backward compatibility
        """
        return s.vehicles_to_pandas()

    def vehicle_trip_to_pandas(s):
        """
        Compute the vehicle trip summary and return as a pandas DataFrame.

        Returns
        -------
        pd.DataFrame
            A DataFrame containing the trip summary of the vehicle trip logs, with the columns:
            
            - 'name': the name of the vehicle (platoon).
            - 'orig': the origin node of the vehicle's trip.
            - 'dest': the destination node of the vehicle's trip.
            - 'departure_time': the departure time of the vehicle.
            - 'final_state': the final state of the vehicle.
            - 'travel_time': the travel time of the vehicle.
            - 'average_speed': the average speed of the vehicle.
            - 'distance_traveled': the distance traveled by the vehicle.
        """
        out = [["name", "orig", "dest", "departure_time", "final_state", "travel_time", "average_speed", "distance_traveled"]]
        for veh in s.W.VEHICLES.values():
            veh_dest_name = veh.dest.name if veh.dest != None else None
            veh_state = veh.log_state[-1]
            veh_ave_speed = np.average([v for v in veh.log_v if v != -1])
            veh_dist_traveled = veh.distance_traveled

            out.append([veh.name, veh.orig.name, veh_dest_name, veh.departure_time*s.W.DELTAT, veh_state, veh.travel_time, veh_ave_speed, veh_dist_traveled])
        
        s.df_vehicle_trip = pd.DataFrame(out[1:], columns=out[0])
        return s.df_vehicle_trip

    def gps_like_log_to_pandas(s):
        """
        Generate GPS-like log (x and y in the coordinate system used for Node) of vehicles and return as a pandas DataFrame.

        Returns
        -------
        pd.DataFrame
        """
        out = [["name", "t", "x", "y", "v"]]
        for veh in s.W.VEHICLES.values():
            for i,t in enumerate(veh.log_t):
                x, y = veh.get_xy_coords(t)
                if (x, y) == (-1, -1):
                    continue
                v = veh.log_v[i]
                out.append([veh.name, t, x, y, v])
        s.df_gps_like_log = pd.DataFrame(out[1:], columns=out[0])
        return s.df_gps_like_log

    def basic_to_pandas(s):
        """
        Comutes the basic stats and return as a pandas DataFrame.

        Returns
        -------
        pd.DataFrame
        """
        out = [["total_trips", "completed_trips", "total_travel_time", "average_travel_time", "total_delay", "average_delay"], [s.trip_all, s.trip_completed, s.total_travel_time, s.average_travel_time, s.total_delay, s.average_delay]]

        s.df_basic = pd.DataFrame(out[1:], columns=out[0])
        return s.df_basic

    def od_to_pandas(s):
        """
        Compute the OD-specific stats and return as a pandas DataFrame.

        Returns
        -------
        pd.DataFrame
        """

        s.od_analysis()

        out = [["orig", "dest", "total_trips", "completed_trips", "free_travel_time", "average_travel_time", "stddiv_travel_time",  "shortest_distance", "average_distance_traveled_per_veh", "stddiv_distance_traveled_per_veh"]]
        for o,d in s.od_trips.keys():
            out.append([o.name, d.name, s.od_trips[o,d], s.od_trips_comp[o,d], s.od_tt_free[o,d], s.od_tt_ave[o,d], s.od_tt_std[o,d], s.od_dist_min[o,d], s.od_dist_ave[o,d], s.od_dist_std[o,d]])

        s.df_od = pd.DataFrame(out[1:], columns=out[0])
        return s.df_od
    
    def areas2areas_to_pandas(s, areas, area_names=None):
        """
        Compute the area-wise OD-specific stats and return a pandas DataFrame. It analyzes travel stats between areas (set of nodes).
        
        Parameters
        ----------
        areas : list
            The list of areas. Each area is defined as a list of nodes. The items of area can be Node objects or names of Nodes.
        area_names : list, optional
            The list of names of areas.
            
        Returns
        -------
        pd.DataFrame
        """
        df = s.od_to_pandas()

        o_name_rec = []
        d_name_rec = []
        total_trips_rec = []
        completed_trips_rec = []
        average_travel_time_rec = []
        stddiv_travel_time_rec = []
        average_distance_traveled_rec = []
        stddiv_distance_traveled_rec = []

        average_free_travel_time_rec = []
        average_shortest_distance_rec = []

        areas = [[s.W.get_node(n).name for n in area] for area in areas]
        if area_names == None: 
            area_names = [f"area {i} including {areas[i][0]}" for i in range(len(areas))]

        for i, origs in enumerate(areas):
            for j, dests in enumerate(areas):
                o_name = area_names[i]
                d_name = area_names[j]

                # print(o_name, d_name)

                # group by area: average travel time from origs to dests
                rows = df["orig"].isin(origs) & df["dest"].isin(dests)
                total_tripss = np.array(df["total_trips"][rows])
                average_travel_times = np.array(df["average_travel_time"][rows])
                completed_tripss = np.array(df["completed_trips"][rows])
                var_travel_times = np.array(df["stddiv_travel_time"][rows])**2
                distance_traveleds = np.array(df["average_distance_traveled_per_veh"][rows])
                var_distance_traveleds = np.array(df["stddiv_distance_traveled_per_veh"][rows])**2

                free_travel_time_times = np.array(df["free_travel_time"][rows])
                shortest_distances = np.array(df["shortest_distance"][rows])

                # print(f"{total_tripss = }")
                # print(f"{average_travel_times = }")
                # print(f"{completed_tripss = }")
                # print(f"{var_travel_times = }")
                # print(f"{distance_traveleds = }")
                # print(f"{var_distance_traveleds = }")

                total_trips = total_tripss.sum()
                completed_trips = completed_tripss.sum()

                if total_trips:
                    average_travel_time = (completed_tripss*average_travel_times).sum()/completed_trips
                    var_travel_time = (completed_tripss*var_travel_times).sum()/completed_trips    #wrong! there is a correct formula. TODO: implement
                    stddiv_travel_time = np.sqrt(var_travel_time)

                    average_shortest_distance = (total_tripss*shortest_distances).sum()/total_trips
                else:
                    continue
                    # average_travel_time = np.nan
                    # var_travel_time = np.nan
                    # stddiv_travel_time = np.nan
                    # average_shortest_distance = np.nan

                if completed_trips:
                    average_distance_traveled = (total_tripss*distance_traveleds).sum()/total_trips
                    var_distance_traveled = (total_tripss*distance_traveleds).sum()/total_trips    #wrong!
                    stddiv_distance_traveled = np.sqrt(var_distance_traveled)

                    average_free_travel_time = (completed_tripss*free_travel_time_times).sum()/completed_trips
                else:
                    average_distance_traveled = np.nan
                    var_distance_traveled = np.nan
                    stddiv_distance_traveled = np.nan
                    average_free_travel_time = np.nan

                # print(f"{total_trips = }")
                # print(f"{completed_trips = }")
                # print(f"{average_travel_time = }")
                # print(f"{stddiv_travel_time = }")
                # print(f"{average_distance_traveled = }")
                # print(f"{stddiv_distance_traveled = }")

                o_name_rec.append(o_name)
                d_name_rec.append(d_name)
                total_trips_rec.append(total_trips)
                completed_trips_rec.append(completed_trips)
                average_travel_time_rec.append(average_travel_time)
                stddiv_travel_time_rec.append(stddiv_travel_time)
                average_distance_traveled_rec.append(average_distance_traveled)
                stddiv_distance_traveled_rec.append(stddiv_distance_traveled)
                average_free_travel_time_rec.append(average_free_travel_time)
                average_shortest_distance_rec.append(average_shortest_distance)

        out = [["origin_area", "destination_area", "total_trips", "completed_trips", "average_travel_time", "average_free_travel_time", "average_distance_traveled", "average_shortest_distance"]]
        out += [[o_name_rec[i], d_name_rec[i], total_trips_rec[i], completed_trips_rec[i], average_travel_time_rec[i], average_free_travel_time_rec[i], average_distance_traveled_rec[i], average_shortest_distance_rec[i]] for i in range(len(o_name_rec))]
        
        s.df_areas2areas = pd.DataFrame(out[1:], columns=out[0])
        return s.df_areas2areas

    def area_to_pandas(s, areas, area_names=None, border_include=True):
        """
        Compute traffic stats in area and return as pandas.DataFrame.

        Parameters
        ----------
        areas : list
            The list of areas. Each area is defined as a list of nodes. The items of area can be Node objects or names of Nodes.
        area_names : list, optional
            The list of names of areas.
        border_include : bool, optional
            If set to True, the links on the border of the area are included in the analysis. Default is True.

        Returns
        -------
        pd.DataFrame
        """

        # Precompute DataFrames
        df_links = s.W.analyzer.link_to_pandas()
        df_veh_link = s.W.analyzer.vehicles_to_pandas().drop_duplicates(subset=['name', 'link'])

        # Prepare areas as sets for fast lookup
        areas_set = [{s.W.get_node(n).name for n in area} for area in areas]

        # Initialize result lists
        n_links_rec = []
        traffic_volume_rec = []
        vehicles_remain_rec = []
        total_travel_time_rec = []
        average_delay_rec = []
        average_speed_rec = []
        vehicle_density_rec = []

        # Vectorized approach to process all areas at once
        for area_set in areas_set:
            if border_include:
                rows = df_links["start_node"].isin(area_set) | df_links["end_node"].isin(area_set)
            else:
                rows = df_links["start_node"].isin(area_set) & df_links["end_node"].isin(area_set)

            links = df_links.loc[rows, "link"].unique()

            n_links = links.size
            traffic_volume = df_veh_link[df_veh_link["link"].isin(links)]["name"].nunique() * s.W.DELTAN
            vehicles_remain = df_links.loc[rows, "vehicles_remain"].sum()

            if traffic_volume > 0:
                traffic_volume_rows = (
                            df_links.loc[rows, "traffic_volume"] - df_links.loc[rows, "vehicles_remain"]).values
                total_travel_time = np.sum(df_links.loc[rows, "average_travel_time"].values * traffic_volume_rows)
                total_free_time = np.sum(df_links.loc[rows, "free_travel_time"].values * traffic_volume_rows)
                average_delay = max(total_travel_time / total_free_time - 1, 0)

                # Average speed calculation: total distance / total time
                total_distance = np.sum(df_links.loc[rows, "length"].values * traffic_volume_rows)
                average_speed = total_distance / total_travel_time if total_travel_time > 0 else np.nan

                # Vehicle density calculation: total number of vehicles / total link length
                total_link_length = df_links.loc[rows, "length"].sum()
                vehicle_density = traffic_volume / total_link_length if total_link_length > 0 else np.nan
            else:
                total_travel_time = 0
                total_free_time = 0
                average_delay = np.nan
                average_speed = np.nan
                vehicle_density = np.nan

            # Append the results to lists
            n_links_rec.append(n_links)
            traffic_volume_rec.append(traffic_volume)
            vehicles_remain_rec.append(vehicles_remain)
            total_travel_time_rec.append(total_travel_time)
            average_delay_rec.append(average_delay)
            average_speed_rec.append(average_speed)
            vehicle_density_rec.append(vehicle_density)

        # Create DataFrame from the results
        df_result = pd.DataFrame({
            "area": area_names,
            "n_links": n_links_rec,
            "traffic_volume": traffic_volume_rec,
            "vehicles_remain": vehicles_remain_rec,
            "total_travel_time": total_travel_time_rec,
            "average_delay": average_delay_rec,
            "average_speed": average_speed_rec,
            "vehicle_density": vehicle_density_rec,
        })

        s.df_area = df_result
        return s.df_area

    def vehicle_groups_to_pandas(s, groups, group_names=None):
        """
        Computes the stats of vehicle group and return as a pandas DataFrame.

        Parameters
        ----------
        groups : list
            The list of vehicle groups. Each group is defined as a list of vehicle object.
        group_names : list, optional
            The list of names of vehicle groups.
        
        Returns
        -------
        pd.DataFrame
        """
        df_od = s.W.analyzer.od_to_pandas()

        if group_names == None: 
            group_names = [f"group {i} including {groups[0].name}" for i in range(len(groups))]

        total_trip_rec = []
        completed_trip_rec = []
        average_travel_time_rec = []
        average_delay_rec = []
        std_delay_rec = []
        average_traveled_distance_rec = []
        average_detour_rec = []
        std_detour_rec = []
        averae_speed_rec = []
        std_speed_rec = []
        for i, group in enumerate(groups):
            total_trips = 0
            completed_trips = 0
            travel_times = []
            delays = []
            traveled_distances = []
            detours = []
            speeds = []


            for veh in group:

                total_trips += 1
                if veh.state == "end":
                    completed_trips += 1
                    travel_times.append(veh.travel_time)
                    traveled_distances.append(veh.distance_traveled)
                    
                    free_travel_time = df_od["free_travel_time"][(df_od["orig"]==veh.orig.name) & (df_od["dest"]==veh.dest.name)].values[0]
                    shortest_distance = df_od["shortest_distance"][(df_od["orig"]==veh.orig.name) & (df_od["dest"]==veh.dest.name)].values[0]

                    delays.append(veh.travel_time/free_travel_time)
                    detours.append(veh.distance_traveled/shortest_distance)

                    speeds.append(veh.distance_traveled/veh.travel_time)

                #print(f"{group_names[i]=}, {np.average(travel_times)=}, {np.average(traveled_distances)=}, {np.average(delays)=}, {np.average(detours)=}, {np.std(delays)=}, {np.std(detours)=}, {np.average(speeds)}, {np.std(speeds)}")

            total_trip_rec.append(total_trips)
            completed_trip_rec.append(completed_trips)
            if completed_trips > 0:
                average_travel_time_rec.append(np.average(travel_times))
                average_delay_rec.append(np.average(delays))
                std_delay_rec.append(np.std(delays))
                average_traveled_distance_rec.append(np.average(traveled_distances))
                average_detour_rec.append(np.average(detours))
                std_detour_rec.append(np.std(detours))
                averae_speed_rec.append(np.average(speeds))
                std_speed_rec.append(np.std(speeds))
            else:
                average_travel_time_rec.append(np.nan)
                average_delay_rec.append(np.nan)
                std_delay_rec.append(np.nan)
                average_traveled_distance_rec.append(np.nan)
                average_detour_rec.append(np.nan)
                std_detour_rec.append(np.nan)
                averae_speed_rec.append(np.nan)
                std_speed_rec.append(np.nan)

        df = pd.DataFrame({
            "group": group_names,
            "total_trips": total_trip_rec,
            "completed_trips": completed_trip_rec,
            "average_travel_time": average_travel_time_rec,
            "average_delay_ratio": average_delay_rec,
            "std_delay_ratio": std_delay_rec,
            "average_traveled_distance": average_traveled_distance_rec,
            "average_detour_ratio": average_detour_rec,
            "std_detour_ratio": std_detour_rec,
            "average_speed": averae_speed_rec,
            "std_speed": std_speed_rec,
        })

        s.df_vehicle_groups = df
        
        return s.df_vehicle_groups

    def mfd_to_pandas(s, links=None):
        """
        Compute the MFD-like stats and return as a pandas DataFrame.

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

        out = [["link", "start_node", "end_node", "traffic_volume", "vehicles_remain", "free_travel_time", "average_travel_time", "stddiv_travel_time", "delay_ratio", "length"]]
        for l in s.W.LINKS:
            out.append([l.name, l.start_node.name, l.end_node.name, s.linkc_volume[l], s.linkc_remain[l], s.linkc_tt_free[l], s.linkc_tt_ave[l], s.linkc_tt_std[l], s.linkc_tt_ave[l]/s.linkc_tt_free[l], l.length])
        s.df_linkc = pd.DataFrame(out[1:], columns=out[0])
        return s.df_linkc

    def link_traffic_state_to_pandas(s):
        """
        Compute the traffic states in links and return as a pandas DataFrame.

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
        Compute the cumulative counts etc. in links and return as a pandas DataFrame.

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
        Save all results to CSV files. This is obsolute; not all functions are implemented.
        """
        if fname == None:
            fname = f"out{s.W.name}/data"
        s.basic_to_pandas().to_csv(fname+"_basic.csv", index=False)
        s.od_to_pandas().to_csv(fname+"_od.csv", index=False)
        s.mfd_to_pandas().to_csv(fname+"_mfd.csv", index=False)
        s.link_to_pandas().to_csv(fname+"_link.csv", index=False)
        s.link_traffic_state_to_pandas().to_csv(fname+"_link_traffic_state.csv", index=False)
        s.vehicles_to_pandas().to_csv(fname+"_vehicles.csv", index=False)
