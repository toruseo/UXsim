from pylab import *
import random, copy, tqdm, glob, os, csv
from collections import deque, OrderedDict
rcParams["font.family"] = "monospace"

# ノードクラス
class Node:
    def __init__(s, name, x, y):
        #ノード位置（可視化用）
        s.x = x
        s.y = y
        
        #流入・流出リンク
        s.inlinks = {}
        s.outlinks = {}
        
        #リンク間遷移リクエスト（デマンド）
        s.incoming_vehicles = []
        
        #発生車両流入待ち行列（vertical queue）
        s.generation_queue = deque()
        
        s.id = len(NODES)
        s.name = name
        NODES.append(s)
    
    def __repr__(s):
        return f"<Node {s.name}>"
    
    def generate(s):
        #出発待ち行列から出発
        if len(s.generation_queue) > 0:
            veh = s.generation_queue[0]
            outlinks = list(s.outlinks.values())
            preference = [veh.route_pref[l] for l in outlinks]
            outlink = random.choices(outlinks, preference)[0]
            
            if (len(outlink.vehicles) == 0 or 
                    outlink.vehicles[-1].x > outlink.delta*DELTAN):
                #受け入れ可能な場合，リンク優先度に応じて選択
                veh = s.generation_queue.popleft()
                
                veh.state = "run"
                veh.link = outlink
                veh.x = 0
                VEHICLES_RUNNING[veh.name] = veh
                
                if len(outlink.vehicles) > 0:
                    veh.leader = outlink.vehicles[-1]
                
                outlink.vehicles.append(veh)
    
    def transfer(s):
        #リンク間遷移
        for outlink in {veh.route_next_link 
                for veh in s.incoming_vehicles}:
            if (len(outlink.vehicles) == 0 or 
                    outlink.vehicles[-1].x > outlink.delta*DELTAN):
                #受け入れ可能な場合，リンク優先度に応じて選択
                vehs = [veh for veh in s.incoming_vehicles 
                        if veh.route_next_link == outlink]
                veh = random.choices(vehs, 
                        [veh.link.merge_priority for veh in vehs])[0]
                
                #リンク間遷移実行
                veh.link.vehicles.popleft()
                veh.link = outlink
                veh.x = 0
                
                veh.leader = None
                if len(outlink.vehicles):
                    veh.leader = outlink.vehicles[-1]
                
                outlink.vehicles.append(veh)
                s.incoming_vehicles.remove(veh)
        
        s.incoming_vehicles = []

# リンククラス
class Link:
    def __init__(s, name, start_node, end_node, length, 
            free_flow_speed, jam_density, merge_priority=1):
        #起点・終点ノード
        s.start_node = get_node(start_node)
        s.end_node = get_node(end_node)
        
        #リンク長
        s.length = length
        
        #フローモデルパラメータ
        s.u = free_flow_speed
        s.kappa = jam_density
        s.tau = REACTION_TIME
        s.w = 1/s.tau/s.kappa
        s.capacity = s.u*s.w*s.kappa/(s.u+s.w)
        s.delta = 1/s.kappa
        
        #合流時優先度
        s.merge_priority = merge_priority
        
        #リンク内車両一覧
        s.vehicles = deque()
        
        #旅行時間
        s.traveltime_instant = []
        
        #Euler型交通状態
        s.edie_dt = DELTAT*TS_AGG_SIZE
        s.edie_dx = s.u*DELTAT
        s.k_mat = zeros([int(TMAX/s.edie_dt)+1, int(s.length/s.edie_dx)])
        s.q_mat = zeros(s.k_mat.shape)
        s.v_mat = zeros(s.k_mat.shape)
        s.tn_mat = zeros(s.k_mat.shape)
        s.dn_mat = zeros(s.k_mat.shape)
        s.an = s.edie_dt*s.edie_dx
        
        s.id = len(LINKS)
        s.name = name
        LINKS.append(s)
        s.start_node.outlinks[s.name] = s
        s.end_node.inlinks[s.name] = s
    
    def __repr__(s):
        return f"<Link {s.name}>"
    
    def update(s):
        #更新
        s.set_traveltime()
    
    def set_traveltime(s):
        #瞬間旅行時間算出
        if len(s.vehicles):
            vave = average([veh.v for veh in s.vehicles])
            if vave > 0:
                s.traveltime_instant.append(s.length/vave)
            else:
                s.traveltime_instant.append(s.length/(s.u/10))
        else:
            s.traveltime_instant.append(s.length/s.u)
    
    def compute_traffic_states(s):
        #Euler型の流率・密度をEdieの定義で計算
        for veh in VEHICLES.values():
            for i in range(len(veh.log_t)):
                if veh.log_link[i] == s and veh.log_x[i] != 0:
                    ii = int(veh.log_t[i]/s.edie_dt)
                    jj = int(veh.log_x[i]/s.edie_dx)
                    if (0 <= ii < s.tn_mat.shape[0] 
                            and 0 <= jj < s.tn_mat.shape[1]):
                        s.tn_mat[ii,jj] += DELTAN*DELTAT
                        s.dn_mat[ii,jj] += DELTAN*veh.log_v[i]*DELTAT
        s.k_mat = s.tn_mat/s.an
        s.q_mat = s.dn_mat/s.an
        with errstate(invalid="ignore"):
            s.v_mat = s.dn_mat/s.tn_mat
        s.v_mat = nan_to_num(s.v_mat, nan=s.u)

# 車両クラス
class Vehicle:
    def __init__(s, orig, dest, departure_time, name=None, 
            route_pref=None):
        #出発・目的地ノード
        s.orig = get_node(orig)
        s.dest = get_node(dest)
        
        #出発・到着時刻
        s.departure_time = departure_time
        s.arrival_time = -1
        
        #状態：home, wait, run，end
        s.state = "home"
        
        #リンク内位置
        s.link = None
        s.x = 0
        s.x_next = 0
        s.v = 0
        
        #先行・後行車
        s.leader = None
        
        #希望リンク重み：{link:重み}
        s.route_pref = route_pref
        if s.route_pref == None:
            s.route_pref = {l:0 for l in LINKS}
        
        #ログなど
        s.log_t = [] #時刻
        s.log_state = [] #状態
        s.log_link = [] #リンク
        s.log_x = [] #位置
        s.log_s = [] #車頭距離
        s.log_v = [] #現在速度
        s.color = (random.random(), random.random(), random.random())
        
        s.id = len(VEHICLES)
        if name != None:
            s.name = name
        else:
            s.name = str(s.id)
        VEHICLES[s.name] = s
        VEHICLES_LIVING[s.name] = s
    
    def __repr__(s):
        return f"<Vehicle {s.name}: {s.state}, x={s.x}, link={s.link}>"

    def update(s):
        #更新
        s.record_log()
        
        if s.state == "home":
            #需要生成
            if TIME >= s.departure_time:
                s.state = "wait"
                s.orig.generation_queue.append(s)
        if s.state == "wait":
            #出発ノードで待つ
            pass
        if s.state == "run":
            #走行
            s.v = (s.x_next-s.x)/DELTAT
            s.x = s.x_next
            
            #リンク下流端
            if s.x == s.link.length:
                if s.link.end_node == s.dest:
                    #トリップ終了
                    s.end_trip()
                else:
                    #リンク間遷移リクエスト
                    s.route_next_link_choice()
                    s.link.end_node.incoming_vehicles.append(s)
        if s.state == "end":
            #終わり
            pass
    
    def end_trip(s):
        #トリップ終了処理
        s.state = "end"
        s.link.vehicles.popleft()
        s.link = None
        s.x = 0
        s.arrival_time = TIME
        VEHICLES_RUNNING.pop(s.name)
        VEHICLES_LIVING.pop(s.name)
    
    def carfollow(s):
        #リンク内走行
        x_free = s.x + s.link.u*DELTAT
        x_cong = x_free
        if s.leader != None and s.link == s.leader.link:
            x_cong = s.leader.x - s.link.delta*DELTAN
            if x_cong < s.x:
                x_cong = s.x
        
        s.x_next = min([x_free, x_cong])
        
        if s.x_next > s.link.length:
            s.x_next = s.link.length
    
    def route_pref_update(s, weight):
        #経路選択のためのリンク選好を慣性DUO式で更新
        route_pref_new = {l:0 for l in LINKS}
        k = s.dest.id
        for l in LINKS:
            i = l.start_node.id
            j = l.end_node.id
            if j == ROUTECHOICE.next[i,k]:
                route_pref_new[l] = 1
        
        if sum(list(s.route_pref.values())) == 0:
            #最初にpreferenceが空なら確定的に初期化
            weight = 1
        for l in s.route_pref.keys():
            s.route_pref[l] = (1-weight)*s.route_pref[l] + \
                    weight*route_pref_new[l]
    
    def route_next_link_choice(s):
        #現在のリンクから次に移るリンクを選択
        if s.dest != s.link.end_node:
            outlinks = list(s.link.end_node.outlinks.values())
            
            preference = [s.route_pref[l] for l in outlinks]
            s.route_next_link = random.choices(outlinks, preference)[0]
    
    def record_log(s):
        #走行履歴保存
        if s.state != "run":
            s.log_t.append(TIME*DELTAT)
            s.log_state.append(s.state)
            s.log_link.append(-1)
            s.log_x.append(-1)
            s.log_s.append(-1)
            s.log_v.append(-1)
        else:
            s.log_t.append(TIME*DELTAT)
            s.log_state.append(s.state)
            s.log_link.append(s.link)
            s.log_x.append(s.x)
            s.log_v.append(s.v)
            if s.leader != None and s.link == s.leader.link:
                s.log_s.append(s.leader.x-s.x)
            else:
                s.log_s.append(-1)

# 経路選択クラス
class RouteChoice:
    def __init__(s):
        #リンク旅行時間行列
        s.adj_mat_time = zeros([len(NODES), len(NODES)]) 
        #ij間最短距離
        s.dist = zeros([len(NODES), len(NODES)]) 
        #iからjに行くために次に進むべきノード
        s.next = zeros([len(NODES), len(NODES)])
    
    def route_search_all(s, infty=9999999999999999999):
        #現時刻の最短経路を計算
        for link in LINKS:
            i = link.start_node.id
            j = link.end_node.id
            if ADJ_MAT[i,j]:
                s.adj_mat_time[i,j] = link.traveltime_instant[-1]
            else:
                s.adj_mat_time[i,j] = 0
        
        dist = zeros([len(NODES), len(NODES)]) 
        next = zeros([len(NODES), len(NODES)])
        for i in range(len(NODES)):
            for j in range(len(NODES)):
                if s.adj_mat_time[i,j] > 0:
                    dist[i,j] = s.adj_mat_time[i,j]
                    next[i,j] = j
                elif i == j:
                    next[i,j] = j
                else:
                    dist[i,j] = infty
                    next[i,j] = -1
        
        for k in range(len(NODES)):
            for i in range(len(NODES)):
                for j in range(len(NODES)):
                    if dist[i,j] > dist[i,k]+dist[k,j]:
                        dist[i,j] = dist[i,k]+dist[k,j]
                        next[i,j] = next[i,k]
        s.dist = dist
        s.next = next

# 結果分析クラス
class Analyzer:
    def __init__(s):
        for l in LINKS:
            l.compute_traffic_states()
        os.makedirs("out", exist_ok=True)
        
    def print_simple_stats(s):
        #簡単な結果表示
        print("results:")
        trip_comp, trip_comp_t = 0, 0
        for veh in VEHICLES.values():
            if veh.arrival_time != -1:
                trip_comp += 1
                trip_comp_t += (veh.arrival_time-veh.departure_time)\
                        *DELTAT
        print(" average speed:\t", sum([sum(l.dn_mat) for l in LINKS])/\
                sum([sum(l.tn_mat) for l in LINKS]), "m/s")
        print(" number of completed trips:\t", trip_comp*DELTAN, 
                "/", len(VEHICLES)*DELTAN)
        print(" average travel time of trips:\t", 
                trip_comp_t/trip_comp, "s")
    
    def time_space_diagram_traj(s, figsize=(12,4)):
        #リンク車両軌跡の時空間図
        print(" drawing trajectories...")
        for l in tqdm.tqdm(LINKS):
            tss, xss, cs = [], [], []
            for veh in VEHICLES.values():
                tss.append([veh.log_t[i] for i in range(len(veh.log_t))\
                         if veh.log_link[i] == l])
                xss.append([veh.log_x[i] for i in range(len(veh.log_t))\
                         if veh.log_link[i] == l])
                cs.append(veh.color)
            for i in range(len(xss)):
                for j in range(len(xss[i])-1, 0, -1):
                    if xss[i][j] < xss[i][j-1]:
                        xss.append(xss[i][j:])
                        tss.append(tss[i][j:])
                        cs.append(cs[i])
                        del xss[i][j:]
                        del tss[i][j:]
            figure(figsize=figsize)
            title(l)
            for i in range(len(xss)):
                plot(tss[i], xss[i], c=cs[i], lw=0.5)
            xlabel("time (s)")
            ylabel("space (m)")
            xlim([0, TMAX])
            ylim([0, l.length])
            grid()
            tight_layout()
            savefig(f"out/tsd_traj_{l.name}.png")
            close("all")#show()
    
    def time_space_diagram_density(s, figsize=(12,4)):
        #リンク密度の時空間図
        print(" drawing traffic states...")
        for l in tqdm.tqdm(LINKS):
            figure(figsize=figsize)
            title(l)
            imshow(l.k_mat.T, origin="lower", aspect="auto", 
                    extent=(0,int(TMAX/l.edie_dt)*l.edie_dt,0, 
                    int(l.length/l.edie_dx)*l.edie_dx), 
                    interpolation="none", vmin=0, vmax=1/l.delta, 
                    cmap="inferno")
            colorbar().set_label("density (veh/m)")
            xlabel("time (s)")
            ylabel("space (m)")
            xlim([0, TMAX])
            ylim([0, l.length])
            tight_layout()
            savefig(f"out/tsd_k_{l.name}.png")
            close("all")#show()
    
    def network(s, t=None, detailed=1, minwidth=0.5, maxwidth=12, 
            left_handed=1, tmp_anim=0, figsize=(6,6)):
        #ネットワーク全体の交通状況
        #detailed=1の時，リンク内部を詳細に描画，0の時簡略化
        figure(figsize=figsize)
        subplot(111, aspect="equal")
        title(f"t = {t*DELTAT :>8} (s)")
        for n in NODES:
            plot(n.x, n.y, "ko", ms=10, zorder=10)
            text(n.x, n.y, n.name, c="g", horizontalalignment="center",
                    verticalalignment="top", zorder=20)
        for l in LINKS:
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
                    k = l.k_mat[int(t*DELTAT/l.edie_dt), i+1]
                    v = l.v_mat[int(t*DELTAT/l.edie_dt), i+1]
                    lw[i] = k*l.delta*(maxwidth-minwidth)+minwidth
                    c[i] = colormaps["viridis"](v/l.u)
                xmid = [((xsize-i)*x1+(i+1)*x2)/(xsize+1)+vx \
                        for i in range(xsize)]
                ymid = [((xsize-i)*y1+(i+1)*y2)/(xsize+1)+vy \
                        for i in range(xsize)]
                plot([x1]+xmid+[x2], [y1]+ymid+[y2], "k--", 
                        lw=0.25, zorder=5)
                for i in range(xsize-1):
                    plot([xmid[i], xmid[i+1]], [ymid[i], ymid[i+1]], 
                            c=c[i], lw=lw[i], zorder=6)
                text(xmid[int(len(xmid)/2)], ymid[int(len(xmid)/2)], 
                        l.name, c="b", zorder=20)
            else:
                #簡略モード
                k = average(l.k_mat[int(t*DELTAT/l.edie_dt), :])
                v = average(l.v_mat[int(t*DELTAT/l.edie_dt), :])
                width = k*l.delta*(maxwidth-minwidth)+minwidth
                c = colormaps["viridis"](v/l.u)                
                xmid1, ymid1 = (2*x1+x2)/3+vx, (2*y1+y2)/3+vy
                xmid2, ymid2 = (x1+2*x2)/3+vx, (y1+2*y2)/3+vy
                plot([x1, xmid1, xmid2, x2], [y1, ymid1, ymid2, y2], 
                        "k--", lw=0.5, zorder=5)
                plot([x1, xmid1, xmid2, x2], [y1, ymid1, ymid2, y2], 
                        c=c, lw=width, zorder=6, solid_capstyle="butt")
                text(xmid1, ymid1, l.name, c="b", zorder=20)
        maxx = max([n.x for n in NODES])
        minx = min([n.x for n in NODES])
        maxy = max([n.y for n in NODES])
        miny = min([n.y for n in NODES])
        buffx, buffy = (maxx-minx)/10, (maxy-miny)/10
        xlim([minx-buffx, maxx+buffx])
        ylim([miny-buffy, maxy+buffy])
        tight_layout()
        if tmp_anim:
            savefig(f"out/tmp_anim_{t}.png")
            close("all")
        else:
            savefig(f"out/network{detailed}_{t}.png")
            close("all")#show()
    
    def network_anim(s, duration=100, detailed=1, minwidth=0.5, 
            maxwidth=12, left_handed=1, figsize=(6,6)):
        #ネットワーク全体の交通状況のアニメーション
        print(" generating animation...")
        from PIL import Image
        pics = []
        for t in tqdm.tqdm(range(0,TSIZE,TS_AGG_SIZE)):
            s.network(t, detailed=detailed, minwidth=minwidth, 
                    maxwidth=maxwidth, left_handed=left_handed, 
                    tmp_anim=1, figsize=figsize)
            pics.append(Image.open(f"out/tmp_anim_{t}.png"))
        pics[0].save(f"out/anim_network{detailed}.gif", save_all=True, 
                append_images=pics[1:], optimize=False, 
                duration=duration, loop=0)
        for f in glob.glob("out/tmp_anim_*.png"):
            os.remove(f)
    
    def macroscopic_fundamental_diagram(s, kappa=0.2, figsize=(4,4)):
        #MFDの描画
        for i in range(len(Q_AREA)):
            tn = sum([l.tn_mat[i,:].sum() for l in LINKS])
            dn = sum([l.dn_mat[i,:].sum() for l in LINKS])
            an = sum([l.length*DELTAT*TS_AGG_SIZE for l in LINKS])
            K_AREA[i] = tn/an
            Q_AREA[i] = dn/an
        figure(figsize=figsize)
        plot(K_AREA, Q_AREA, "ko-")
        xlabel("network average density (veh/m)")
        ylabel("network average flow (veh/s)")
        xlim([0, kappa])
        ylim([0, 1/REACTION_TIME])
        grid()        
        tight_layout()
        savefig(f"out/mfd.png")
        close("all")#show()

    def output_data(s):
        out = [["name", "dn", "orig", "dest", "t", "link", "x", "s", "v"]]
        for veh in VEHICLES.values():
            for i in range(len(veh.log_t)):
                if veh.log_state[i] in ("wait", "run"):
                    if veh.log_link[i] != -1:
                        linkname = veh.log_link[i].name
                    else:
                        linkname = "waiting_at_origin_node"
                    out.append([veh.name, DELTAN, veh.orig.name, 
                            veh.dest.name, veh.log_t[i], linkname, 
                            veh.log_x[i], veh.log_s[i], veh.log_v[i]])
        with open("out/out.csv", "w") as f:
            writer = csv.writer(f, lineterminator="\n")
            writer.writerows(out)

# 各種ユーティリティ関数
def get_node(node):
    #Nodeインスタンスnodeか，
    #nameがnodeであるNodeインスタンスを返す関数
    if type(node) is Node:
        return node
    elif type(node) is str:
        for n in NODES:
            if n.name == node:
                return n
    raise Exception(f"{node} is not node")

def generate_demand(orig, dest, t_start, t_end, flow):
    #時間帯OD需要の生成関数
    #時刻t_start (s)からt_end (s)までorigからdestに
    #向かう流率flow (veh/s)の需要を生成
    f = 0
    for t in range(int(t_start/DELTAT), int(t_end/DELTAT)):
        f += flow*DELTAT
        if f >= DELTAN:
            Vehicle(orig, dest, t)
            f -= DELTAN

# メインループ
if __name__ == "__main__":
    ## パラメータ設定
    #----編集箇所：ここから----
    TMAX = 1200    #s
    DELTAN = 5     #veh
    REACTION_TIME = 1     #s
    DUO_UPDATE_TIME = 600 #s
    DUO_UPDATE_WEIGHT = 0.2
    random.seed(0)
    #----編集箇所：ここまで----
    
    DELTAT = REACTION_TIME*DELTAN
    TSIZE = int(TMAX/DELTAT)
    DELTAT_ROUTE = int(DUO_UPDATE_TIME/DELTAT)
    
    ## データ格納先定義
    TS_AGG_SIZE = 20
    VEHICLES = OrderedDict()        #home, wait, run, end
    VEHICLES_LIVING = OrderedDict() #home, wait, run
    VEHICLES_RUNNING = OrderedDict()#run
    NODES = []
    LINKS = []
    Q_AREA = zeros(int(TMAX/DELTAT/TS_AGG_SIZE))
    K_AREA = zeros(int(TMAX/DELTAT/TS_AGG_SIZE))
    
    ## ネットワーク定義・需要生成
    #----編集箇所：ここから----
    #合流ネットワーク
    Node("orig1", 0, 0)
    Node("orig2", 0, 2)
    Node("merge", 1, 1)
    Node("dest", 2, 1)
    Link("link1", "orig1", "merge", length=1000, 
            free_flow_speed=20, jam_density=0.2, merge_priority=0.5)
    Link("link2", "orig2", "merge", length=1000, 
            free_flow_speed=20, jam_density=0.2, merge_priority=2)
    Link("link3", "merge", "dest", length=1000, 
            free_flow_speed=20, jam_density=0.2)
    generate_demand("orig1", "dest", 0, 1000, 0.4)
    generate_demand("orig2", "dest", 500, 1000, 0.6)
    #----編集箇所：ここまで----
    
    ## 隣接行列計算
    ROUTECHOICE = RouteChoice()
    ADJ_MAT = zeros([len(NODES), len(NODES)])
    for link in LINKS:
        for i,node in enumerate(NODES):
            if node == link.start_node:
                break
        for j,node in enumerate(NODES):
            if node == link.end_node:
                break
        ADJ_MAT[i,j] = 1
    
    ## 問題規模表示
    print("simulation setting:")
    print(" simulation duration:\t", TMAX, "s")
    print(" number of vehicles:\t", len(VEHICLES)*DELTAN, "veh")
    print(" total road length:\t", sum([l.length for l in LINKS]),"m")
    print(" time discret. width:\t", DELTAT, "s")
    print(" platoon size:\t\t", DELTAN, "veh")
    print(" number of timesteps:\t", TSIZE)
    print(" number of platoons:\t", len(VEHICLES))
    print(" number of links:\t", len(LINKS))
    print(" number of nodes:\t", len(NODES))
    
    ## メインループ本体
    print("simulating...")
    for TIME in tqdm.tqdm(range(0, TSIZE)):
        for link in LINKS:
            link.update()
        
        for node in NODES:
            node.generate()
        
        for node in NODES:
            node.transfer()

        for veh in VEHICLES_RUNNING.values():
            veh.carfollow()
        
        for veh in copy.copy(VEHICLES_LIVING).values():
            veh.update()
        
        if TIME % DELTAT_ROUTE == 0:
            ROUTECHOICE.route_search_all()
            for veh in VEHICLES_LIVING.values():
                veh.route_pref_update(weight=DUO_UPDATE_WEIGHT)
    
    ## 結果出力
    analyzer = Analyzer()
    analyzer.print_simple_stats()
    #----編集箇所：ここから----
    analyzer.time_space_diagram_density()
    analyzer.time_space_diagram_traj()
    analyzer.macroscopic_fundamental_diagram()
    for t in range(0,TSIZE,int(TSIZE/8)):
        analyzer.network(t, detailed=0)
    for t in range(0,TSIZE,int(TSIZE/8)):
        analyzer.network(t, detailed=1)
    analyzer.network_anim(100, detailed=0)
    analyzer.network_anim(100, detailed=1)
    analyzer.output_data()
    #----編集箇所：ここまで----