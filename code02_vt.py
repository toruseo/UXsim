from pylab import *

# 初期設定
#----編集箇所：ここから----
u = 20       #自由流速度 m/s
w = 5        #後進波速度 m/s
kappa = 0.25 #渋滞密度 veh/m
tmax = 300   #シミュ時間 s
xmax = 500   #リンク長 m
dt = 10      #時間離散化幅 s
#----編集箇所：ここまで----

dx = u*w/(u+w)*dt
k_c = kappa*w/(u+w)
q_c = u*k_c

## 離散化幅
tsize = round(tmax/dt)
xsize = round(xmax/dx)

# VTネットワーク構成
net = full([tsize*xsize, tsize*xsize], inf)
for i in range(tsize):
    for j in range(xsize):
        if j < xsize-1:
            net[i*xsize+j, i*xsize+j+1] = 0
        if j > 0 and i < tsize-1:
            net[i*xsize+j, (i+1)*xsize+j-1] = kappa*dx
    ## 境界条件
    #----編集箇所：ここから----
    net[0, i*xsize+0] = q_c*dt*i*0.4    #veh
    #----編集箇所：ここまで----

## 内部条件
#----編集箇所：ここから----
#x = 300の50 < t < 150は赤信号で容量0
j = round(300/dx)
for i in range(tsize):
    if 50 < i*dt < 150:
        net[i*xsize+j, (i+1)*xsize+j] = 0
#----編集箇所：ここまで---

# VT計算本体
from scipy.sparse.csgraph import csgraph_from_dense, shortest_path
net = csgraph_from_dense(net, null_value=inf)
sol = shortest_path(csgraph=net, indices=0)

# 可視化
t = [i*dt + j*dx/u for i in range(tsize) for j in range(xsize)]
x = [j*dx for i in range(tsize) for j in range(xsize)]
figure(figsize=(8,3))
tricontourf(t, x, sol, levels=20)
colorbar().set_label("cumulative vehicle (veh)")
xlabel("time (s)")
ylabel("space (m)")
tight_layout()
show()