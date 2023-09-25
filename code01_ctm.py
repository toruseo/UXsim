from pylab import *

# 初期設定
#----編集箇所：ここから----
u = 20       #自由流速度 m/s
w = 5        #後進波速度 m/s
kappa = 0.25 #渋滞密度 veh/m
tmax = 300   #シミュ時間 s
xmax = 500   #リンク長 m
dt = 2       #タイムステップ幅 s
dx = 40      #セルサイズ m
#----編集箇所：ここまで----

k_c = kappa*w/(u+w)
q_c = u*k_c

## 離散化幅
tsize = round(tmax/dt)
xsize = round(xmax/dx)+2    #足している2は境界条件を表現するダミーセル
k = zeros([tsize, xsize])
q = zeros([tsize, xsize])

## 初期・境界条件
#----編集箇所：ここから----
q[:,0] = 0.5       #上流流入率 veh/s
k[:,-1] = 0.2      #下流密度（前半） veh/m
k[80:,-1] = 0.05   #下流密度（後半） veh/m
#----編集箇所：ここまで----

# CTM本体
for i in range(tsize-1):
    for j in range(1, xsize-1):
        q[i,j] = min([u*k[i,j], q_c, w*(kappa-k[i,j+1])])
        k[i+1,j] = k[i,j] + dt/dx*(q[i,j-1]-q[i,j])

# 可視化
figure(figsize=(8,3))
imshow(k[:,1:-1].T, origin="lower", aspect="auto", 
    extent=(0,tmax,0,xmax), vmin=0, vmax=kappa, 
    interpolation="none", cmap="inferno")
colorbar().set_label("density (veh/m)")
xlabel("time (s)")
ylabel("space (m)")
tight_layout()
show()