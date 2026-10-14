# Prints the geometry of the UXsim mark (ouroboros road) in the 512x512 icon coordinate system, centre (256,256).
# Angles are degrees, clockwise in SVG coordinates.
# Output: YELLOW (tail half), BLUE (head half), BITE (clipPath for the yellow path), LANE (lane marking path and dasharray), EYE (circle).
# The same geometry is used in uxsim-icon-*.svg, and scaled in uxsim-logo-*.svg and uxsim-hero-*.svg.
import math
CX=CY=256
R=164;W=92;th=-60;delta=8;Dt=105;tipf=0.22;k=0.5;b=8;neck=24;gap=9;snout=1.25
seam=120; ov=1.5
lane_start=-15.775  #start angle of the lane marking (where the tapered tail is about 0.62W wide); value taken from the final SVG
lane_n=17  # number of dashes; the marking starts and ends with a dash, dash:gap = 1:0.7
Ro=R+W/2; Ri=R-W/2
tt=th+delta; ta=tt+Dt; tb=th+360; tn=tb-neck
def P(r,a):
    t=math.radians(a); return (CX+r*math.cos(t), CY+r*math.sin(t))
def f(p): return f"{p[0]:.2f} {p[1]:.2f}"
s=lambda u: math.sin(math.pi*u/2); ds=lambda u: math.pi/2*math.cos(math.pi*u/2)
def radii(a):
    u=(a-tt)/Dt; w=W*(tipf+(1-tipf)*s(u)); dw=W*(1-tipf)*ds(u)/math.radians(Dt); loss=W-w
    return (Ro-(1-k)*loss, (1-k)*dw), (Ri+k*loss, -k*dw)
def pt(rr,a):
    r,dr=rr; t=math.radians(a)
    return (CX+r*math.cos(t),CY+r*math.sin(t)), (dr*math.cos(t)-r*math.sin(t), dr*math.sin(t)+r*math.cos(t))
def bez(fn,a0,a1,n=5):
    out=""
    for i in range(n):
        u0=a0+(a1-a0)*i/n; u1=a0+(a1-a0)*(i+1)/n; h=math.radians(u1-u0)
        p0,d0=pt(fn(u0),u0); p1,d1=pt(fn(u1),u1)
        out+=f" C{f((p0[0]+h/3*d0[0],p0[1]+h/3*d0[1]))} {f((p1[0]-h/3*d1[0],p1[1]-h/3*d1[1]))} {f(p1)}"
    return out
def arc(r,a0,a1):
    return f" A{r:.2f} {r:.2f} 0 {1 if abs(a1-a0)>180 else 0} {1 if a1>a0 else 0} {f(P(r,a1))}"
o=lambda a: radii(a)[0]; i_=lambda a: radii(a)[1]
rot,_=o(tt); rit,_=i_(tt)
yellow=f"M{f(P(Ro,seam+ov))}"+arc(Ro,seam+ov,ta)+bez(o,ta,tt)+f" A{(rot-rit)/2:.2f} {(rot-rit)/2:.2f} 0 0 0 {f(P(rit,tt))}"+bez(i_,tt,ta)+arc(Ri,ta,seam+ov)+" Z"
g=lambda sg: 3*sg*sg-2*sg**3; dg=lambda sg: 6*sg-6*sg*sg
ho=lambda a: (Ro+b*g((a-tn)/neck), b*dg((a-tn)/neck)/math.radians(neck))
hi=lambda a: (Ri-b*g((a-tn)/neck), -b*dg((a-tn)/neck)/math.radians(neck))
hr=W/2+b; rx=hr*snout; rd=th+90
blue=f"M{f(P(Ro,seam))}"+arc(Ro,seam,tn)+bez(ho,tn,tb,3)+f" A{rx:.2f} {hr:.2f} {rd:.2f} 0 1 {f(P(Ri-b,tb))}"+bez(hi,tb,tn,3)+arc(Ri,tn,seam)+" Z"
print("YELLOW:",yellow); print("BLUE:",blue)
# Gap around the head: an ellipse (head ellipse enlarged by gap) cut out of the full canvas with clip-rule evenodd
def bite(shrink):
    ex,ey=rx+gap-shrink,hr+gap-shrink; c=P(R,th); t=math.radians(rd)
    e1=(c[0]-ex*math.cos(t),c[1]-ex*math.sin(t)); e2=(c[0]+ex*math.cos(t),c[1]+ex*math.sin(t))
    return f"M0 0H512V512H0Z M{f(e1)} A{ex:.2f} {ey:.2f} {rd:.2f} 1 0 {f(e2)} A{ex:.2f} {ey:.2f} {rd:.2f} 1 0 {f(e1)}Z"
print("BITE:",bite(0))
# transparent-outline style: the bite shrunk by the outline width w (mark units), used to clip the white underlay of the tail
# hero w=1.5/1.0296 (1.5px), logo w=3 (1px), icon w=4 (4px); the underlay stroke-width is 2w
for w in (1.5/1.0296,3,4):
    print(f"BITE_OUTLINE w={w:.3f}:",bite(w))
# Lane marking on the centre line (radius R) from lane_start to tn-6
lane_end=tn-6
lane=f"M{f(P(R,lane_start))}"+bez(lambda a:(R,0),lane_start,ta,3)+arc(R,ta,lane_end)
L=math.radians(lane_end-lane_start)*R; dash=L/(lane_n+0.7*(lane_n-1))
print("LANE:",lane); print("LANE dasharray:",f"{dash:.2f} {0.7*dash:.2f}")
eye=P(R+hr*0.42,th+4)
print("EYE:",f'cx="{eye[0]:.2f}" cy="{eye[1]:.2f}" r="10.1"')
