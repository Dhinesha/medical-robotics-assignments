import browserbotics as bb
import time
import math
import random

# ═══════════════════════════════════════════════════════════════════════════════
#  HOSPITAL ROOM  (walls, bed, patient, furniture)
# ═══════════════════════════════════════════════════════════════════════════════
def setup_hospital_room():
    bb.addGroundPlane()

    W = 6; H = 1.5; WT = 0.05
    bb.createBody('box', halfExtent=[WT, W, H], position=[-W, 0,  H], color='white', mass=0)
    bb.createBody('box', halfExtent=[WT, W, H], position=[ W, 0,  H], color='white', mass=0)
    bb.createBody('box', halfExtent=[W, WT, H], position=[ 0, W,  H], color='white', mass=0)

    DW = 0.5; DH = 1.0; SW = W - DW
    bb.createBody('box', halfExtent=[SW, WT, H],     position=[-(DW+SW), -W, H],      color='white', mass=0)
    bb.createBody('box', halfExtent=[SW, WT, H],     position=[ (DW+SW), -W, H],      color='white', mass=0)
    bb.createBody('box', halfExtent=[DW, WT, H-DH/2],position=[0, -W, DH+H-DH/2],    color='white', mass=0)

    bb.createBody('box', halfExtent=[W, W, 0.05], position=[0, 0, H*2+0.05], color='#F0F0F0', mass=0)

    FC = '#3E2723'; FH = 0.05
    DX, DY = 0.0, -W+WT+0.01
    bb.createBody('box', halfExtent=[DW-FH, 0.03, DH/2-FH], position=[DX, DY, DH/2],         color='#8B4513', mass=0)
    bb.createBody('box', halfExtent=[DW*0.5, 0.04, DH*0.13],position=[DX, DY+0.01, DH*0.74], color='#6B3410', mass=0)
    bb.createBody('box', halfExtent=[DW*0.5, 0.04, DH*0.20],position=[DX, DY+0.01, DH*0.27], color='#6B3410', mass=0)
    bb.createBody('box', halfExtent=[FH, FH, DH/2],          position=[-DW, -W, DH/2],        color=FC, mass=0)
    bb.createBody('box', halfExtent=[FH, FH, DH/2],          position=[ DW, -W, DH/2],        color=FC, mass=0)
    bb.createBody('box', halfExtent=[DW+FH, FH, FH],         position=[0,   -W, DH],           color=FC, mass=0)
    HX = DW*0.48; HZ = DH*0.50
    bb.createBody('box',    halfExtent=[0.04, 0.01, 0.07],   position=[HX, DY-0.01, HZ],      color='#B8860B', mass=0)
    bb.createBody('box',    halfExtent=[0.09, 0.013, 0.013], position=[HX+0.05, DY, HZ],       color='#DAA520', mass=0)
    bb.createBody('sphere', radius=0.025,                    position=[HX+0.15, DY, HZ],       color='#DAA520', mass=0)

    bb.createBody('box', halfExtent=[0.45,1.0,0.2],  position=[0,1.5,0.2],  color='#AAAAAA', mass=0)
    bb.createBody('box', halfExtent=[0.42,0.9,0.06], position=[0,1.5,0.46], color='white',   mass=0)
    bb.createBody('box', halfExtent=[0.28,0.15,0.04],position=[0,2.3,0.54], color='#FFFDE7', mass=0)
    bb.createBody('box', halfExtent=[0.45,0.04,0.3], position=[0,2.5,0.5],  color='#8B6914', mass=0)
    bb.createBody('box', halfExtent=[0.45,0.04,0.2], position=[0,0.5,0.4],  color='#8B6914', mass=0)
    for lx,ly in [(-0.38,0.55),(0.38,0.55),(-0.38,2.45),(0.38,2.45)]:
        bb.createBody('box', halfExtent=[0.03,0.03,0.1], position=[lx,ly,0.1], color='#888888', mass=0)

    phz = 0.52
    bb.createBody('sphere', radius=0.12,              position=[0,2.32,phz+0.20], color='#FDBCB4', mass=0)
    bb.createBody('box', halfExtent=[0.18,0.35,0.12], position=[0,1.80,phz+0.12], color='#E8B4B8', mass=0)
    bb.createBody('box', halfExtent=[0.05,0.22,0.06], position=[-0.25,1.85,phz+0.18], color='#FDBCB4', mass=0)
    bb.createBody('box', halfExtent=[0.05,0.22,0.06], position=[ 0.25,1.85,phz+0.18], color='#FDBCB4', mass=0)
    bb.createBody('box', halfExtent=[0.04,0.14,0.05], position=[-0.32,1.55,phz+0.10], color='#F5DEB3', mass=0)
    bb.createBody('box', halfExtent=[0.04,0.14,0.05], position=[ 0.32,1.55,phz+0.10], color='#F5DEB3', mass=0)
    bb.createBody('box', halfExtent=[0.09,0.22,0.08], position=[-0.12,1.25,phz+0.08], color='#E8B4B8', mass=0)
    bb.createBody('box', halfExtent=[0.09,0.22,0.08], position=[ 0.12,1.25,phz+0.08], color='#E8B4B8', mass=0)
    bb.createBody('box', halfExtent=[0.07,0.20,0.06], position=[-0.15,0.85,phz+0.06], color='#D2B48C', mass=0)
    bb.createBody('box', halfExtent=[0.07,0.20,0.06], position=[ 0.15,0.85,phz+0.06], color='#D2B48C', mass=0)
    bb.createBody('box', halfExtent=[0.06,0.08,0.04], position=[-0.15,0.55,phz+0.04], color='#D2B48C', mass=0)
    bb.createBody('box', halfExtent=[0.06,0.08,0.04], position=[ 0.15,0.55,phz+0.04], color='#D2B48C', mass=0)

    bb.createBody('box', halfExtent=[0.015,0.015,0.8], position=[-0.8,2.4,0.8],  color='#CCCCCC', mass=0)
    bb.createBody('box', halfExtent=[0.07,0.025,0.1],  position=[-0.8,2.4,1.65], color='#80D8FF', mass=0)
    bb.createBody('box', halfExtent=[0.2,0.2,0.01],    position=[-0.8,2.4,0.02], color='#999999', mass=0)

    bb.createBody('box', halfExtent=[0.25,0.2,0.35],  position=[-0.9,2.0,0.35], color='#A0522D', mass=0)
    bb.createBody('box', halfExtent=[0.28,0.22,0.02], position=[-0.9,2.0,0.72], color='#C19A6B', mass=0)
    bb.createBody('box', halfExtent=[0.02,0.02,0.18], position=[-0.9,2.0,0.92],  color='#333333', mass=0)
    bb.createBody('box', halfExtent=[0.16,0.03,0.11], position=[-0.9,2.03,1.11], color='#111111', mass=0)
    bb.createBody('box', halfExtent=[0.13,0.01,0.09], position=[-0.9,2.01,1.11], color='#00E676', mass=0)

    bb.createBody('box', halfExtent=[0.18,0.18,0.025], position=[-1.5,0.5,0.55], color='#1565C0', mass=0)
    bb.createBody('box', halfExtent=[0.02,0.02,0.25],  position=[-1.5,0.5,0.27], color='#888888', mass=0)

    WX, WY, WZ = W-0.03, 2.0, 1.5
    bb.createBody('box', halfExtent=[0.03,0.66,0.03], position=[WX,WY,WZ+0.55],  color='#888888', mass=0)
    bb.createBody('box', halfExtent=[0.03,0.66,0.03], position=[WX,WY,WZ-0.55],  color='#888888', mass=0)
    bb.createBody('box', halfExtent=[0.03,0.03,0.55], position=[WX,WY-0.64,WZ],  color='#888888', mass=0)
    bb.createBody('box', halfExtent=[0.03,0.03,0.55], position=[WX,WY+0.64,WZ],  color='#888888', mass=0)
    bb.createBody('box', halfExtent=[0.02,0.62,0.52], position=[WX,WY,WZ],        color='#90CAF9', mass=0)

    bb.createBody('box', halfExtent=[0.4,0.15,0.5],  position=[-4.5,3.0,0.50], color='white',   mass=0)
    bb.createBody('box', halfExtent=[0.4,0.15,0.02], position=[-4.5,3.0,1.02], color='#DDDDDD', mass=0)

    bb.createBody('box', halfExtent=[0.3,0.3,0.025], position=[0,1.5,2.8], color='#FFFFE0', mass=0)
    bb.createBody('box', halfExtent=[0.02,0.02,0.3], position=[0,1.5,2.5], color='#AAAAAA', mass=0)


# ═══════════════════════════════════════════════════════════════════════════════
#  WALL STICKERS & POSTERS
# ═══════════════════════════════════════════════════════════════════════════════
def build_wall_stickers():
    W = 6; WY = W - 0.07
    LX = -W + 0.07
    RX =  W - 0.07

    ax, ay, az = -2.8, WY, 1.4
    bb.createBody('box', halfExtent=[0.55,0.04,0.70], position=[ax,ay,az],         color='#1565C0', mass=0)
    bb.createBody('box', halfExtent=[0.50,0.03,0.65], position=[ax,ay+0.01,az],    color='#E3F2FD', mass=0)
    bb.createBody('box', halfExtent=[0.50,0.02,0.07], position=[ax,ay+0.02,az+0.56], color='#0D47A1', mass=0)
    bb.createBody('sphere', radius=0.08,              position=[ax,ay+0.03,az+0.35], color='#90CAF9', mass=0)
    bb.createBody('box', halfExtent=[0.10,0.02,0.14], position=[ax,ay+0.03,az+0.14], color='#42A5F5', mass=0)
    bb.createBody('box', halfExtent=[0.03,0.02,0.10], position=[ax-0.13,ay+0.03,az+0.16], color='#64B5F6', mass=0)
    bb.createBody('box', halfExtent=[0.03,0.02,0.10], position=[ax+0.13,ay+0.03,az+0.16], color='#64B5F6', mass=0)
    bb.createBody('box', halfExtent=[0.03,0.02,0.10], position=[ax-0.06,ay+0.03,az-0.06], color='#64B5F6', mass=0)
    bb.createBody('box', halfExtent=[0.03,0.02,0.10], position=[ax+0.06,ay+0.03,az-0.06], color='#64B5F6', mass=0)
    bb.createBody('sphere', radius=0.015, position=[ax-0.03,ay+0.04,az+0.37], color='#00E5FF', mass=0)
    bb.createBody('sphere', radius=0.015, position=[ax+0.03,ay+0.04,az+0.37], color='#00E5FF', mass=0)
    for lz in [0.42, 0.22, 0.02, -0.14]:
        bb.createBody('box', halfExtent=[0.18,0.01,0.006], position=[ax+0.25,ay+0.02,az+lz], color='#90A4AE', mass=0)
        bb.createBody('sphere', radius=0.012, position=[ax+0.13,ay+0.025,az+lz], color='#FF5722', mass=0)

    bx, by, bz = -0.8, WY, 1.55
    bb.createBody('box', halfExtent=[0.30,0.03,0.30], position=[bx,by,bz],         color='#00695C', mass=0)
    bb.createBody('box', halfExtent=[0.26,0.02,0.26], position=[bx,by+0.01,bz],    color='#E0F2F1', mass=0)
    bb.createBody('box', halfExtent=[0.04,0.015,0.16], position=[bx,by+0.02,bz+0.05],     color='#00897B', mass=0)
    bb.createBody('box', halfExtent=[0.12,0.015,0.03], position=[bx,by+0.02,bz+0.18],     color='#00897B', mass=0)
    bb.createBody('box', halfExtent=[0.03,0.015,0.08], position=[bx-0.10,by+0.02,bz+0.12],color='#26A69A', mass=0)
    bb.createBody('box', halfExtent=[0.03,0.015,0.08], position=[bx+0.10,by+0.02,bz+0.12],color='#26A69A', mass=0)
    for ga in range(6):
        gangle = ga * math.pi / 3
        bb.createBody('box', halfExtent=[0.025,0.01,0.015],
                      position=[bx+0.16*math.cos(gangle), by+0.015, bz-0.14+0.16*math.sin(gangle)],
                      color='#80CBC4', mass=0)
    bb.createBody('sphere', radius=0.04, position=[bx,by+0.015,bz-0.14], color='#4DB6AC', mass=0)

    cx, cy, cz = 0.8, WY, 1.60
    bb.createBody('box', halfExtent=[0.22,0.03,0.22], position=[cx,cy,cz],         color='#B71C1C', mass=0)
    bb.createBody('box', halfExtent=[0.18,0.02,0.18], position=[cx,cy+0.01,cz],    color='#FFCDD2', mass=0)
    bb.createBody('box', halfExtent=[0.025,0.015,0.10], position=[cx,cy+0.02,cz+0.03], color='#C62828', mass=0)
    bb.createBody('sphere', radius=0.030,               position=[cx,cy+0.02,cz-0.10], color='#C62828', mass=0)
    bb.createBody('box', halfExtent=[0.22,0.015,0.015], position=[cx,cy+0.005,cz+0.20], color='#E53935', mass=0)
    bb.createBody('box', halfExtent=[0.22,0.015,0.015], position=[cx,cy+0.005,cz-0.20], color='#E53935', mass=0)

    dx2, dy2, dz2 = 2.5, WY, 1.3
    bb.createBody('box', halfExtent=[0.45,0.03,0.55], position=[dx2,dy2,dz2],         color='#37474F', mass=0)
    bb.createBody('box', halfExtent=[0.40,0.02,0.50], position=[dx2,dy2+0.01,dz2],    color='#ECEFF1', mass=0)
    bb.createBody('box', halfExtent=[0.40,0.015,0.06],position=[dx2,dy2+0.015,dz2+0.42], color='#455A64', mass=0)
    bb.createBody('sphere', radius=0.055,              position=[dx2,dy2+0.02,dz2+0.22], color='#FDBCB4', mass=0)
    bb.createBody('box', halfExtent=[0.04,0.015,0.12], position=[dx2,dy2+0.02,dz2+0.04], color='#E8B4B8', mass=0)
    bb.createBody('box', halfExtent=[0.03,0.015,0.09], position=[dx2-0.07,dy2+0.02,dz2-0.10],color='#CFD8DC',mass=0)
    bb.createBody('box', halfExtent=[0.03,0.015,0.09], position=[dx2+0.07,dy2+0.02,dz2-0.10],color='#CFD8DC',mass=0)
    bb.createBody('box', halfExtent=[0.15,0.01,0.30],  position=[dx2-0.22,dy2+0.01,dz2+0.05],color='#90A4AE',mass=0)
    bb.createBody('box', halfExtent=[0.15,0.01,0.30],  position=[dx2+0.22,dy2+0.01,dz2+0.05],color='#90A4AE',mass=0)
    for iz2 in [0.30, -0.20]:
        bb.createBody('box', halfExtent=[0.22,0.01,0.02], position=[dx2,dy2+0.01,dz2+iz2], color='#90A4AE', mass=0)

    ex, ey, ez = LX, -1.0, 1.5
    bb.createBody('box', halfExtent=[0.03,0.38,0.48], position=[ex,ey,ez],         color='#E65100', mass=0)
    bb.createBody('box', halfExtent=[0.02,0.33,0.43], position=[ex+0.01,ey,ez],    color='#FFF3E0', mass=0)
    bb.createBody('sphere', radius=0.04,               position=[ex+0.02,ey,ez+0.28],    color='#FF6D00', mass=0)
    bb.createBody('box', halfExtent=[0.015,0.08,0.015],position=[ex+0.02,ey-0.12,ez+0.20],color='#FF8F00',mass=0)
    bb.createBody('sphere', radius=0.03,               position=[ex+0.02,ey-0.22,ez+0.20],color='#FFA000',mass=0)
    bb.createBody('box', halfExtent=[0.015,0.10,0.015],position=[ex+0.02,ey-0.28,ez+0.10],color='#FFB300',mass=0)
    bb.createBody('sphere', radius=0.025,              position=[ex+0.02,ey-0.32,ez-0.02],color='#FFC107',mass=0)
    bb.createBody('box', halfExtent=[0.005,0.04,0.005],position=[ex+0.02,ey+0.20,ez-0.20],color='#FF3D00',mass=0)
    bb.createBody('box', halfExtent=[0.005,0.005,0.04],position=[ex+0.02,ey+0.20,ez-0.20],color='#00E676',mass=0)

    fx, fy, fz = LX, 1.5, 1.80
    bb.createBody('box', halfExtent=[0.02,0.22,0.22], position=[fx,fy,fz],         color='#1B5E20', mass=0)
    bb.createBody('box', halfExtent=[0.015,0.18,0.18],position=[fx+0.005,fy,fz],   color='#C8E6C9', mass=0)
    bb.createBody('box', halfExtent=[0.01,0.12,0.03], position=[fx+0.01,fy,fz],    color='#2E7D32', mass=0)
    bb.createBody('box', halfExtent=[0.01,0.03,0.12], position=[fx+0.01,fy,fz],    color='#2E7D32', mass=0)
    bb.createBody('box', halfExtent=[0.01,0.22,0.02], position=[fx+0.005,fy,fz+0.20],  color='#388E3C', mass=0)
    bb.createBody('box', halfExtent=[0.01,0.22,0.02], position=[fx+0.005,fy,fz-0.20],  color='#388E3C', mass=0)
    bb.createBody('box', halfExtent=[0.01,0.02,0.22], position=[fx+0.005,fy+0.20,fz],  color='#388E3C', mass=0)
    bb.createBody('box', halfExtent=[0.01,0.02,0.22], position=[fx+0.005,fy-0.20,fz],  color='#388E3C', mass=0)

    gx, gy, gz = LX, 3.2, 1.60
    bb.createBody('box', halfExtent=[0.02,0.28,0.28], position=[gx,gy,gz],          color='#212121', mass=0)
    bb.createBody('box', halfExtent=[0.015,0.24,0.24],position=[gx+0.005,gy,gz],    color='#1A1A2E', mass=0)
    bb.createBody('sphere', radius=0.09,               position=[gx+0.01,gy,gz+0.02],color='#00E5FF', mass=0)
    bb.createBody('sphere', radius=0.06,               position=[gx+0.012,gy,gz+0.02],color='#0097A7',mass=0)
    bb.createBody('sphere', radius=0.03,               position=[gx+0.015,gy,gz+0.02],color='#006064',mass=0)
    for sl in [-0.18, -0.10, 0.10, 0.18]:
        bb.createBody('box', halfExtent=[0.005,0.22,0.005], position=[gx+0.01,gy,gz+sl], color='#00BCD4', mass=0)

    hx, hy, hz = RX, -1.0, 1.4
    bb.createBody('box', halfExtent=[0.03,0.40,0.55], position=[hx,hy,hz],          color='#4A148C', mass=0)
    bb.createBody('box', halfExtent=[0.02,0.35,0.50], position=[hx-0.01,hy,hz],     color='#F3E5F5', mass=0)
    bb.createBody('sphere', radius=0.055, position=[hx-0.02,hy,hz+0.30],            color='#FDBCB4', mass=0)
    bb.createBody('box', halfExtent=[0.015,0.06,0.12],position=[hx-0.02,hy,hz+0.10],color='#CE93D8', mass=0)
    bb.createBody('box', halfExtent=[0.015,0.04,0.09],position=[hx-0.02,hy-0.10,hz+0.00],color='#BA68C8',mass=0)
    bb.createBody('box', halfExtent=[0.015,0.04,0.09],position=[hx-0.02,hy+0.10,hz+0.00],color='#BA68C8',mass=0)
    bb.createBody('box', halfExtent=[0.015,0.04,0.09],position=[hx-0.02,hy-0.04,hz-0.14],color='#CE93D8',mass=0)
    bb.createBody('box', halfExtent=[0.015,0.04,0.09],position=[hx-0.02,hy+0.04,hz-0.14],color='#CE93D8',mass=0)
    for sdz in [0.32, 0.12, -0.04]:
        bb.createBody('sphere', radius=0.018, position=[hx-0.025,hy,hz+sdz], color='#E040FB', mass=0)
    for lz2 in [0.32, 0.12, -0.04, -0.20]:
        bb.createBody('box', halfExtent=[0.01,0.10,0.005], position=[hx-0.02,hy+0.22,hz+lz2], color='#CE93D8', mass=0)

    ix2, iy2, iz3 = RX, 1.8, 1.85
    bb.createBody('box', halfExtent=[0.02,0.24,0.24], position=[ix2,iy2,iz3],        color='#F57F17', mass=0)
    bb.createBody('box', halfExtent=[0.015,0.20,0.20],position=[ix2-0.005,iy2,iz3],  color='#FFFDE7', mass=0)
    bb.createBody('sphere', radius=0.025, position=[ix2-0.01,iy2-0.07,iz3+0.06], color='#F57F17', mass=0)
    bb.createBody('sphere', radius=0.025, position=[ix2-0.01,iy2+0.07,iz3+0.06], color='#F57F17', mass=0)
    bb.createBody('box', halfExtent=[0.01,0.10,0.008],position=[ix2-0.01,iy2,iz3-0.06],color='#F57F17',mass=0)
    bb.createBody('box', halfExtent=[0.01,0.01,0.06], position=[ix2-0.01,iy2,iz3+0.19], color='#FFA000', mass=0)
    bb.createBody('sphere', radius=0.018,             position=[ix2-0.01,iy2,iz3+0.25], color='#FF6F00', mass=0)
    bb.createBody('sphere', radius=0.018, position=[ix2-0.01,iy2-0.21,iz3+0.04], color='#FFA000', mass=0)
    bb.createBody('sphere', radius=0.018, position=[ix2-0.01,iy2+0.21,iz3+0.04], color='#FFA000', mass=0)

    jx, jy, jz = RX, 3.5, 1.30
    bb.createBody('box', halfExtent=[0.02,0.35,0.42], position=[jx,jy,jz],          color='#1B5E20', mass=0)
    bb.createBody('box', halfExtent=[0.015,0.30,0.37],position=[jx-0.005,jy,jz],    color='#2E7D32', mass=0)
    for row in [-0.25, -0.10, 0.05, 0.20]:
        bb.createBody('box', halfExtent=[0.005,0.25,0.008], position=[jx-0.01,jy,jz+row],   color='#69F0AE', mass=0)
    for col in [-0.18, 0.0, 0.18]:
        bb.createBody('box', halfExtent=[0.005,0.008,0.30], position=[jx-0.01,jy+col,jz],   color='#69F0AE', mass=0)
    for cx2,cy2 in [(-0.10,-0.10),(0.08,0.12),(-0.08,0.18),(0.05,-0.20)]:
        bb.createBody('box', halfExtent=[0.005,0.06,0.04], position=[jx-0.01,jy+cy2,jz+cx2], color='#212121', mass=0)
    for lx2,ly2 in [(-0.28,-0.12),(0.15,0.22),(0.28,-0.05)]:
        bb.createBody('sphere', radius=0.015, position=[jx-0.01,jy+ly2,jz+lx2], color='#00E676', mass=0)


# ═══════════════════════════════════════════════════════════════════════════════
#  INSTRUMENT TABLE
# ═══════════════════════════════════════════════════════════════════════════════
def build_instrument_table(tx, ty, tz=0):
    S = '#80CBC4'; D = '#00897B'
    tt = tz + 0.85
    iz = tt + 0.022
    for lx,ly in [(-0.28,-0.18),(0.28,-0.18),(-0.28,0.18),(0.28,0.18)]:
        bb.createBody('box', halfExtent=[0.015,0.015,0.42], position=[tx+lx,ty+ly,tz+0.42], color=S, mass=0)
    bb.createBody('box', halfExtent=[0.26,0.16,0.012], position=[tx,ty,tz+0.30], color=D, mass=0)
    bb.createBody('box', halfExtent=[0.30,0.20,0.015], position=[tx,ty,tt],      color=S, mass=0)
    bb.createBody('box', halfExtent=[0.30,0.012,0.018],position=[tx,ty-0.20,tt+0.018], color=D, mass=0)
    bb.createBody('box', halfExtent=[0.30,0.012,0.018],position=[tx,ty+0.20,tt+0.018], color=D, mass=0)
    bb.createBody('box', halfExtent=[0.012,0.20,0.018],position=[tx-0.30,ty,tt+0.018],  color=D, mass=0)
    bb.createBody('box', halfExtent=[0.012,0.20,0.018],position=[tx+0.30,ty,tt+0.018],  color=D, mass=0)
    bb.createBody('box', halfExtent=[0.012,0.012,0.44],position=[tx,ty,tz+0.44],        color=S, mass=0)
    bb.createBody('box', halfExtent=[0.008,0.065,0.008], position=[tx-0.13,ty-0.06,iz+0.008], color='#FAFAFA', mass=0)
    bb.createBody('box', halfExtent=[0.018,0.008,0.005], position=[tx-0.13,ty-0.14,iz+0.005], color='#90A4AE', mass=0)
    bb.createBody('box', halfExtent=[0.003,0.022,0.003], position=[tx-0.13,ty+0.03, iz+0.004], color='#CFD8DC', mass=0)
    bb.createBody('box',    halfExtent=[0.028,0.018,0.060], position=[tx+0.00,ty-0.04,iz+0.060], color='#E3F2FD', mass=0)
    bb.createBody('box',    halfExtent=[0.022,0.013,0.042], position=[tx+0.00,ty-0.04,iz+0.052], color='#81D4FA', mass=0)
    bb.createBody('box',    halfExtent=[0.014,0.014,0.009], position=[tx+0.00,ty-0.04,iz+0.145], color='#1565C0', mass=0)


# ═══════════════════════════════════════════════════════════════════════════════
#  DOCTOR WORKSTATION
# ═══════════════════════════════════════════════════════════════════════════════
def build_doctor_workstation(dx, dy, dz=0):
    SC='#FDBCB4'; WC='#E8F5E9'; PC='#1A237E'; HC='#3E2723'
    GC='#B2DFDB'; DSK='#5D4037'; DKT='#C8A97A'; CHR='#212121'
    MON='#111111'

    DESK_TOP = dz + 0.79
    bb.createBody('box', halfExtent=[0.70,0.35,0.03],  position=[dx,dy,dz+0.76],       color=DKT, mass=0)
    bb.createBody('box', halfExtent=[0.70,0.03,0.32],  position=[dx,dy-0.35,dz+0.44],  color=DSK, mass=0)
    bb.createBody('box', halfExtent=[0.03,0.35,0.32],  position=[dx-0.70,dy,dz+0.44],  color=DSK, mass=0)
    bb.createBody('box', halfExtent=[0.03,0.35,0.32],  position=[dx+0.70,dy,dz+0.44],  color=DSK, mass=0)
    bb.createBody('box', halfExtent=[0.70,0.03,0.32],  position=[dx,dy+0.35,dz+0.44],  color=DSK, mass=0)
    for fx,fy in [(-0.62,-0.28),(0.62,-0.28),(-0.62,0.28),(0.62,0.28)]:
        bb.createBody('box', halfExtent=[0.04,0.04,0.12], position=[dx+fx,dy+fy,dz+0.12], color='#424242', mass=0)

    for mx, scr_color in [(-0.32,'#1565C0'),(0.32,'#004D40')]:
        bb.createBody('box', halfExtent=[0.10,0.06,0.015],  position=[dx+mx,dy+0.10,DESK_TOP+0.01],  color='#555555', mass=0)
        bb.createBody('box', halfExtent=[0.022,0.022,0.18], position=[dx+mx,dy+0.10,DESK_TOP+0.19],  color='#444444', mass=0)
        bb.createBody('box', halfExtent=[0.26,0.03,0.17],   position=[dx+mx,dy+0.10,DESK_TOP+0.42],  color=MON,       mass=0)
        bb.createBody('box', halfExtent=[0.23,0.012,0.14],  position=[dx+mx,dy+0.09,DESK_TOP+0.42],  color=scr_color, mass=0)
        bb.createBody('box', halfExtent=[0.16,0.004,0.008], position=[dx+mx,dy+0.076,DESK_TOP+0.50], color='#00E5FF', mass=0)
        bb.createBody('box', halfExtent=[0.16,0.004,0.008], position=[dx+mx,dy+0.076,DESK_TOP+0.47], color='#69F0AE', mass=0)

    bb.createBody('box', halfExtent=[0.22,0.09,0.010], position=[dx,dy-0.08,DESK_TOP+0.012], color='#37474F', mass=0)
    bb.createBody('box', halfExtent=[0.030,0.045,0.014],position=[dx+0.30,dy-0.06,DESK_TOP+0.014], color='#424242', mass=0)

    bb.createBody('box', halfExtent=[0.14,0.10,0.018], position=[dx-0.44,dy-0.06,DESK_TOP+0.018], color='#263238', mass=0)
    bb.createBody('box',    halfExtent=[0.020,0.020,0.030], position=[dx-0.52,dy-0.06,DESK_TOP+0.048], color='#37474F', mass=0)
    bb.createBody('sphere', radius=0.022,                   position=[dx-0.52,dy-0.06,DESK_TOP+0.080], color='#FF5722', mass=0)
    for lx,lc in [(-0.38,'#00E676'),(-0.34,'#FF1744'),(-0.30,'#FFEA00')]:
        bb.createBody('sphere', radius=0.012, position=[dx+lx,dy-0.12,DESK_TOP+0.030], color=lc, mass=0)

    CY = dy + 0.65
    bb.createBody('box', halfExtent=[0.22,0.22,0.04],  position=[dx,CY,dz+0.50], color=CHR,      mass=0)
    bb.createBody('box', halfExtent=[0.20,0.20,0.03],  position=[dx,CY,dz+0.54], color='#1A237E',mass=0)
    bb.createBody('box', halfExtent=[0.20,0.04,0.26],  position=[dx,CY+0.22,dz+0.76], color=CHR, mass=0)
    bb.createBody('box', halfExtent=[0.03,0.03,0.22],  position=[dx,CY,dz+0.27],      color='#616161', mass=0)
    for i in range(5):
        a = i*(2*math.pi/5)
        bb.createBody('box', halfExtent=[0.10,0.025,0.015],
                      position=[dx+0.20*math.cos(a),CY+0.20*math.sin(a),dz+0.04], color='#424242', mass=0)

    SY = CY; SZ = dz + 0.57
    bb.createBody('box', halfExtent=[0.07,0.22,0.07], position=[dx-0.09,SY-0.22,SZ+0.02], color=PC, mass=0)
    bb.createBody('box', halfExtent=[0.07,0.22,0.07], position=[dx+0.09,SY-0.22,SZ+0.02], color=PC, mass=0)
    bb.createBody('box', halfExtent=[0.06,0.06,0.24], position=[dx-0.09,SY-0.42,SZ-0.22], color=PC, mass=0)
    bb.createBody('box', halfExtent=[0.06,0.06,0.24], position=[dx+0.09,SY-0.42,SZ-0.22], color=PC, mass=0)
    bb.createBody('box', halfExtent=[0.16,0.12,0.06], position=[dx,SY,SZ+0.04],            color=PC, mass=0)
    bb.createBody('box', halfExtent=[0.18,0.13,0.26], position=[dx,SY-0.06,SZ+0.30],       color=WC, mass=0)
    bb.createBody('box', halfExtent=[0.09,0.012,0.012], position=[dx,SY-0.10,SZ+0.54],      color='#37474F', mass=0)
    bb.createBody('box', halfExtent=[0.012,0.012,0.08], position=[dx-0.09,SY-0.06,SZ+0.48], color='#37474F', mass=0)
    bb.createBody('sphere', radius=0.020,               position=[dx-0.09,SY-0.06,SZ+0.40], color='#546E7A', mass=0)
    bb.createBody('box', halfExtent=[0.038,0.038,0.055], position=[dx,SY-0.07,SZ+0.60],   color=SC, mass=0)
    bb.createBody('sphere', radius=0.12,                  position=[dx,SY-0.09,SZ+0.76],   color=SC, mass=0)
    bb.createBody('box', halfExtent=[0.12,0.09,0.04],     position=[dx,SY-0.06,SZ+0.87],   color=HC, mass=0)
    bb.createBody('box', halfExtent=[0.085,0.014,0.050],  position=[dx,SY-0.185,SZ+0.71],  color='#81D4FA', mass=0)


# ═══════════════════════════════════════════════════════════════════════════════
#  LOKOMAT NX FRAME
# ═══════════════════════════════════════════════════════════════════════════════
def setup_lokomat(X, Y, G=0):
    bb.createBody('box', halfExtent=[0.70,1.10,0.09], position=[X,Y,G+0.09],      color='#F5F5F5', mass=0)
    bb.createBody('box', halfExtent=[0.55,0.90,0.02], position=[X,Y,G+0.19],      color='#1C1C1C', mass=0)
    bb.createBody('box', halfExtent=[0.02,0.85,0.005],position=[X,Y,G+0.21],      color='#FFFFFF', mass=0)
    bb.createBody('box', halfExtent=[0.70,0.25,0.04], position=[X,Y-1.32,G+0.05], color='#E0E0E0', mass=0)
    bb.createBody('box', halfExtent=[0.04,1.10,0.09], position=[X-0.70,Y,G+0.09], color='#EEEEEE', mass=0)
    bb.createBody('box', halfExtent=[0.04,1.10,0.09], position=[X+0.70,Y,G+0.09], color='#EEEEEE', mass=0)

    col_h = 1.10; col_z = G+0.19+col_h
    for cx,cy in [(-0.62,0.85),(0.62,0.85),(-0.62,-0.75),(0.62,-0.75)]:
        bb.createBody('box', halfExtent=[0.045,0.045,col_h],
                      position=[X+cx,Y+cy,col_z], color='#E0E0E0', mass=0)
        bb.createBody('box', halfExtent=[0.07,0.07,0.04],
                      position=[X+cx,Y+cy,G+0.22], color='#BDBDBD', mass=0)

    top_z = G+0.19+col_h*2-0.04; mid_z = G+0.19+col_h
    bb.createBody('box', halfExtent=[0.66,0.045,0.045], position=[X,Y+0.85,top_z],      color='#E0E0E0', mass=0)
    bb.createBody('box', halfExtent=[0.66,0.045,0.045], position=[X,Y-0.75,top_z],      color='#E0E0E0', mass=0)
    bb.createBody('box', halfExtent=[0.045,0.80,0.045], position=[X-0.62,Y+0.05,top_z], color='#E0E0E0', mass=0)
    bb.createBody('box', halfExtent=[0.045,0.80,0.045], position=[X+0.62,Y+0.05,top_z], color='#E0E0E0', mass=0)
    bb.createBody('box', halfExtent=[0.66,0.03,0.03],   position=[X,Y+0.85,mid_z],      color='#BDBDBD', mass=0)
    bb.createBody('box', halfExtent=[0.66,0.03,0.03],   position=[X,Y-0.75,mid_z],      color='#BDBDBD', mass=0)

    bb.createBody('box', halfExtent=[0.68,0.82,0.04], position=[X,Y+0.05,top_z+0.06], color='#F5F5F5', mass=0)

    mon_y=Y+0.85; mon_z=mid_z+0.25
    pole_half=(mon_z-0.24-(G+0.19))/2.0; pole_centre=(G+0.19)+pole_half
    bb.createBody('box', halfExtent=[0.04,0.04,pole_half], position=[X,mon_y+0.04,pole_centre], color='#9E9E9E', mass=0)
    bb.createBody('box', halfExtent=[0.38,0.05,0.24],      position=[X,mon_y+0.07,mon_z],       color='#212121', mass=0)
    bb.createBody('box', halfExtent=[0.36,0.01,0.22],      position=[X,mon_y+0.03,mon_z],       color='#1565C0', mass=0)
    bb.createBody('box', halfExtent=[0.30,0.005,0.18],     position=[X,mon_y+0.02,mon_z],       color='#64B5F6', mass=0)

    bws_z=top_z-0.10
    bb.createBody('box', halfExtent=[0.18,0.12,0.09], position=[X,Y+0.05,bws_z],           color='#546E7A', mass=0)
    bb.createBody('box', halfExtent=[0.02,0.02,0.45], position=[X-0.10,Y+0.05,bws_z-0.54], color='#BDBDBD', mass=0)
    bb.createBody('box', halfExtent=[0.02,0.02,0.45], position=[X+0.10,Y+0.05,bws_z-0.54], color='#BDBDBD', mass=0)
    bb.createBody('box', halfExtent=[0.17,0.11,0.18], position=[X,Y+0.05,bws_z-0.85],      color='#212121', mass=0)

    hip_z=G+0.19+0.85
    bb.createBody('box', halfExtent=[0.24,0.14,0.16], position=[X,Y+0.05,hip_z],      color='#37474F', mass=0)
    bb.createBody('box', halfExtent=[0.05,0.05,0.10], position=[X-0.29,Y+0.05,hip_z], color='#455A64', mass=0)
    bb.createBody('box', halfExtent=[0.05,0.05,0.10], position=[X+0.29,Y+0.05,hip_z], color='#455A64', mass=0)

    rail_z=G+0.19+0.78
    bb.createBody('box', halfExtent=[0.03,0.35,0.025], position=[X-0.62,Y+0.20,rail_z],     color='#E0E0E0', mass=0)
    bb.createBody('box', halfExtent=[0.03,0.35,0.025], position=[X+0.62,Y+0.20,rail_z],     color='#E0E0E0', mass=0)
    bb.createBody('sphere', radius=0.04,               position=[X-0.62,Y+0.45,rail_z+0.04],color='#F44336', mass=0)
    bb.createBody('sphere', radius=0.04,               position=[X+0.62,Y+0.45,rail_z+0.04],color='#F44336', mass=0)


# ═══════════════════════════════════════════════════════════════════════════════
#  LOKOMAT ANIMATED LEGS
# ═══════════════════════════════════════════════════════════════════════════════
def build_lokomat_legs(t, X, Y, G=0):
    ids = []
    hip_z = G+0.19+0.85
    SWING_AMP=0.13; ABDUCT_AMP=0.025; KNEE_AMP=0.14; ANKLE_AMP=0.04

    for base_x, phase, clamp_sign in [(X-0.18,0.0,+0.08),(X+0.18,math.pi,-0.08)]:
        hip_swing  = SWING_AMP  * math.sin(t+phase)
        hip_abduct = ABDUCT_AMP * math.sin(t+phase+1.0)
        knee_flex  = KNEE_AMP   * max(0.0, math.sin(t+phase+math.pi*0.55))
        ankle_df   = ANKLE_AMP  * math.sin(t+phase+math.pi*0.8)

        SX = base_x+hip_abduct; SY = Y+0.05+hip_swing

        ids.append(bb.createBody('box', halfExtent=[0.03,0.03,0.06],   position=[SX,SY,hip_z-0.22],     color='#90A4AE', mass=0))
        ids.append(bb.createBody('box', halfExtent=[0.12,0.10,0.20],   position=[SX,SY,hip_z-0.46],     color='#CFD8DC', mass=0))
        ids.append(bb.createBody('box', halfExtent=[0.08,0.07,0.16],   position=[SX,SY-0.03,hip_z-0.46],color='#90A4AE', mass=0))
        ids.append(bb.createBody('box', halfExtent=[0.13,0.02,0.025],  position=[SX,SY-0.05,hip_z-0.32],color='#ECEFF1', mass=0))
        ids.append(bb.createBody('box', halfExtent=[0.13,0.02,0.025],  position=[SX,SY-0.05,hip_z-0.60],color='#ECEFF1', mass=0))

        knee_z = hip_z-0.74-knee_flex*0.30
        ids.append(bb.createBody('box',    halfExtent=[0.05,0.05,0.05], position=[SX,SY,knee_z],              color='#546E7A', mass=0))
        ids.append(bb.createBody('sphere', radius=0.03,                 position=[SX+clamp_sign,SY,knee_z],   color='#78909C', mass=0))

        shank_z = knee_z-0.17-knee_flex*0.38
        ids.append(bb.createBody('box', halfExtent=[0.025,0.025,0.17], position=[SX,SY,shank_z], color='#CFD8DC', mass=0))

        shin_up_z = shank_z+0.14
        ids.append(bb.createBody('box', halfExtent=[0.09,0.02,0.025], position=[SX,SY-0.05,shin_up_z],        color='#9E9E9E', mass=0))
        ids.append(bb.createBody('box', halfExtent=[0.02,0.06,0.025], position=[SX+clamp_sign,SY-0.01,shin_up_z],color='#9E9E9E',mass=0))
        ids.append(bb.createBody('box', halfExtent=[0.02,0.06,0.025], position=[SX+clamp_sign,SY-0.09,shin_up_z],color='#9E9E9E',mass=0))

        ankle_z = shank_z-0.14
        ids.append(bb.createBody('box', halfExtent=[0.09,0.02,0.025], position=[SX,SY-0.05,ankle_z],          color='#9E9E9E', mass=0))
        ids.append(bb.createBody('box', halfExtent=[0.02,0.06,0.025], position=[SX+clamp_sign,SY-0.01,ankle_z],color='#9E9E9E',mass=0))
        ids.append(bb.createBody('box', halfExtent=[0.02,0.06,0.025], position=[SX+clamp_sign,SY-0.09,ankle_z],color='#9E9E9E',mass=0))

        foot_z = max(ankle_z-0.12+ankle_df, G+0.21)
        ids.append(bb.createBody('box', halfExtent=[0.09,0.13,0.02], position=[SX,SY,foot_z], color='#78909C', mass=0))

    return ids


# ═══════════════════════════════════════════════════════════════════════════════
#  AUTO SURGICAL ROBOT — computes its own animated state (no sliders needed)
# ═══════════════════════════════════════════════════════════════════════════════

def _compute_auto_surgical_state(t):
    """Generate smooth automatic motion for all 4 surgical arms."""
    # Base position stays fixed over patient
    px = 0.0
    py = 1.5
    pz = 0.0
    col_h = 1.28

    s = {
        'robot_x': px, 'robot_y': py, 'robot_z': pz,
        'column_height': col_h,
    }

    # Each arm gets an independent oscillating motion
    # arm1: right side, sweeping Y
    s['arm1_shoulder_x'] =  0.18 + 0.08 * math.sin(t * 0.7)
    s['arm1_shoulder_y'] =  0.00 + 0.15 * math.sin(t * 0.5 + 0.0)
    s['arm1_reach']      =  0.58 + 0.12 * math.sin(t * 0.6 + 1.0)
    s['arm1_elbow_drop'] =  0.30 + 0.10 * math.sin(t * 0.8 + 0.5)

    # arm2: left side, counter-phase
    s['arm2_shoulder_x'] = -0.18 - 0.08 * math.sin(t * 0.7)
    s['arm2_shoulder_y'] =  0.00 + 0.15 * math.sin(t * 0.5 + math.pi)
    s['arm2_reach']      =  0.58 + 0.12 * math.sin(t * 0.6 + 2.0)
    s['arm2_elbow_drop'] =  0.30 + 0.10 * math.sin(t * 0.8 + 1.5)

    # arm3: top center, gentle rotation
    s['arm3_shoulder_x'] =  0.10 * math.cos(t * 0.4 + 0.0)
    s['arm3_shoulder_y'] =  0.10 * math.sin(t * 0.4 + 0.0)
    s['arm3_reach']      =  0.18 + 0.06 * math.sin(t * 0.9)
    s['arm3_elbow_drop'] =  0.30 + 0.08 * math.sin(t * 0.6 + 2.5)

    # arm4: right offset, slower sweep
    s['arm4_shoulder_x'] =  0.14 + 0.06 * math.sin(t * 0.3 + 1.0)
    s['arm4_shoulder_y'] =  0.00 + 0.10 * math.sin(t * 0.4 + 0.8)
    s['arm4_reach']      =  0.42 + 0.10 * math.sin(t * 0.5 + 3.0)
    s['arm4_elbow_drop'] =  0.40 + 0.08 * math.sin(t * 0.7 + 1.2)

    return s


def _arm_tip(s, n):
    px=s['robot_x']; py=s['robot_y']; pz=s['robot_z']
    hub_z = pz+s['column_height']
    sx=s[f'arm{n}_shoulder_x']; sy=s[f'arm{n}_shoulder_y']
    reach=s[f'arm{n}_reach'];   drop=s[f'arm{n}_elbow_drop']
    mag=abs(sx)+abs(sy)+0.001; dx=sx/mag; dy=sy/mag
    ax=px+sx; ay=py+sy; az=hub_z
    ix=ax+dx*reach; iy=ay+dy*reach; iz=az-drop-0.62
    return [ix, iy, iz-0.155]


def _compute_robot_positions(s):
    px=s['robot_x']; py=s['robot_y']; pz=s['robot_z']
    hub_z = pz+s['column_height']
    pos = []
    pos.append([px, py, hub_z])
    pos.append([px, py, pz+2.32])
    pos.append([px, py, pz+1.84])
    pos.append([px+0.44, py, pz+2.32])
    for lx in [-0.12,0.0,0.12]:
        for ly in [-0.09,0.09]:
            pos.append([px+0.44+lx, py+ly, pz+2.29])
    for n in [1,2,3,4]:
        sx=s[f'arm{n}_shoulder_x']; sy=s[f'arm{n}_shoulder_y']
        reach=s[f'arm{n}_reach'];   drop=s[f'arm{n}_elbow_drop']
        mag=abs(sx)+abs(sy)+0.001; dx=sx/mag; dy=sy/mag
        ax=px+sx; ay=py+sy; az=hub_z
        pos.append([ax,              ay,              az         ])
        pos.append([ax+dx*0.22,      ay+dy*0.22,      az-0.10    ])
        pos.append([ax+dx*0.38,      ay+dy*0.38,      az-drop    ])
        pos.append([ax+dx*0.55,      ay+dy*0.55,      az-drop-0.24])
        pos.append([ax+dx*reach*0.9, ay+dy*reach*0.9, az-drop-0.46])
        ix=ax+dx*reach; iy=ay+dy*reach; iz=az-drop-0.62
        pos.append([ix, iy, iz      ])
        pos.append([ix, iy, iz+0.045])
        pos.append([ix, iy, iz-0.115])
        px2=-dy*0.030; py2=dx*0.030
        pos.append([ix+px2, iy+py2, iz-0.155])
        pos.append([ix-px2, iy-py2, iz-0.155])
    return pos


def build_surgical_robot(s0):
    CW='#ECEFF1'; CG='#546E7A'; CS='#90A4AE'; CB='#42A5F5'; CD='#263238'
    ids = []
    def _b(shape, pos, **kw):
        bid = bb.createBody(shape, position=pos, **kw, mass=0)
        ids.append(bid)

    px0=s0['robot_x']; py0=s0['robot_y']; pz0=s0['robot_z']
    hub_z0 = pz0+s0['column_height']

    _b('box',   [px0,py0,hub_z0],        halfExtent=[0.22,0.22,0.10], color=CW)
    _b('box',   [px0,py0,pz0+2.32],      halfExtent=[0.65,0.06,0.04], color=CG)
    _b('box',   [px0,py0,pz0+1.84],      halfExtent=[0.04,0.04,0.50], color=CG)
    _b('box',   [px0+0.44,py0,pz0+2.32], halfExtent=[0.24,0.24,0.04], color='#ECEFF1')
    for lx in [-0.12,0.0,0.12]:
        for ly in [-0.09,0.09]:
            _b('sphere',[px0+0.44+lx,py0+ly,pz0+2.29], radius=0.022, color='#E3F2FD')

    for n in [1,2,3,4]:
        sx=s0[f'arm{n}_shoulder_x']; sy=s0[f'arm{n}_shoulder_y']
        reach=s0[f'arm{n}_reach'];   drop=s0[f'arm{n}_elbow_drop']
        mag=abs(sx)+abs(sy)+0.001; dx=sx/mag; dy=sy/mag
        ax=px0+sx; ay=py0+sy; az=hub_z0
        _b('box',    [ax,ay,az],                             halfExtent=[0.07,0.07,0.07],   color=CG)
        _b('box',    [ax+dx*0.22,ay+dy*0.22,az-0.10],       halfExtent=[0.04,0.04,0.18],   color=CW)
        _b('sphere', [ax+dx*0.38,ay+dy*0.38,az-drop],       radius=0.055,                  color=CG)
        _b('box',    [ax+dx*0.55,ay+dy*0.55,az-drop-0.24],  halfExtent=[0.032,0.032,0.20], color=CS)
        _b('sphere', [ax+dx*reach*0.9,ay+dy*reach*0.9,az-drop-0.46], radius=0.040,         color=CG)
        ix=ax+dx*reach; iy=ay+dy*reach; iz=az-drop-0.62
        _b('box',    [ix,iy,iz],       halfExtent=[0.055,0.038,0.09], color=CW)
        _b('box',    [ix,iy,iz+0.045], halfExtent=[0.060,0.043,0.022],color=CB)
        _b('box',    [ix,iy,iz-0.115], halfExtent=[0.007,0.007,0.095],color=CD)
        px2=-dy*0.030; py2=dx*0.030
        _b('box', [ix+px2,iy+py2,iz-0.155], halfExtent=[0.006,0.006,0.040], color='#78909C')
        _b('box', [ix-px2,iy-py2,iz-0.155], halfExtent=[0.006,0.006,0.040], color='#78909C')

    return ids


# ═══════════════════════════════════════════════════════════════════════════════
#  AUTO PANDA JOINT POSES — cycles through clinical motion sequences
# ═══════════════════════════════════════════════════════════════════════════════

# Named joint-space poses for the Panda arm (7 joints)
PANDA_POSES = [
    # Home / ready
    [0.0,  -0.785, 0.0, -2.356, 0.0,  1.571, 0.785],
    # Reach toward patient torso
    [0.3,  -0.5,   0.1, -2.0,   0.0,  1.8,   0.9  ],
    # Extended reach right
    [0.6,  -0.3,   0.2, -1.8,  -0.2,  2.0,   1.0  ],
    # Elevated check position
    [0.0,  -1.0,   0.0, -1.5,   0.0,  1.3,   0.785],
    # Reach left across bed
    [-0.4, -0.6,  -0.1, -2.1,   0.1,  1.9,   0.7  ],
    # Low tuck
    [0.0,  -0.2,   0.0, -2.8,   0.0,  2.6,   0.785],
]

POSE_HOLD_TICKS = 80   # ticks to hold each pose before blending
POSE_BLEND_TICKS = 40  # ticks to blend between poses


def _lerp(a, b, frac):
    return [a[i] + (b[i]-a[i])*frac for i in range(len(a))]


def _smooth_step(x):
    """Smooth interpolation curve (ease in-out)."""
    x = max(0.0, min(1.0, x))
    return x * x * (3 - 2 * x)


# ═══════════════════════════════════════════════════════════════════════════════
#  BUILD EVERYTHING
# ═══════════════════════════════════════════════════════════════════════════════
setup_hospital_room()
build_wall_stickers()

TX, TY = 0.9, 0.3
build_instrument_table(tx=TX, ty=TY, tz=0)
TABLE_TOP_Z = 0.872

build_doctor_workstation(dx=4.2, dy=1.2, dz=0)

LK_X, LK_Y = -3.5, 1.5
setup_lokomat(LK_X, LK_Y)

# Panda arm on small plinth
bb.createBody('box', halfExtent=[0.25,0.25,0.03], position=[-0.8,1.5,0.03], color='#444444', mass=0)
panda = bb.loadURDF('panda.urdf', [-0.8, 1.5, 0.0], fixedBase=True)

panda_joints = []
for i in range(bb.getNumJoints(panda)):
    joint_name, joint_type, joint_limits = bb.getJointInfo(panda, i)
    if joint_type != 'fixed':
        panda_joints.append((i, joint_name, joint_limits))

# Surgical robot — build with initial auto-state
s0 = _compute_auto_surgical_state(0.0)
surg_ids = build_surgical_robot(s0)

# Instruments on table (auto pick-up)
_inst_ids  = []
_inst_rest = []
_inst_held = []
_arm_holds = {1:-1, 2:-1, 3:-1, 4:-1}

def _add_inst(shape, pos, **kw):
    bid = bb.createBody(shape, position=pos, **kw, mass=0)
    _inst_ids.append(bid)
    _inst_rest.append(list(pos))
    _inst_held.append(-1)

_add_inst('box', [TX-0.13,TY-0.06,TABLE_TOP_Z+0.008], halfExtent=[0.008,0.065,0.008], color='#FAFAFA')
_add_inst('box', [TX+0.00,TY-0.04,TABLE_TOP_Z+0.060], halfExtent=[0.028,0.018,0.060], color='#E3F2FD')
_add_inst('box', [TX+0.14,TY+0.04,TABLE_TOP_Z+0.050], halfExtent=[0.023,0.016,0.050], color='#FAFAFA')

PICKUP_DIST = 0.32


# ═══════════════════════════════════════════════════════════════════════════════
#  NURSE ROBOT — humanoid body that patrols the room
# ═══════════════════════════════════════════════════════════════════════════════

# Patrol waypoints — a path around the room interior (x, y)
PATROL_WAYPOINTS = [
    ( 0.0, -3.5),   # front centre (near door)
    ( 3.5, -3.5),   # front-right corner
    ( 3.5,  0.0),   # right wall mid
    ( 3.5,  3.5),   # back-right corner
    ( 0.0,  3.5),   # back centre
    (-3.5,  3.5),   # back-left corner
    (-3.5,  0.0),   # left wall mid  (near Lokomat — brief pause)
    (-3.5, -3.5),   # front-left corner
]
PATROL_SPEED   = 0.04   # units per tick
NURSE_FLOOR_Z  = 0.0    # base z (feet on floor)
NURSE_HEIGHT   = 1.55   # total robot height
PAUSE_TICKS    = 60     # ticks to pause at each waypoint


def _build_nurse_robot(rx, ry, facing_angle, walk_t, nurse_ids_out):
    """
    Build the nurse robot at position (rx, ry) facing facing_angle (radians).
    walk_t drives the walking leg/arm swing.
    Appends all created body IDs to nurse_ids_out.
    Returns list of ids (same reference).
    """
    ids = nurse_ids_out

    def _b(shape, pos, **kw):
        bid = bb.createBody(shape, position=pos, mass=0, **kw)
        ids.append(bid)

    def _rot(dx, dy, angle):
        """Rotate 2D offset by angle."""
        c, s = math.cos(angle), math.sin(angle)
        return c*dx - s*dy,  s*dx + c*dy

    def _p(lx, ly, lz):
        """Local → world position, applying facing rotation."""
        wx, wy = _rot(lx, ly, facing_angle)
        return [rx + wx, ry + wy, NURSE_FLOOR_Z + lz]

    # ── LEG SWING ──────────────────────────────────────────────────────────
    LSTEP = 0.10 * math.sin(walk_t)          # left leg forward/back
    RSTEP = 0.10 * math.sin(walk_t + math.pi)
    LKNEE = 0.06 * max(0, math.sin(walk_t + math.pi*0.55))
    RKNEE = 0.06 * max(0, math.sin(walk_t + math.pi + math.pi*0.55))

    # Left leg
    _b('box',    _p(-0.10, LSTEP,        0.62), halfExtent=[0.055,0.055,0.18], color='#E0E0E0')
    _b('box',    _p(-0.10, LSTEP-LKNEE,  0.38), halfExtent=[0.048,0.048,0.16], color='#CFD8DC')
    _b('box',    _p(-0.10, LSTEP-LKNEE*1.5, 0.16), halfExtent=[0.05, 0.08,0.06], color='#BDBDBD')  # foot

    # Right leg
    _b('box',    _p( 0.10, RSTEP,        0.62), halfExtent=[0.055,0.055,0.18], color='#E0E0E0')
    _b('box',    _p( 0.10, RSTEP-RKNEE,  0.38), halfExtent=[0.048,0.048,0.16], color='#CFD8DC')
    _b('box',    _p( 0.10, RSTEP-RKNEE*1.5, 0.16), halfExtent=[0.05, 0.08,0.06], color='#BDBDBD')  # foot

    # ── PELVIS ─────────────────────────────────────────────────────────────
    _b('box',    _p(0.0, 0.0, 0.82),  halfExtent=[0.15,0.10,0.08], color='#E0E0E0')

    # ── TORSO ──────────────────────────────────────────────────────────────
    _b('box',    _p(0.0, 0.0, 1.02),  halfExtent=[0.17,0.09,0.16], color='#F5F5F5')
    # Chest panel detail
    _b('box',    _p(0.0, -0.08, 1.06),halfExtent=[0.10,0.01,0.10], color='#E3F2FD')
    # Blue chest circle
    _b('sphere', _p(0.0, -0.09, 1.10),radius=0.04,                 color='#0097A7')
    _b('sphere', _p(0.0, -0.09, 1.10),radius=0.025,                color='#00E5FF')

    # ── STETHOSCOPE ────────────────────────────────────────────────────────
    _b('box',    _p(0.0,  -0.09, 1.17), halfExtent=[0.10,0.01,0.01], color='#424242')
    _b('box',    _p(-0.10,-0.05, 1.12), halfExtent=[0.01,0.01,0.06], color='#424242')
    _b('sphere', _p(-0.10,-0.05, 1.06), radius=0.020,                color='#546E7A')

    # ── SHOULDER JOINTS ────────────────────────────────────────────────────
    _b('sphere', _p(-0.20, 0.0, 1.16), radius=0.055, color='#9E9E9E')
    _b('sphere', _p( 0.20, 0.0, 1.16), radius=0.055, color='#9E9E9E')

    # ── LEFT ARM (tray-holding) — horizontal, level ────────────────────────
    TRAY_BOB = 0.015 * math.sin(walk_t * 0.5)   # gentle tray bob
    _b('box',    _p(-0.20, 0.0, 1.13),  halfExtent=[0.04,0.04,0.16], color='#E0E0E0')  # upper arm
    _b('sphere', _p(-0.20, 0.0, 0.96),  radius=0.038,                color='#9E9E9E')  # elbow
    _b('box',    _p(-0.30,-0.14,0.93),  halfExtent=[0.035,0.12,0.035],color='#E0E0E0') # forearm (extended forward)
    # Left hand
    _b('box',    _p(-0.30,-0.28,0.93),  halfExtent=[0.042,0.035,0.028],color='#CFD8DC')

    # ── RIGHT ARM (swings gently) ──────────────────────────────────────────
    R_ARM_SWING = 0.08 * math.sin(walk_t + math.pi)
    _b('box',    _p( 0.20, R_ARM_SWING, 1.13), halfExtent=[0.04,0.04,0.16], color='#E0E0E0')
    _b('sphere', _p( 0.20, R_ARM_SWING, 0.96), radius=0.038,                color='#9E9E9E')
    _b('box',    _p( 0.20, R_ARM_SWING-0.06, 0.80), halfExtent=[0.035,0.035,0.14], color='#E0E0E0')
    _b('box',    _p( 0.20, R_ARM_SWING-0.06, 0.64), halfExtent=[0.042,0.035,0.028], color='#CFD8DC')

    # ── MEDICINE TRAY (held by left arm, level) ───────────────────────────
    TZ = 0.93 + TRAY_BOB
    _b('box',    _p(-0.30,-0.30, TZ),        halfExtent=[0.20,0.13,0.008], color='#B0BEC5')   # tray
    _b('box',    _p(-0.20,-0.30, TZ),        halfExtent=[0.20,0.012,0.012],color='#90A4AE')   # tray edge front
    _b('box',    _p(-0.40,-0.30, TZ),        halfExtent=[0.20,0.012,0.012],color='#90A4AE')   # tray edge back
    # Medicine bottles
    _b('box',    _p(-0.22,-0.32, TZ+0.045), halfExtent=[0.025,0.025,0.045], color='white')
    _b('box',    _p(-0.22,-0.32, TZ+0.095), halfExtent=[0.018,0.018,0.015], color='#78909C')  # bottle cap
    _b('box',    _p(-0.32,-0.32, TZ+0.040), halfExtent=[0.022,0.022,0.040], color='#FFF8E1')
    _b('box',    _p(-0.32,-0.32, TZ+0.085), halfExtent=[0.016,0.016,0.012], color='#FF8F00')  # cap
    # Coloured pills
    for pi, (pox, poc) in enumerate([
        (-0.18, '#2196F3'), (-0.24, '#FF9800'), (-0.30, '#4CAF50'),
        (-0.36, '#F44336'), (-0.20, '#9C27B0'),
    ]):
        _b('sphere', _p(pox, -0.24, TZ+0.014), radius=0.014, color=poc)

    # ── NECK ───────────────────────────────────────────────────────────────
    _b('box',    _p(0.0, 0.0, 1.32), halfExtent=[0.06,0.06,0.06], color='#757575')
    # Neck joint ring
    _b('box',    _p(0.0, 0.0, 1.36), halfExtent=[0.07,0.07,0.02], color='#0097A7')

    # ── HEAD ───────────────────────────────────────────────────────────────
    _b('box',    _p(0.0, 0.0, 1.48), halfExtent=[0.14,0.11,0.14], color='#F5F5F5')
    # Face plate (front = -y in local)
    _b('box',    _p(0.0,-0.10, 1.48),halfExtent=[0.11,0.02,0.10], color='#ECEFF1')
    # Eyes — glowing cyan
    _b('sphere', _p(-0.05,-0.115,1.52), radius=0.028, color='#00BCD4')
    _b('sphere', _p(-0.05,-0.115,1.52), radius=0.016, color='#00E5FF')
    _b('sphere', _p( 0.05,-0.115,1.52), radius=0.028, color='#00BCD4')
    _b('sphere', _p( 0.05,-0.115,1.52), radius=0.016, color='#00E5FF')
    # Smile line
    _b('box',    _p(0.0,-0.115,1.44), halfExtent=[0.04,0.01,0.008], color='#90A4AE')
    # Ear circles
    _b('sphere', _p(-0.14,0.0,1.48), radius=0.030, color='#0097A7')
    _b('sphere', _p( 0.14,0.0,1.48), radius=0.030, color='#0097A7')

    # ── NURSE CAP ──────────────────────────────────────────────────────────
    _b('box',    _p(0.0, 0.0, 1.63), halfExtent=[0.13,0.09,0.03],  color='white')
    _b('box',    _p(0.0,-0.06,1.67), halfExtent=[0.10,0.06,0.025], color='white')
    # Red cross on cap
    _b('box',    _p(0.0,-0.065,1.70),halfExtent=[0.025,0.005,0.008],color='#F44336')
    _b('box',    _p(0.0,-0.065,1.70),halfExtent=[0.008,0.005,0.025],color='#F44336')

    return ids


# ── Nurse patrol state ────────────────────────────────────────────────────────
nurse_wp_idx   = 0
nurse_x        = PATROL_WAYPOINTS[0][0]
nurse_y        = PATROL_WAYPOINTS[0][1]
nurse_angle    = 0.0
nurse_pause    = 0          # countdown ticks for pause at waypoint
nurse_walk_t   = 0.0
nurse_ids      = []         # current frame body IDs


# ═══════════════════════════════════════════════════════════════════════════════
#  MAIN LOOP — all robots animate automatically every tick
# ═══════════════════════════════════════════════════════════════════════════════
t         = 0.0
tick      = 0
loko_ids  = []

# Panda pose-sequencing state
pose_idx       = 0
pose_tick      = 0
current_pose   = list(PANDA_POSES[0])
target_pose    = list(PANDA_POSES[1])

while True:
    # ── 1. PANDA ARM — automatic pose cycling ────────────────────────────────
    pose_tick += 1
    total_cycle = POSE_HOLD_TICKS + POSE_BLEND_TICKS

    if pose_tick <= POSE_HOLD_TICKS:
        # Hold current pose
        joint_vals = current_pose
    else:
        # Blend toward next pose
        blend_frac = _smooth_step((pose_tick - POSE_HOLD_TICKS) / POSE_BLEND_TICKS)
        joint_vals = _lerp(current_pose, target_pose, blend_frac)

    if pose_tick >= total_cycle:
        # Advance to next pose
        pose_tick    = 0
        pose_idx     = (pose_idx + 1) % len(PANDA_POSES)
        current_pose = list(target_pose)
        target_pose  = list(PANDA_POSES[(pose_idx + 1) % len(PANDA_POSES)])

    # Apply joint values to Panda
    for idx, (joint_idx, joint_name, joint_limits) in enumerate(panda_joints):
        if idx < len(joint_vals):
            val = max(joint_limits[0], min(joint_limits[1], joint_vals[idx]))
            bb.setJointMotorControl(panda, joint_idx, targetPosition=val)

    # ── 2. LOKOMAT — continuous walking animation ────────────────────────────
    for bid in loko_ids:
        bb.removeBody(bid)
    loko_ids.clear()
    loko_ids = build_lokomat_legs(t, LK_X, LK_Y)

    # ── 3. SURGICAL ROBOT — fully automatic motion ───────────────────────────
    s = _compute_auto_surgical_state(t)

    # Auto instrument pickup
    for arm_n in [1,2,3,4]:
        tip      = _arm_tip(s, arm_n)
        held_idx = _arm_holds[arm_n]
        if held_idx == -1:
            for i in range(len(_inst_ids)):
                if _inst_held[i] != -1:
                    continue
                rest = _inst_rest[i]
                dist = ((tip[0]-rest[0])**2+(tip[1]-rest[1])**2+(tip[2]-rest[2])**2)**0.5
                if dist < PICKUP_DIST:
                    _inst_held[i]     = arm_n
                    _arm_holds[arm_n] = i
                    break
        else:
            bb.resetBasePose(_inst_ids[held_idx], tip)

    # Update surgical robot body positions
    new_pos = _compute_robot_positions(s)
    for bid, p in zip(surg_ids, new_pos):
        bb.resetBasePose(bid, p)

    # ── 4. NURSE ROBOT — patrol around the room ──────────────────────────────
    # Remove previous frame's nurse bodies
    for bid in nurse_ids:
        bb.removeBody(bid)
    nurse_ids.clear()

    # Target waypoint
    twx, twy = PATROL_WAYPOINTS[nurse_wp_idx]
    dx_n = twx - nurse_x
    dy_n = twy - nurse_y
    dist_n = math.sqrt(dx_n*dx_n + dy_n*dy_n)

    if nurse_pause > 0:
        # Pausing at waypoint — stand still, idle sway
        nurse_pause -= 1
        nurse_walk_t += 0.02
    elif dist_n < PATROL_SPEED:
        # Reached waypoint — snap and start pause
        nurse_x      = twx
        nurse_y      = twy
        nurse_pause  = PAUSE_TICKS
        nurse_wp_idx = (nurse_wp_idx + 1) % len(PATROL_WAYPOINTS)
    else:
        # Move toward waypoint
        nurse_x += (dx_n / dist_n) * PATROL_SPEED
        nurse_y += (dy_n / dist_n) * PATROL_SPEED
        # Face direction of travel
        nurse_angle  = math.atan2(dx_n, -dy_n) + math.pi
        nurse_walk_t += 0.18   # gait speed

    # Build nurse at current position
    _build_nurse_robot(nurse_x, nurse_y, nurse_angle, nurse_walk_t, nurse_ids)

    # ── 5. Advance clocks ────────────────────────────────────────────────────
    t    += 0.07
    tick += 1
    time.sleep(0.05)
