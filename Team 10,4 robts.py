import browserbotics as bb
import time
import math

# ═══════════════════════════════════════════════════════════════════════════════
#  CONSTANTS  — single source of truth for every position
# ═══════════════════════════════════════════════════════════════════════════════
W = 6.0          # room half-width
BED_X, BED_Y = 0.0, 1.5          # bed centre XY
BED_TOP_Z    = 0.52              # top of mattress

# Patient body-part world positions
PATIENT = {
    'head':    (BED_X,       BED_Y + 0.82, BED_TOP_Z + 0.20),
    'chest':   (BED_X,       BED_Y + 0.30, BED_TOP_Z + 0.12),
    'abdomen': (BED_X,       BED_Y - 0.25, BED_TOP_Z + 0.08),
    'legs':    (BED_X,       BED_Y - 0.65, BED_TOP_Z + 0.06),
    'l_arm':   (BED_X - 0.25,BED_Y + 0.35, BED_TOP_Z + 0.18),
    'r_arm':   (BED_X + 0.25,BED_Y + 0.35, BED_TOP_Z + 0.18),
}

# Da Vinci base sits at the head of the bed, slightly above
DV_X, DV_Y, DV_BASE_Z = BED_X, BED_Y + 1.35, BED_TOP_Z + 0.40
DV_HUB_Z = DV_BASE_Z + 0.95    # central hub height

# Arm scan targets: each arm has a dedicated patient zone
ARM_TARGETS = {
    1: PATIENT['chest'],
    2: PATIENT['abdomen'],
    3: PATIENT['l_arm'],
    4: PATIENT['head'],
}

# ═══════════════════════════════════════════════════════════════════════════════
#  ASSISTANT ROBOT PATROL PATH
#  Circular orbit around the room perimeter, avoiding furniture
# ═══════════════════════════════════════════════════════════════════════════════
PATROL_RADIUS = 4.0          # radius of circular patrol path
PATROL_SPEED  = 0.30         # radians per second  (~full lap ≈ 21 s)
PATROL_Z      = 0.0          # floor level

def _patrol_pos(t_sec):
    """Return (x, y, heading_angle) for the assistant robot at time t."""
    angle   = t_sec * PATROL_SPEED          # current angle on circle
    x       = math.cos(angle) * PATROL_RADIUS
    y       = math.sin(angle) * PATROL_RADIUS
    heading = angle + math.pi / 2.0         # face tangent to circle (forward)
    return x, y, heading

# ═══════════════════════════════════════════════════════════════════════════════
#  ROOM + FURNITURE
# ═══════════════════════════════════════════════════════════════════════════════
def setup_room():
    bb.addGroundPlane()
    H = 1.5; WT = 0.05
    bb.createBody('box', halfExtent=[WT,W,H], position=[-W,0,H],  color='#F5F5F5', mass=0)
    bb.createBody('box', halfExtent=[WT,W,H], position=[ W,0,H],  color='#F5F5F5', mass=0)
    bb.createBody('box', halfExtent=[W,WT,H], position=[0, W,H],  color='#F5F5F5', mass=0)
    DW=0.55; DH=1.05; SW=W-DW
    bb.createBody('box', halfExtent=[SW,WT,H],      position=[-(DW+SW),-W,H],        color='#F5F5F5', mass=0)
    bb.createBody('box', halfExtent=[SW,WT,H],      position=[ (DW+SW),-W,H],        color='#F5F5F5', mass=0)
    bb.createBody('box', halfExtent=[DW,WT,H-DH/2], position=[0,-W,DH+H-DH/2],      color='#F5F5F5', mass=0)
    bb.createBody('box', halfExtent=[W,W,0.04], position=[0,0,H*2+0.04], color='#EEEEEE', mass=0)
    for fx in range(-5,6):
        for fy in range(-5,6):
            c = '#E8E8E8' if (fx+fy)%2==0 else '#DCDCDC'
            bb.createBody('box', halfExtent=[0.49,0.49,0.005], position=[fx,fy,0.006], color=c, mass=0)
    FC='#4E342E'; FH=0.05; DX,DY=0.0,-W+WT+0.01
    bb.createBody('box', halfExtent=[DW-FH,0.03,DH/2-FH], position=[DX,DY,DH/2],       color='#795548', mass=0)
    bb.createBody('box', halfExtent=[FH,FH,DH/2],          position=[-DW,-W,DH/2],      color=FC, mass=0)
    bb.createBody('box', halfExtent=[FH,FH,DH/2],          position=[ DW,-W,DH/2],      color=FC, mass=0)
    bb.createBody('sphere', radius=0.025, position=[DW*0.45,DY-0.01,DH*0.50], color='#DAA520', mass=0)
    WX,WY,WZ=W-0.03,2.0,1.5
    for pos,ext in [
        ([WX,WY,WZ+0.55],[0.03,0.66,0.03]),([WX,WY,WZ-0.55],[0.03,0.66,0.03]),
        ([WX,WY-0.64,WZ],[0.03,0.03,0.55]),([WX,WY+0.64,WZ],[0.03,0.03,0.55]),
    ]: bb.createBody('box', halfExtent=ext, position=pos, color='#888888', mass=0)
    bb.createBody('box', halfExtent=[0.02,0.62,0.52], position=[WX,WY,WZ], color='#B3E5FC', mass=0)
    bb.createBody('box', halfExtent=[0.35,0.35,0.03], position=[BED_X,BED_Y,2.90], color='#FFFDE7', mass=0)
    bb.createBody('box', halfExtent=[0.025,0.025,0.35], position=[BED_X,BED_Y,2.60], color='#BDBDBD', mass=0)
    bb.createBody('box', halfExtent=[0.30,0.30,0.015], position=[BED_X,BED_Y,2.88], color='#FFF9C4', mass=0)

def setup_bed_and_patient():
    bb.createBody('box', halfExtent=[0.48,1.05,0.22], position=[BED_X,BED_Y,0.22],  color='#9E9E9E', mass=0)
    bb.createBody('box', halfExtent=[0.44,0.92,0.06], position=[BED_X,BED_Y,0.48],  color='white',   mass=0)
    bb.createBody('box', halfExtent=[0.44,0.92,0.02], position=[BED_X,BED_Y,0.52],  color='#E3F2FD', mass=0)
    bb.createBody('box', halfExtent=[0.46,0.04,0.32], position=[BED_X,BED_Y+1.08,0.50], color='#6D4C41', mass=0)
    bb.createBody('box', halfExtent=[0.46,0.04,0.22], position=[BED_X,BED_Y-1.08,0.44], color='#6D4C41', mass=0)
    for lx in [-0.44, 0.44]:
        bb.createBody('box', halfExtent=[0.02,0.80,0.06], position=[lx,BED_Y+0.15,0.72], color='#BDBDBD', mass=0)
    for lx,ly in [(-0.40,BED_Y+0.95),(0.40,BED_Y+0.95),(-0.40,BED_Y-0.95),(0.40,BED_Y-0.95)]:
        bb.createBody('box', halfExtent=[0.03,0.03,0.11], position=[lx,ly,0.11], color='#757575', mass=0)
    px, py = BED_X, BED_Y
    pz = BED_TOP_Z
    bb.createBody('sphere', radius=0.13,               position=[px, py+0.82, pz+0.20], color='#FDBCB4', mass=0)
    bb.createBody('box', halfExtent=[0.11,0.09,0.04],  position=[px, py+0.87, pz+0.31], color='#4E342E', mass=0)
    bb.createBody('box', halfExtent=[0.04,0.04,0.06],  position=[px, py+0.70, pz+0.14], color='#FDBCB4', mass=0)
    bb.createBody('box', halfExtent=[0.20,0.38,0.10],  position=[px, py+0.28, pz+0.12], color='#B3E5FC', mass=0)
    bb.createBody('box', halfExtent=[0.05,0.20,0.05],  position=[px-0.26, py+0.35, pz+0.16], color='#FDBCB4', mass=0)
    bb.createBody('box', halfExtent=[0.05,0.20,0.05],  position=[px+0.26, py+0.35, pz+0.16], color='#FDBCB4', mass=0)
    bb.createBody('box', halfExtent=[0.04,0.16,0.04],  position=[px-0.30, py+0.08, pz+0.10], color='#F5DEB3', mass=0)
    bb.createBody('box', halfExtent=[0.04,0.16,0.04],  position=[px+0.30, py+0.08, pz+0.10], color='#F5DEB3', mass=0)
    bb.createBody('box', halfExtent=[0.04,0.07,0.03],  position=[px-0.31, py-0.12, pz+0.07], color='#FDBCB4', mass=0)
    bb.createBody('box', halfExtent=[0.04,0.07,0.03],  position=[px+0.31, py-0.12, pz+0.07], color='#FDBCB4', mass=0)
    bb.createBody('box', halfExtent=[0.18,0.18,0.09],  position=[px, py-0.22, pz+0.10], color='#B3E5FC', mass=0)
    bb.createBody('box', halfExtent=[0.08,0.22,0.07],  position=[px-0.10, py-0.52, pz+0.08], color='#D2B48C', mass=0)
    bb.createBody('box', halfExtent=[0.08,0.22,0.07],  position=[px+0.10, py-0.52, pz+0.08], color='#D2B48C', mass=0)
    bb.createBody('box', halfExtent=[0.06,0.20,0.06],  position=[px-0.10, py-0.82, pz+0.07], color='#D2B48C', mass=0)
    bb.createBody('box', halfExtent=[0.06,0.20,0.06],  position=[px+0.10, py-0.82, pz+0.07], color='#D2B48C', mass=0)
    bb.createBody('box', halfExtent=[0.06,0.09,0.04],  position=[px-0.10, py-1.02, pz+0.05], color='#FDBCB4', mass=0)
    bb.createBody('box', halfExtent=[0.06,0.09,0.04],  position=[px+0.10, py-1.02, pz+0.05], color='#FDBCB4', mass=0)
    bb.createBody('box', halfExtent=[0.004,0.30,0.004], position=[px-0.18, py+0.10, pz+0.18], color='#FF5722', mass=0)
    bb.createBody('box', halfExtent=[0.004,0.30,0.004], position=[px,      py+0.10, pz+0.20], color='#4CAF50', mass=0)
    bb.createBody('box', halfExtent=[0.22,0.28,0.005], position=[px, py-0.02, pz+0.25], color='#00796B', mass=0)

def setup_furniture():
    bb.createBody('box', halfExtent=[0.016,0.016,0.82], position=[-0.85,BED_Y+0.85,0.82],  color='#BDBDBD', mass=0)
    bb.createBody('box', halfExtent=[0.08,0.028,0.12],  position=[-0.85,BED_Y+0.85,1.70],  color='#80D8FF', mass=0)
    bb.createBody('sphere', radius=0.025,                position=[-0.85,BED_Y+0.85,1.85],  color='#4FC3F7', mass=0)
    bb.createBody('box', halfExtent=[0.20,0.20,0.01],   position=[-0.85,BED_Y+0.85,0.02],  color='#9E9E9E', mass=0)
    bb.createBody('box', halfExtent=[0.003,0.003,0.50], position=[-0.85,BED_Y+0.85,1.30],  color='#E3F2FD', mass=0)
    bb.createBody('box', halfExtent=[0.03,0.03,0.50],   position=[-1.10,BED_Y+0.60,0.50],  color='#424242', mass=0)
    bb.createBody('box', halfExtent=[0.18,0.06,0.14],   position=[-1.10,BED_Y+0.63,1.08],  color='#212121', mass=0)
    bb.createBody('box', halfExtent=[0.15,0.02,0.11],   position=[-1.10,BED_Y+0.60,1.08],  color='#1565C0', mass=0)
    bb.createBody('box', halfExtent=[0.12,0.01,0.005],  position=[-1.10,BED_Y+0.58,1.10],  color='#00E676', mass=0)
    bb.createBody('box', halfExtent=[0.005,0.01,0.04],  position=[-1.00,BED_Y+0.58,1.11],  color='#00E676', mass=0)
    bb.createBody('box', halfExtent=[0.005,0.01,0.04],  position=[-1.18,BED_Y+0.58,1.11],  color='#00E676', mass=0)
    TX, TY = 0.95, BED_Y - 0.20
    for lx,ly in [(-0.28,-0.18),(0.28,-0.18),(-0.28,0.18),(0.28,0.18)]:
        bb.createBody('box', halfExtent=[0.015,0.015,0.42], position=[TX+lx,TY+ly,0.42], color='#80CBC4', mass=0)
    bb.createBody('box', halfExtent=[0.30,0.20,0.015], position=[TX,TY,0.87], color='#80CBC4', mass=0)
    bb.createBody('box', halfExtent=[0.008,0.06,0.008],  position=[TX-0.12,TY-0.05,0.90], color='#ECEFF1', mass=0)
    bb.createBody('box', halfExtent=[0.025,0.018,0.055], position=[TX+0.02,TY-0.04,0.93], color='#E3F2FD', mass=0)
    bb.createBody('box', halfExtent=[0.020,0.015,0.045], position=[TX+0.14,TY+0.05,0.92], color='#ECEFF1', mass=0)
    bb.createBody('box', halfExtent=[0.006,0.06,0.004],  position=[TX-0.08,TY+0.08,0.90], color='#B0BEC5', mass=0)
    bb.createBody('box', halfExtent=[0.42,0.16,0.52],  position=[-4.6,3.2,0.52], color='#FAFAFA', mass=0)
    bb.createBody('box', halfExtent=[0.42,0.16,0.02],  position=[-4.6,3.2,1.05], color='#E0E0E0', mass=0)
    for i,c in enumerate(['#E3F2FD','#FCE4EC','#F3E5F5','#E8F5E9']):
        bb.createBody('box', halfExtent=[0.07,0.08,0.06], position=[-4.6+(-0.18+i*0.12),3.10,0.72], color=c, mass=0)
    DX, DY2 = 4.3, 1.2
    bb.createBody('box', halfExtent=[0.70,0.35,0.03],  position=[DX,DY2,0.76],      color='#C8A97A', mass=0)
    bb.createBody('box', halfExtent=[0.70,0.03,0.32],  position=[DX,DY2-0.35,0.44], color='#5D4037', mass=0)
    bb.createBody('box', halfExtent=[0.03,0.35,0.32],  position=[DX-0.70,DY2,0.44], color='#5D4037', mass=0)
    bb.createBody('box', halfExtent=[0.03,0.35,0.32],  position=[DX+0.70,DY2,0.44], color='#5D4037', mass=0)
    for mx,sc in [(-0.32,'#1565C0'),(0.32,'#004D40')]:
        bb.createBody('box', halfExtent=[0.26,0.03,0.17],  position=[DX+mx,DY2+0.10,1.22],  color='#111111', mass=0)
        bb.createBody('box', halfExtent=[0.23,0.012,0.14], position=[DX+mx,DY2+0.09,1.22],  color=sc,        mass=0)
        bb.createBody('box', halfExtent=[0.022,0.022,0.18],position=[DX+mx,DY2+0.10,1.00],  color='#444444', mass=0)
    bb.createBody('box', halfExtent=[0.22,0.09,0.01],  position=[DX,DY2-0.08,0.81],   color='#37474F', mass=0)
    SY2=DY2+0.65; SZ=0.57
    bb.createBody('sphere', radius=0.12,               position=[DX,SY2-0.09,SZ+0.76], color='#FDBCB4', mass=0)
    bb.createBody('box', halfExtent=[0.18,0.13,0.26],  position=[DX,SY2-0.06,SZ+0.30], color='#E8F5E9', mass=0)
    bb.createBody('box', halfExtent=[0.16,0.12,0.06],  position=[DX,SY2,SZ+0.04],      color='#1A237E', mass=0)
    bb.createBody('box', halfExtent=[0.12,0.09,0.04],  position=[DX,SY2-0.06,SZ+0.87], color='#3E2723', mass=0)

def build_wall_stickers():
    WY = W - 0.07; LX = -W + 0.07; RX = W - 0.07
    ax,ay,az = -2.8,WY,1.4
    bb.createBody('box', halfExtent=[0.55,0.04,0.70], position=[ax,ay,az],        color='#1565C0', mass=0)
    bb.createBody('box', halfExtent=[0.50,0.03,0.65], position=[ax,ay+0.01,az],   color='#E3F2FD', mass=0)
    bb.createBody('box', halfExtent=[0.50,0.02,0.07], position=[ax,ay+0.02,az+0.56], color='#0D47A1', mass=0)
    bb.createBody('sphere', radius=0.07,              position=[ax,ay+0.03,az+0.34], color='#90CAF9', mass=0)
    bb.createBody('box', halfExtent=[0.09,0.02,0.13], position=[ax,ay+0.03,az+0.14], color='#42A5F5', mass=0)
    for ax2,c in [(-0.12,'#64B5F6'),(0.12,'#64B5F6')]:
        bb.createBody('box', halfExtent=[0.025,0.02,0.09], position=[ax+ax2,ay+0.03,az+0.15], color=c, mass=0)
        bb.createBody('box', halfExtent=[0.025,0.015,0.06],position=[ax+ax2*1.3,ay+0.02,az+0.04], color=c, mass=0)
    bx,by,bz = -0.8,WY,1.55
    bb.createBody('box', halfExtent=[0.28,0.03,0.28], position=[bx,by,bz],       color='#00695C', mass=0)
    bb.createBody('box', halfExtent=[0.24,0.02,0.24], position=[bx,by+0.01,bz],  color='#E0F2F1', mass=0)
    cx,cy,cz = 0.8,WY,1.60
    bb.createBody('box', halfExtent=[0.20,0.03,0.20], position=[cx,cy,cz],        color='#B71C1C', mass=0)
    bb.createBody('box', halfExtent=[0.16,0.02,0.16], position=[cx,cy+0.01,cz],   color='#FFCDD2', mass=0)
    dx2,dy2,dz2 = 2.5,WY,1.3
    bb.createBody('box', halfExtent=[0.42,0.03,0.52], position=[dx2,dy2,dz2],      color='#37474F', mass=0)
    bb.createBody('box', halfExtent=[0.38,0.02,0.48], position=[dx2,dy2+0.01,dz2], color='#ECEFF1', mass=0)
    ex,ey,ez = LX,-1.0,1.5
    bb.createBody('box', halfExtent=[0.03,0.36,0.46], position=[ex,ey,ez],       color='#E65100', mass=0)
    bb.createBody('box', halfExtent=[0.02,0.31,0.41], position=[ex+0.01,ey,ez],  color='#FFF3E0', mass=0)
    fx,fy,fz = LX,1.5,1.80
    bb.createBody('box', halfExtent=[0.02,0.20,0.20], position=[fx,fy,fz],       color='#1B5E20', mass=0)
    bb.createBody('box', halfExtent=[0.015,0.16,0.16],position=[fx+0.005,fy,fz], color='#C8E6C9', mass=0)
    gx,gy,gz = LX,3.2,1.60
    bb.createBody('box', halfExtent=[0.02,0.26,0.26], position=[gx,gy,gz],        color='#212121', mass=0)
    bb.createBody('sphere', radius=0.08,  position=[gx+0.01,gy,gz+0.02], color='#00E5FF', mass=0)
    hx,hy,hz = RX,-1.0,1.4
    bb.createBody('box', halfExtent=[0.03,0.38,0.52], position=[hx,hy,hz],       color='#4A148C', mass=0)
    bb.createBody('box', halfExtent=[0.02,0.33,0.47], position=[hx-0.01,hy,hz],  color='#F3E5F5', mass=0)
    ix2,iy2,iz3 = RX,1.8,1.85
    bb.createBody('box', halfExtent=[0.02,0.22,0.22], position=[ix2,iy2,iz3],       color='#F57F17', mass=0)
    bb.createBody('box', halfExtent=[0.015,0.18,0.18],position=[ix2-0.005,iy2,iz3], color='#FFFDE7', mass=0)
    jx,jy,jz = RX,3.5,1.30
    bb.createBody('box', halfExtent=[0.02,0.33,0.40], position=[jx,jy,jz],       color='#1B5E20', mass=0)
    bb.createBody('box', halfExtent=[0.015,0.28,0.35],position=[jx-0.005,jy,jz], color='#2E7D32', mass=0)
    for row in [-0.23,-0.09,0.05,0.19]:
        bb.createBody('box', halfExtent=[0.005,0.23,0.007], position=[jx-0.01,jy,jz+row], color='#69F0AE', mass=0)

# ═══════════════════════════════════════════════════════════════════════════════
#  DA VINCI ROBOT
# ═══════════════════════════════════════════════════════════════════════════════
def _dv_arm_parts(hub_x, hub_y, hub_z, sx, sy, reach, drop):
    mag = abs(sx) + abs(sy) + 1e-6
    dx  = sx / mag;  dy = sy / mag
    ax  = hub_x + sx;  ay = hub_y + sy;  az = hub_z
    ix  = ax + dx * reach;  iy = ay + dy * reach
    iz  = hub_z - drop - 0.62
    parts = []
    parts.append(('box',    [ax, ay, az],                            dict(halfExtent=[0.06,0.06,0.06],   color='#546E7A')))
    parts.append(('box',    [ax+dx*0.20, ay+dy*0.20, az-0.08],      dict(halfExtent=[0.035,0.035,0.16], color='#ECEFF1')))
    parts.append(('sphere', [ax+dx*0.36, ay+dy*0.36, az-drop*0.6],  dict(radius=0.048,                  color='#546E7A')))
    parts.append(('box',    [ax+dx*0.50, ay+dy*0.50, az-drop*0.8],  dict(halfExtent=[0.028,0.028,0.18], color='#B0BEC5')))
    parts.append(('sphere', [ax+dx*reach*0.88, ay+dy*reach*0.88, az-drop*0.9-0.12], dict(radius=0.036, color='#546E7A')))
    parts.append(('box',    [ix, iy, iz],                             dict(halfExtent=[0.048,0.033,0.08], color='#ECEFF1')))
    parts.append(('box',    [ix, iy, iz+0.04],                        dict(halfExtent=[0.052,0.038,0.018],color='#42A5F5')))
    parts.append(('box',    [ix, iy, iz-0.10],                        dict(halfExtent=[0.006,0.006,0.085],color='#263238')))
    px2 = -dy*0.028;  py2 = dx*0.028
    parts.append(('box',    [ix+px2, iy+py2, iz-0.148],              dict(halfExtent=[0.005,0.005,0.036],color='#78909C')))
    parts.append(('box',    [ix-px2, iy-py2, iz-0.148],              dict(halfExtent=[0.005,0.005,0.036],color='#78909C')))
    return parts

def _dv_ik(tx, ty, tz):
    SFRAC = 0.20
    ddx = tx - DV_X;  ddy = ty - DV_Y
    L1  = abs(ddx) + abs(ddy) + 1e-6
    sx  = ddx * SFRAC
    sy  = ddy * SFRAC
    if abs(ddx) >= abs(ddy):
        reach = abs(ddx * (1.0 - SFRAC) / (ddx / L1))
    else:
        reach = abs(ddy * (1.0 - SFRAC) / (ddy / L1))
    reach = max(0.12, min(0.88, reach))
    drop  = max(0.02, min(0.82, DV_HUB_Z - tz - 0.775))
    return sx, sy, reach, drop

def build_davinci_base():
    ids = []
    def _b(shape, pos, **kw):
        ids.append(bb.createBody(shape, position=pos, **kw, mass=0))
    _b('box',   [DV_X, DV_Y, DV_BASE_Z-0.38],   halfExtent=[0.32,0.32,0.04],  color='#37474F')
    _b('box',   [DV_X, DV_Y, DV_BASE_Z+0.10],   halfExtent=[0.12,0.12,0.52],  color='#546E7A')
    _b('box',   [DV_X, DV_Y, DV_BASE_Z+0.30],   halfExtent=[0.13,0.13,0.03],  color='#42A5F5')
    _b('box',   [DV_X, DV_Y, DV_HUB_Z],         halfExtent=[0.20,0.20,0.09],  color='#ECEFF1')
    _b('box',   [DV_X, DV_Y, DV_HUB_Z+0.06],    halfExtent=[0.18,0.18,0.025], color='#42A5F5')
    _b('box',   [DV_X, DV_Y+0.45, DV_HUB_Z+0.16], halfExtent=[0.06,0.50,0.04], color='#546E7A')
    _b('sphere',[DV_X, DV_Y+0.92, DV_HUB_Z+0.18], radius=0.055,                color='#90A4AE')
    _b('sphere',[DV_X, DV_Y+0.92, DV_HUB_Z+0.18], radius=0.028,                color='#00E5FF')
    for ang in range(8):
        a = ang * math.pi / 4
        _b('sphere', [DV_X+0.17*math.cos(a), DV_Y+0.17*math.sin(a), DV_HUB_Z+0.10],
           radius=0.012, color='#00E676')
    return ids

def build_davinci_arms(arm_params):
    ids = []
    for arm_n in [1, 2, 3, 4]:
        sx, sy, reach, drop = arm_params[arm_n]
        parts = _dv_arm_parts(DV_X, DV_Y, DV_HUB_Z, sx, sy, reach, drop)
        for shape, pos, kw in parts:
            ids.append(bb.createBody(shape, position=pos, **kw, mass=0))
    return ids

def update_davinci_arms(ids, arm_params):
    idx = 0
    for arm_n in [1, 2, 3, 4]:
        sx, sy, reach, drop = arm_params[arm_n]
        parts = _dv_arm_parts(DV_X, DV_Y, DV_HUB_Z, sx, sy, reach, drop)
        for _, pos, _ in parts:
            bb.resetBasePose(ids[idx], pos)
            idx += 1

PARTS_PER_ARM = 10

DV_CYCLE = 12.0
DV_REST = {
    1: ( 0.30,  0.42, 0.13, 0.02),
    2: (-0.30,  0.42, 0.13, 0.02),
    3: ( 0.30, -0.20, 0.13, 0.02),
    4: (-0.30, -0.20, 0.13, 0.02),
}

def _smooth(a, b, p):
    f = (1 - math.cos(p * math.pi)) * 0.5
    return a + (b - a) * f

def _dv_arm_values(t_sec, arm_n):
    offset  = (arm_n - 1) * (DV_CYCLE / 4.0)
    phase_t = (t_sec + offset) % DV_CYCLE
    frac    = phase_t / DV_CYCLE
    tx, ty, tz = ARM_TARGETS[arm_n]
    sx_s, sy_s, r_s, d_s = _dv_ik(tx, ty, tz)
    sx_r, sy_r, r_r, d_r = DV_REST[arm_n]
    if frac < 0.25:
        p = 0.0
    elif frac < 0.52:
        p = _smooth(0.0, 1.0, (frac - 0.25) / 0.27)
    elif frac < 0.68:
        p = 1.0
    elif frac < 0.88:
        p = _smooth(1.0, 0.0, (frac - 0.68) / 0.20)
    else:
        p = 0.0
    return (
        sx_r  + (sx_s - sx_r) * p,
        sy_r  + (sy_s - sy_r) * p,
        r_r   + (r_s  - r_r)  * p,
        d_r   + (d_s  - d_r)  * p,
    )

def _arm_at_patient(t_sec, arm_n):
    offset = (arm_n - 1) * (DV_CYCLE / 4.0)
    frac   = ((t_sec + offset) % DV_CYCLE) / DV_CYCLE
    return 0.52 <= frac <= 0.68

# ═══════════════════════════════════════════════════════════════════════════════
#  LOKOMAT NX
# ═══════════════════════════════════════════════════════════════════════════════
LK_X, LK_Y = -3.5, 1.5

def setup_lokomat():
    X, Y, G = LK_X, LK_Y, 0
    bb.createBody('box', halfExtent=[0.70,1.10,0.09], position=[X,Y,G+0.09],      color='#F5F5F5', mass=0)
    bb.createBody('box', halfExtent=[0.55,0.90,0.02], position=[X,Y,G+0.19],      color='#1C1C1C', mass=0)
    bb.createBody('box', halfExtent=[0.70,0.25,0.04], position=[X,Y-1.32,G+0.05], color='#E0E0E0', mass=0)
    col_h=1.10; col_z=G+0.19+col_h
    for cx,cy in [(-0.62,0.85),(0.62,0.85),(-0.62,-0.75),(0.62,-0.75)]:
        bb.createBody('box', halfExtent=[0.045,0.045,col_h], position=[X+cx,Y+cy,col_z], color='#E0E0E0', mass=0)
        bb.createBody('box', halfExtent=[0.07,0.07,0.04],    position=[X+cx,Y+cy,G+0.22],color='#BDBDBD', mass=0)
    top_z=G+0.19+col_h*2-0.04; mid_z=G+0.19+col_h
    bb.createBody('box', halfExtent=[0.66,0.045,0.045], position=[X,Y+0.85,top_z],     color='#E0E0E0', mass=0)
    bb.createBody('box', halfExtent=[0.66,0.045,0.045], position=[X,Y-0.75,top_z],     color='#E0E0E0', mass=0)
    bb.createBody('box', halfExtent=[0.045,0.80,0.045], position=[X-0.62,Y+0.05,top_z],color='#E0E0E0', mass=0)
    bb.createBody('box', halfExtent=[0.045,0.80,0.045], position=[X+0.62,Y+0.05,top_z],color='#E0E0E0', mass=0)
    bb.createBody('box', halfExtent=[0.68,0.82,0.04],   position=[X,Y+0.05,top_z+0.06],color='#F5F5F5', mass=0)
    mon_y=Y+0.85; mon_z=mid_z+0.25
    bb.createBody('box', halfExtent=[0.38,0.05,0.24],   position=[X,mon_y+0.07,mon_z],  color='#212121', mass=0)
    bb.createBody('box', halfExtent=[0.34,0.01,0.20],   position=[X,mon_y+0.03,mon_z],  color='#1565C0', mass=0)
    hip_z=G+0.19+0.85
    bb.createBody('box', halfExtent=[0.24,0.14,0.16],   position=[X,Y+0.05,hip_z],      color='#37474F', mass=0)
    bb.createBody('box', halfExtent=[0.05,0.05,0.10],   position=[X-0.29,Y+0.05,hip_z], color='#455A64', mass=0)
    bb.createBody('box', halfExtent=[0.05,0.05,0.10],   position=[X+0.29,Y+0.05,hip_z], color='#455A64', mass=0)
    rail_z=G+0.19+0.78
    bb.createBody('box', halfExtent=[0.03,0.35,0.025],  position=[X-0.62,Y+0.20,rail_z],color='#E0E0E0', mass=0)
    bb.createBody('box', halfExtent=[0.03,0.35,0.025],  position=[X+0.62,Y+0.20,rail_z],color='#E0E0E0', mass=0)
    bb.createBody('sphere', radius=0.04, position=[X-0.62,Y+0.45,rail_z+0.04], color='#F44336', mass=0)
    bb.createBody('sphere', radius=0.04, position=[X+0.62,Y+0.45,rail_z+0.04], color='#F44336', mass=0)
    bws_z=top_z-0.10
    bb.createBody('box', halfExtent=[0.18,0.12,0.09], position=[X,Y+0.05,bws_z],           color='#546E7A', mass=0)
    bb.createBody('box', halfExtent=[0.02,0.02,0.45], position=[X-0.10,Y+0.05,bws_z-0.54], color='#BDBDBD', mass=0)
    bb.createBody('box', halfExtent=[0.02,0.02,0.45], position=[X+0.10,Y+0.05,bws_z-0.54], color='#BDBDBD', mass=0)
    bb.createBody('box', halfExtent=[0.17,0.11,0.18], position=[X,Y+0.05,bws_z-0.85],      color='#212121', mass=0)
    bb.createBody('sphere', radius=0.10,              position=[X,Y+0.05,hip_z+0.62],   color='#FDBCB4', mass=0)
    bb.createBody('box', halfExtent=[0.12,0.09,0.22], position=[X,Y+0.02,hip_z+0.30],   color='#E8F5E9', mass=0)

def build_lokomat_legs(t):
    ids = []
    X, Y, G = LK_X, LK_Y, 0
    hip_z = G+0.19+0.85
    SWING_AMP=0.13; ABDUCT_AMP=0.025; KNEE_AMP=0.14; ANKLE_AMP=0.04
    for base_x, phase, cs in [(X-0.18,0.0,+0.08),(X+0.18,math.pi,-0.08)]:
        hs  = SWING_AMP  * math.sin(t+phase)
        ha  = ABDUCT_AMP * math.sin(t+phase+1.0)
        kf  = KNEE_AMP   * max(0.0, math.sin(t+phase+math.pi*0.55))
        adf = ANKLE_AMP  * math.sin(t+phase+math.pi*0.8)
        SX  = base_x+ha;  SY = Y+0.05+hs
        ids.append(bb.createBody('box', halfExtent=[0.03,0.03,0.06],   position=[SX,SY,hip_z-0.22],     color='#90A4AE', mass=0))
        ids.append(bb.createBody('box', halfExtent=[0.12,0.10,0.20],   position=[SX,SY,hip_z-0.46],     color='#CFD8DC', mass=0))
        ids.append(bb.createBody('box', halfExtent=[0.08,0.07,0.16],   position=[SX,SY-0.03,hip_z-0.46],color='#90A4AE', mass=0))
        ids.append(bb.createBody('box', halfExtent=[0.13,0.02,0.025],  position=[SX,SY-0.05,hip_z-0.32],color='#ECEFF1', mass=0))
        ids.append(bb.createBody('box', halfExtent=[0.13,0.02,0.025],  position=[SX,SY-0.05,hip_z-0.60],color='#ECEFF1', mass=0))
        knee_z = hip_z-0.74-kf*0.30
        ids.append(bb.createBody('box',    halfExtent=[0.05,0.05,0.05], position=[SX,SY,knee_z],        color='#546E7A', mass=0))
        ids.append(bb.createBody('sphere', radius=0.03, position=[SX+cs,SY,knee_z],                    color='#78909C', mass=0))
        shank_z = knee_z-0.17-kf*0.38
        ids.append(bb.createBody('box', halfExtent=[0.025,0.025,0.17], position=[SX,SY,shank_z],        color='#CFD8DC', mass=0))
        ankle_z = shank_z-0.14
        ids.append(bb.createBody('box', halfExtent=[0.09,0.02,0.025],  position=[SX,SY-0.05,ankle_z],   color='#9E9E9E', mass=0))
        foot_z  = max(ankle_z-0.12+adf, G+0.21)
        ids.append(bb.createBody('box', halfExtent=[0.09,0.13,0.02],   position=[SX,SY,foot_z],         color='#78909C', mass=0))
    return ids

# ═══════════════════════════════════════════════════════════════════════════════
#  PANDA ARM  (URDF)
# ═══════════════════════════════════════════════════════════════════════════════
PANDA_CYCLE = 14.0
PANDA_KF = [
    [ 0.0,  -0.785,  0.0, -2.356,  0.0,  1.571,  0.785],
    [ 0.75, -0.40,   0.0, -1.60,  -0.30,  1.20,   1.0  ],
    [ 0.90, -0.12,   0.0, -1.15,  -0.30,  1.02,   1.0  ],
    [ 0.35, -0.65,   0.0, -2.10,   0.0,   1.45,   0.785],
]
PANDA_DUR = [3.0, 2.5, 4.0, 2.5, 2.0]

def _panda_angles(t_sec):
    phase = t_sec % PANDA_CYCLE
    kf_seq = [(0,1),(1,2),(2,3),(3,0)]
    cumul = 0.0
    for ki,(kf_a,kf_b) in enumerate(kf_seq):
        dur = PANDA_DUR[ki]
        if phase < cumul + dur:
            alpha = _smooth(0.0, 1.0, (phase - cumul) / dur)
            src = PANDA_KF[kf_a];  dst = PANDA_KF[kf_b]
            return [src[j] + (dst[j]-src[j]) * alpha for j in range(7)]
        cumul += dur
    return PANDA_KF[0]

# ═══════════════════════════════════════════════════════════════════════════════
#  DA VINCI HUD
# ═══════════════════════════════════════════════════════════════════════════════
_dv_hud_id   = None
_dv_hud_text = ''
_scan_labels = {}
ARM_NAMES  = {1:'CHEST', 2:'ABDOMEN', 3:'L-ARM VITALS', 4:'AIRWAY'}
ARM_COLORS = {1:'#00E676', 2:'#69F0AE', 3:'#40C4FF',    4:'#FF80AB'}

def _update_dv_hud(t_sec):
    global _dv_hud_id, _dv_hud_text
    active  = [ARM_NAMES[n] for n in [1,2,3,4] if _arm_at_patient(t_sec, n)]
    new_txt = ('  DA VINCI: CHECKING — ' + ' | '.join(active)) if active else '  MONITORING PATIENT...'
    col     = '#00E676' if active else '#00BCD4'
    if new_txt != _dv_hud_text:
        if _dv_hud_id is not None:
            bb.removeDebugObject(_dv_hud_id)
        _dv_hud_id = bb.createDebugText(new_txt, (BED_X, BED_Y, 2.35),
                                        bb.getQuaternionFromEuler((0,0,0)),
                                        color=col, size=0.16)
        _dv_hud_text = new_txt

def _update_scan_labels(t_sec):
    global _scan_labels
    for arm_n in [1,2,3,4]:
        tx,ty,tz = ARM_TARGETS[arm_n]
        at = _arm_at_patient(t_sec, arm_n)
        if at and arm_n not in _scan_labels:
            lid = bb.createDebugText(
                f'[ {ARM_NAMES[arm_n]} OK ✓ ]',
                (tx, ty, tz + 0.28),
                bb.getQuaternionFromEuler((0,0,0)),
                color=ARM_COLORS[arm_n], size=0.13)
            _scan_labels[arm_n] = (lid, t_sec + 3.0)
        if arm_n in _scan_labels:
            lid, exp = _scan_labels[arm_n]
            if t_sec > exp:
                bb.removeDebugObject(lid)
                del _scan_labels[arm_n]

# ═══════════════════════════════════════════════════════════════════════════════
#  ASSISTANT ROBOT — full body rebuilt each frame at patrol position
# ═══════════════════════════════════════════════════════════════════════════════
def _rot(lx, ly, heading):
    """Rotate local (lx,ly) offset by heading angle."""
    c = math.cos(heading); s = math.sin(heading)
    return c*lx - s*ly,  s*lx + c*ly

def build_assistant_robot_frame(t_sec):
    """
    Rebuild the entire assistant robot at its current patrol position.
    Returns list of body IDs (all removed next frame).
    The robot faces the direction of travel (tangent to circle).
    """
    ids = []
    X, Y, heading = _patrol_pos(t_sec)
    Z = PATROL_Z

    # Walking animation
    walk_t   = t_sec * 4.0          # walk cycle speed
    bob      = math.sin(walk_t * 2) * 0.010
    leg_l_sw = math.sin(walk_t) * 0.12
    leg_r_sw = math.sin(walk_t + math.pi) * 0.12
    arm_l_sw = math.sin(walk_t + math.pi) * 0.015   # arm swings opposite leg
    arm_r_sw = 0.0                                   # right arm holds tray, stays steady

    # Blink
    blink_col = '#FFFFFF' if (t_sec % 3.0) / 3.0 > 0.96 else '#E0F7FA'
    # Antenna pulse
    ant_col = '#FF1744' if math.sin(t_sec * 4.0) > 0 else '#FF8A80'

    def _p(lx, ly, lz):
        """Local → world: rotate (lx,ly) by heading, add (X,Y,Z+lz+bob)."""
        wx, wy = _rot(lx, ly, heading)
        return [X + wx, Y + wy, Z + lz + bob]

    def _add(shape, lx, ly, lz, **kw):
        ids.append(bb.createBody(shape, position=_p(lx, ly, lz), **kw, mass=0))

    # ── FEET ─────────────────────────────────────────────────────────────────
    # Feet step in the forward (local-y) direction
    _add('box',  -0.12, leg_l_sw, 0.04, halfExtent=[0.07,0.11,0.04], color='#BDBDBD')
    _add('box',   0.12, leg_r_sw, 0.04, halfExtent=[0.07,0.11,0.04], color='#BDBDBD')

    # ── LOWER LEGS ───────────────────────────────────────────────────────────
    _add('box',  -0.12, leg_l_sw, 0.20, halfExtent=[0.055,0.055,0.12], color='#E0E0E0')
    _add('box',   0.12, leg_r_sw, 0.20, halfExtent=[0.055,0.055,0.12], color='#E0E0E0')
    # Knee caps
    _add('box',  -0.12, leg_l_sw, 0.34, halfExtent=[0.065,0.065,0.030], color='#9E9E9E')
    _add('box',   0.12, leg_r_sw, 0.34, halfExtent=[0.065,0.065,0.030], color='#9E9E9E')

    # ── UPPER LEGS ───────────────────────────────────────────────────────────
    _add('box',  -0.12, leg_l_sw*0.5, 0.52, halfExtent=[0.065,0.065,0.15], color='#E0E0E0')
    _add('box',   0.12, leg_r_sw*0.5, 0.52, halfExtent=[0.065,0.065,0.15], color='#E0E0E0')

    # ── PELVIS ───────────────────────────────────────────────────────────────
    _add('box',   0.0,  0.0, 0.70, halfExtent=[0.18,0.11,0.07], color='#BDBDBD')

    # ── TORSO ────────────────────────────────────────────────────────────────
    _add('box',  0.0, 0.0, 0.84, halfExtent=[0.17,0.10,0.07], color='#E8E8E8')
    _add('box',  0.0, 0.0, 0.97, halfExtent=[0.18,0.10,0.06], color='#EEEEEE')
    _add('box',  0.0, 0.0, 1.08, halfExtent=[0.17,0.10,0.05], color='#E8E8E8')

    # Chest screen — offset forward (local -y = facing direction)
    _add('box',  0.0, -0.095, 0.95, halfExtent=[0.10,0.008,0.08], color='#1565C0')
    _add('box',  0.0, -0.104, 1.00, halfExtent=[0.07,0.005,0.006], color='#00E5FF')
    _add('box',  0.0, -0.104, 0.95, halfExtent=[0.07,0.005,0.006], color='#00E5FF')
    _add('box',  0.0, -0.104, 0.90, halfExtent=[0.05,0.005,0.006], color='#40C4FF')
    _add('sphere', 0.0, -0.105, 1.07, radius=0.018, color='#F44336')

    # ── SHOULDERS ────────────────────────────────────────────────────────────
    _add('box', -0.24, 0.0, 1.08, halfExtent=[0.06,0.06,0.06], color='#9E9E9E')
    _add('box',  0.24, 0.0, 1.08, halfExtent=[0.06,0.06,0.06], color='#9E9E9E')

    # ── UPPER ARMS ───────────────────────────────────────────────────────────
    _add('box', -0.28, arm_l_sw, 0.94, halfExtent=[0.048,0.048,0.10], color='#E0E0E0')
    _add('box',  0.28, 0.0,      0.94, halfExtent=[0.048,0.048,0.10], color='#E0E0E0')

    # Elbows
    _add('sphere', -0.28, arm_l_sw, 0.82, radius=0.04, color='#9E9E9E')
    _add('sphere',  0.28, 0.0,      0.82, radius=0.04, color='#9E9E9E')

    # ── FOREARMS ─────────────────────────────────────────────────────────────
    _add('box', -0.28, arm_l_sw, 0.68, halfExtent=[0.040,0.040,0.10], color='#BDBDBD')
    # Right forearm extends forward to hold tray
    _add('box',  0.22, -0.20,    0.78, halfExtent=[0.040,0.040,0.10], color='#BDBDBD')

    # ── HANDS ────────────────────────────────────────────────────────────────
    _add('box', -0.28, arm_l_sw, 0.555, halfExtent=[0.040,0.035,0.03], color='#9E9E9E')
    _add('box',  0.16, -0.38,    0.78,  halfExtent=[0.040,0.035,0.03], color='#9E9E9E')

    # ── TRAY ─────────────────────────────────────────────────────────────────
    _add('box',  0.06, -0.38, 0.82,  halfExtent=[0.20,0.14,0.012], color='#80CBC4')
    _add('box',  0.06, -0.53, 0.832, halfExtent=[0.20,0.006,0.012], color='#546E7A')
    _add('box',  0.06, -0.23, 0.832, halfExtent=[0.20,0.006,0.012], color='#546E7A')
    _add('box', -0.15, -0.38, 0.832, halfExtent=[0.006,0.14,0.012], color='#546E7A')
    _add('box',  0.27, -0.38, 0.832, halfExtent=[0.006,0.14,0.012], color='#546E7A')
    # Tray items
    _add('box',   -0.06, -0.38, 0.86,  halfExtent=[0.04,0.05,0.04],  color='#E3F2FD')
    _add('sphere', 0.13, -0.34, 0.845, radius=0.025,                  color='#FF5722')
    _add('box',    0.14, -0.44, 0.855, halfExtent=[0.030,0.018,0.022],color='#FCE4EC')

    # ── NECK ─────────────────────────────────────────────────────────────────
    _add('box', 0.0, 0.0, 1.175, halfExtent=[0.055,0.050,0.045], color='#9E9E9E')

    # ── HEAD ─────────────────────────────────────────────────────────────────
    _add('box', 0.0, 0.0, 1.32,  halfExtent=[0.155,0.140,0.13], color='#EEEEEE')
    _add('box', 0.0, 0.0, 1.435, halfExtent=[0.13,0.12,0.025],  color='#E0E0E0')

    # Eye sockets
    _add('box', -0.055, -0.135, 1.34, halfExtent=[0.040,0.010,0.035], color='#424242')
    _add('box',  0.055, -0.135, 1.34, halfExtent=[0.040,0.010,0.035], color='#424242')
    # Eyes
    _add('sphere', -0.055, -0.142, 1.34, radius=0.025, color='#00BCD4')
    _add('sphere',  0.055, -0.142, 1.34, radius=0.025, color='#00BCD4')
    # Pupils
    _add('sphere', -0.055, -0.148, 1.34, radius=0.012, color='#006064')
    _add('sphere',  0.055, -0.148, 1.34, radius=0.012, color='#006064')
    # Glints (blink)
    _add('sphere', -0.044, -0.152, 1.348, radius=0.005, color=blink_col)
    _add('sphere',  0.066, -0.152, 1.348, radius=0.005, color=blink_col)
    # Mouth grille
    for mx in [-0.07, -0.035, 0.0, 0.035, 0.07]:
        _add('box', mx, -0.138, 1.26, halfExtent=[0.012,0.008,0.006], color='#78909C')
    # Nose
    _add('sphere', 0.0, -0.142, 1.295, radius=0.010, color='#546E7A')
    # Ears
    _add('box', -0.167, 0.0, 1.32, halfExtent=[0.010,0.035,0.055], color='#9E9E9E')
    _add('box',  0.167, 0.0, 1.32, halfExtent=[0.010,0.035,0.055], color='#9E9E9E')
    _add('sphere', -0.178, 0.0, 1.32, radius=0.012, color='#00BCD4')
    _add('sphere',  0.178, 0.0, 1.32, radius=0.012, color='#00BCD4')

    # ── ANTENNA ──────────────────────────────────────────────────────────────
    _add('box',    0.0, 0.0, 1.52,  halfExtent=[0.008,0.008,0.06], color='#9E9E9E')
    _add('sphere', 0.0, 0.0, 1.595, radius=0.018, color=ant_col)

    return ids

# ─── Assistant HUD ───────────────────────────────────────────────────────────
_asst_hud_id = None

def _update_asst_hud(t_sec):
    global _asst_hud_id
    rx, ry, _ = _patrol_pos(t_sec)
    msgs = ['  PATROLLING ●', '  DELIVERING ITEMS...', '  ALL CLEAR ✓', '  STANDING BY ◆']
    msg  = msgs[int(t_sec / 5.0) % len(msgs)]
    if _asst_hud_id is not None:
        bb.removeDebugObject(_asst_hud_id)
    _asst_hud_id = bb.createDebugText(
        msg, (rx, ry, PATROL_Z + 1.80),
        bb.getQuaternionFromEuler((0, 0, 0)),
        color='#FFD740', size=0.13)

# ═══════════════════════════════════════════════════════════════════════════════
#  BUILD SCENE
# ═══════════════════════════════════════════════════════════════════════════════
setup_room()
setup_bed_and_patient()
setup_furniture()
build_wall_stickers()
setup_lokomat()

# Panda arm on plinth beside bed
bb.createBody('box', halfExtent=[0.25,0.25,0.04], position=[-0.85,BED_Y-0.30,0.04], color='#424242', mass=0)
panda = bb.loadURDF('panda.urdf', [-0.85, BED_Y-0.30, 0.0], fixedBase=True)
panda_joints = []
for i in range(bb.getNumJoints(panda)):
    jname, jtype, jlimits = bb.getJointInfo(panda, i)
    if jtype != 'fixed':
        panda_joints.append((i, jname, jlimits))

# Da Vinci robot — static base + dynamic arms
dv_base_ids = build_davinci_base()
init_params = {n: DV_REST[n] for n in [1,2,3,4]}
dv_arm_ids  = build_davinci_arms(init_params)

# Opening HUD
_dv_hud_id = bb.createDebugText(
    '  SYSTEM INITIALISING...', (BED_X, BED_Y, 2.35),
    bb.getQuaternionFromEuler((0,0,0)), color='#FFD740', size=0.16)
_dv_hud_text = '  SYSTEM INITIALISING...'

# ═══════════════════════════════════════════════════════════════════════════════
#  MAIN LOOP
# ═══════════════════════════════════════════════════════════════════════════════
t_sec      = 0.0
T_STEP     = 0.05
loko_ids   = []
asst_ids   = []   # assistant robot parts rebuilt every frame

while True:

    # ── 1. LOKOMAT legs ───────────────────────────────────────────────────────
    for bid in loko_ids:
        bb.removeBody(bid)
    loko_ids = build_lokomat_legs(t_sec * 1.4)

    # ── 2. PANDA ARM ──────────────────────────────────────────────────────────
    angles = _panda_angles(t_sec)
    for idx, (ji, jn, jlim) in enumerate(panda_joints):
        if idx < len(angles):
            target = max(jlim[0], min(jlim[1], angles[idx]))
            bb.setJointMotorControl(panda, ji, targetPosition=target)

    # ── 3. DA VINCI ARMS ──────────────────────────────────────────────────────
    arm_params = {n: _dv_arm_values(t_sec, n) for n in [1,2,3,4]}
    update_davinci_arms(dv_arm_ids, arm_params)

    # ── 4. ASSISTANT ROBOT PATROL ─────────────────────────────────────────────
    for bid in asst_ids:
        bb.removeBody(bid)
    asst_ids = build_assistant_robot_frame(t_sec)

    # ── 5. HUD LABELS ─────────────────────────────────────────────────────────
    _update_dv_hud(t_sec)
    _update_scan_labels(t_sec)
    _update_asst_hud(t_sec)

    t_sec += T_STEP
    time.sleep(T_STEP)
