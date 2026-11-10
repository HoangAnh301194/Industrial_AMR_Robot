from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch, FancyArrowPatch, Circle
from matplotlib import rcParams
rcParams['font.family'] = 'DejaVu Sans'
OUT = Path(__file__).resolve().parents[1]/'docs'/'assets'
OUT.mkdir(parents=True,exist_ok=True)

# geometry
fig,ax=plt.subplots(figsize=(10,6.3));ax.set_aspect('equal')
ax.add_patch(Rectangle((-0.35,-0.275),0.70,0.55,facecolor='#f8eaea',edgecolor='#9c222a',linewidth=2.5))
ax.add_patch(Rectangle((-0.16,0.275),0.24,0.055,facecolor='#396f9d',edgecolor='black'))
ax.add_patch(Rectangle((-0.16,-0.33),0.24,0.055,facecolor='#396f9d',edgecolor='black'))
for x in [-0.28,0.26]:
    ax.add_patch(Circle((x,0),0.035,facecolor='#777',edgecolor='black'))
ax.plot([0,0],[0,0],'.',color='black',markersize=8)
ax.arrow(0,0,0.56,0,head_width=0.045,head_length=0.06,width=0.008,color='#9c222a',length_includes_head=True)
ax.text(0.60,0,'v, +x',fontsize=12,color='#9c222a',va='center')
ax.annotate('',xy=(-0.04,0.28),xytext=(-0.04,-0.28),arrowprops={'arrowstyle':'<->','color':'#3374aa','lw':1.8})
ax.text(-0.11,0,'b = 0.45 m (giả định)',fontsize=10,rotation=90,va='center',ha='center',color='#3374aa')
ax.text(-0.04,0.355,'Bánh trái: $\\dot{\\varphi}_L$',ha='center',fontsize=12)
ax.text(-0.04,-0.40,'Bánh phải: $\\dot{\\varphi}_R$',ha='center',fontsize=12)
ax.text(0.00,0.15,'Tải 20 kg',ha='center',fontsize=13)
ax.annotate('$\\omega=\\dot{\\theta}$ (+CCW)',xy=(0.25,0.20),xytext=(0.62,0.37),fontsize=12,color='#2b8a63',arrowprops={'arrowstyle':'->','color':'#2b8a63'})
ax.text(0.03,-0.12,'$r=0.08$ m',ha='center',fontsize=11)
ax.set_xlim(-0.7,1.2);ax.set_ylim(-0.6,0.65);ax.axis('off')
ax.set_title('Robot vi sai: khung toa do, banh, van toc va kich thuoc so bo',fontsize=14)
fig.tight_layout();fig.savefig(OUT/'diff_drive_geometry.png',dpi=190,bbox_inches='tight');plt.close(fig)

# flow
fig, ax=plt.subplots(figsize=(12,7.3));ax.set_xlim(0,12);ax.set_ylim(0,7);ax.axis('off')
def block(x,y,w,h,title,desc,color):
    patch=FancyBboxPatch((x,y),w,h,boxstyle='round,pad=0.08,rounding_size=0.09',ec=color,fc='white',lw=2)
    ax.add_patch(patch)
    ax.text(x+w/2,y+h*0.68,title,ha='center',va='center',fontsize=12,fontweight='bold',color=color)
    ax.text(x+w/2,y+h*0.31,desc,ha='center',va='center',fontsize=10)
    return x,y,w,h
def arrow(a,b,color='#444',style='-|>'):
    ax.add_patch(FancyArrowPatch(a,b,arrowstyle=style,mutation_scale=15,lw=1.7,color=color))

block(0.3,4.65,2.15,1.0,'Nav2 / mission','Lệnh (v*, omega*)','#9d252c')
block(3.0,4.65,2.15,1.0,'Động học nghịch','omegaR*, omegaL*','#225b8a')
block(5.7,4.65,2.2,1.0,'PI/PID 2 bánh','Đặt mô-men/điện áp','#225b8a')
block(8.45,4.65,2.8,1.0,'Motor + Robot','Động lực học, bánh xe','#9d252c')
arrow((2.53,5.14),(2.93,5.14));arrow((5.23,5.14),(5.63,5.14));arrow((7.97,5.14),(8.37,5.14))
block(8.45,2.35,2.8,0.9,'Encoder','omegaR, omegaL đo','#297b53')
arrow((9.85,4.56),(9.85,3.31));arrow((8.4,2.80),(6.8,2.80));arrow((6.8,2.80),(6.8,4.57))
block(3.0,2.35,2.15,0.9,'Odometry / TF','odom -> base_link','#297b53')
arrow((8.4,2.80),(5.22,2.80))
block(0.3,2.35,2.15,0.9,'SLAM / AMCL','map -> odom','#297b53')
arrow((2.91,2.80),(2.53,2.80))
block(8.45,0.45,2.8,0.8,'Giám sát an toàn','E-stop / bảo vệ (riêng)','#bc742e')
ax.text(0.5,1.00,'Lớp giám sát an toàn cần được thiết kế và kiểm chứng độc lập với thuật toán điều hướng / PID.',
        ha='left',va='center',fontsize=10.3,color='#a04519')
ax.set_title('Cau truc dieu khien de xuat: vong ngoai va vong PI hai banh',fontsize=15)
fig.tight_layout();fig.savefig(OUT/'control_architecture.png',dpi=175,bbox_inches='tight');plt.close(fig)
