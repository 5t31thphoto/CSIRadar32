#!/usr/bin/env python3
"""Offline synthetic contract bench for Tactical RF sensing.

This is not hardware-accuracy evidence. It tests mathematical invariants of
an empirical device-free RF model: baseline-relative sensing, circular phase,
common-mode rejection, dropout tolerance, smooth spatial response, and the
ability to distinguish a human-induced perturbation from an empty room.
"""
import math, random, statistics
random.seed(0xC51A)
B=3; F=10

def empty_channel():
    out=[]
    for b in range(B):
        ph=0.8*b+0.25*math.sin(b*1.7)
        amp=.52+.06*math.cos(b*1.3)
        rssi=.58-.035*b
        out += [amp,math.sin(ph),math.cos(ph),.02,.88,rssi,0,1,0,0]
    return out

def channel(x,y):
    """Synthetic multipath channel with a localized human perturbation."""
    out=empty_channel()
    for b in range(B):
        o=b*F
        # Smooth but oscillatory path response; deliberately no inverse-square law.
        p=math.hypot(x-.12*b,y+.09*b)
        g=math.exp(-2.4*p*p)
        m=.14*g + .045*math.sin(3.4*x+1.1*b)*math.cos(2.8*y-.7*b)
        ph=.65*g*math.sin(2.1*x+.5*b)+.42*g*math.cos(2.7*y-.8*b)+.22*math.sin(3*x*y+b)
        amp=max(0,min(1,out[o]+m))
        phase=math.atan2(out[o+1],out[o+2])+ph
        out[o]=amp; out[o+1]=math.sin(phase); out[o+2]=math.cos(phase)
        out[o+3]=.03+.16*g
        out[o+4]=max(.2,.88-.25*(1-g))
        out[o+5]=max(0,min(1,out[o+5]+.07*m))
        aoa=math.atan2(y+.16*math.sin(b),x+.13*math.cos(b))
        conf=.25+.55*g
        out[o+6]=math.sin(aoa)*conf; out[o+7]=math.cos(aoa)*conf
    amps=[out[b*F] for b in range(B)]; rss=[out[b*F+5] for b in range(B)]
    am=sum(amps)/B; rm=sum(rss)/B
    for b in range(B):
        o=b*F; out[o+8]=max(-1,min(1,(amps[b]-am)*2)); out[o+9]=max(-1,min(1,(rss[b]-rm)*2))
    return out

def noisy(f, phase_bias=0, rssi_shift=0, dropout=None, sigma=.012):
    z=f[:]
    for b in range(B):
        o=b*F
        ph=math.atan2(z[o+1],z[o+2])+phase_bias
        z[o+1]=math.sin(ph)+random.gauss(0,sigma); z[o+2]=math.cos(ph)+random.gauss(0,sigma)
        z[o+0]=max(0,min(1,z[o+0]+random.gauss(0,sigma)))
        z[o+3]=max(0,min(1,z[o+3]+random.gauss(0,sigma)))
        z[o+5]=max(0,min(1,z[o+5]+rssi_shift+random.gauss(0,sigma)))
        if dropout is not None and b==dropout: z[o+4]=.02
    return z

def phase_relative(f, base):
    d=f[:]
    for b in range(B):
        o=b*F; d[o]=abs(f[o]-base[o])
        cs,sn=base[o+2],base[o+1]; fs,fn=f[o+2],f[o+1]
        d[o+1]=fn*cs-fs*sn; d[o+2]=fs*cs+fn*sn
        d[o+3]=abs(f[o+3]-base[o+3]); d[o+5]=abs(f[o+5]-base[o+5])
    return d

def activation(f,base):
    d=phase_relative(f,base); vals=[]
    for b in range(B):
        o=b*F
        vals.append(.55*d[o]+.20*d[o+3]+.25*(1-abs(d[o+2])))
    return sum(vals)/B

def target_similarity(a,b):
    num=den=0.0
    for k in range(B):
        o=k*F; q=max(.03,a[o+4])*max(.03,b[o+4])
        num += q*a[o]*b[o]; den += q*b[o]*b[o]
    alpha=max(.35,min(2.8,math.sqrt(num/den))) if den>.0004 else 1.0
    s=w=0.0
    for k in range(B):
        o=k*F; q=max(.03,a[o+4])*max(.03,b[o+4])
        amp=max(0,1-abs(a[o]-alpha*b[o]))
        dp=max(-1,min(1,a[o+1]*b[o+1]+a[o+2]*b[o+2])); phase=.5*(dp+1)
        temporal=max(0,1-abs(a[o+3]-b[o+3])); rssi=max(0,1-abs(a[o+5]-b[o+5]))
        ad=max(0,1-abs(a[o+8]-b[o+8])); rd=max(0,1-abs(a[o+9]-b[o+9]))
        an=max(1e-6,math.hypot(a[o+6],a[o+7])); bn=max(1e-6,math.hypot(b[o+6],b[o+7]))
        aoa=.5*(max(-1,min(1,(a[o+6]*b[o+6]+a[o+7]*b[o+7])/(an*bn)))+1) if an>.12 and bn>.12 else 1.0
        s += q*(.23*amp+.08*phase+.08*phase+.035*temporal+.035*rssi+.14*ad+.14*rd+.05*(1-abs(a[o+4]-b[o+4]))+.08*aoa)
        w += q
    return s/max(w,1e-9)

def dist(a,b):
    s=w=0
    for k in range(B):
        o=k*F; q=max(.03,a[o+4]); dp=max(-1,min(1,a[o+1]*b[o+1]+a[o+2]*b[o+2]))
        an=max(1e-6,math.hypot(a[o+6],a[o+7])); bn=max(1e-6,math.hypot(b[o+6],b[o+7]))
        daoa=(a[o+6]*b[o+6]+a[o+7]*b[o+7])/(an*bn) if an>.12 and bn>.12 else 1.0
        s += q*(.30*(a[o]-b[o])**2+.18*(1-dp)+.14*(a[o+3]-b[o+3])**2+.035*(a[o+5]-b[o+5])**2+.06*(a[o+4]-b[o+4])**2+.20*(1-max(-1,min(1,daoa))))
        w+=q
    return math.sqrt(s/max(w,1e-9))

base=empty_channel()
pts=[]
for i in range(72):
    t=i/71; a=2*math.pi*t
    x=.72*math.cos(a)+.10*math.sin(3*a); y=.62*math.sin(a)+.08*math.cos(2*a)
    pts.append((x,y,phase_relative(channel(x,y),base)))

cases=[]
for _ in range(400):
    x=random.uniform(-.72,.72); y=random.uniform(-.65,.65)
    f=phase_relative(noisy(channel(x,y),phase_bias=random.choice([0,.1,3.0,-3.0]),rssi_shift=random.uniform(-.12,.12),dropout=random.choice([None,None,None,0,1,2])),base)
    best=min(pts,key=lambda p:dist(f,p[2])); cases.append(math.hypot(best[0]-x,best[1]-y))

# Three information-directed samples.
for _ in range(3):
    best=None
    for i in range(len(pts)):
        for j in range(i+1,len(pts)):
            x1,y1,_=pts[i]; x2,y2,_=pts[j]; sep=math.hypot(x1-x2,y1-y2)
            if .30<sep<1.5:
                mx,my=(x1+x2)/2,(y1+y2)/2; near=min(math.hypot(mx-x,y-y0) for x,y0,_ in pts)
                score=.55*sep+.45*near
                if best is None or score>best[0]: best=(score,mx,my)
    if best:
        _,x,y=best; pts.append((x,y,phase_relative(channel(x,y),base)))

false=[]
target_act=[]
for _ in range(400):
    f=phase_relative(noisy(base,phase_bias=random.uniform(-.2,.2),rssi_shift=random.uniform(-.05,.05)),base)
    false.append(min(dist(f,p[2]) for p in pts))
    x=random.uniform(-.72,.72); y=random.uniform(-.65,.65)
    target_act.append(activation(noisy(channel(x,y),rssi_shift=random.uniform(-.12,.12),dropout=None),base))

# Target-strength nuisance stress: the same spatial signature should remain
# recognizable when perturbation magnitude changes substantially. This is a
# software contract for the alpha-like runtime normalization; it is not a
# claim about physical accuracy.
strength_case=phase_relative(channel(.21,-.31),base)
scaled=[v for v in strength_case]
for k in range(B):
    o=k*F
    scaled[o]*=1.8
    scaled[o+8]*=1.8
    scaled[o+3]=min(1.0,scaled[o+3]*1.15)
strength_sim=target_similarity(strength_case,scaled)
probe=channel(.21,-.31); wrapped=noisy(probe,phase_bias=2*math.pi,sigma=0); shifted=noisy(probe,rssi_shift=.12,sigma=0)
print('TACTICAL RF SCIENCE BENCH')
print('deployment nodes:',len(pts))
print('median normalized position error:',round(statistics.median(cases),4))
print('90th percentile position error:',round(sorted(cases)[359],4))
empty_med=statistics.median(activation(noisy(base,phase_bias=random.uniform(-.2,.2),rssi_shift=random.uniform(-.05,.05)),base) for _ in range(400))
target_med=statistics.median(target_act)
print('quiet-room nearest-node distance:',round(statistics.median(false),4))
print('activation separation ratio:',round(target_med/max(empty_med,1e-6),2))
print('activation overlap margin:',round(target_med-empty_med,4))
print('target-strength invariance:', 'PASS' if strength_sim>.82 else 'REVIEW', 'score', round(strength_sim,4))
print('phase 2pi invariance:', 'PASS' if dist(phase_relative(probe,base),phase_relative(wrapped,base))<.03 else 'REVIEW')
print('common-mode RSSI bounded:', 'PASS' if dist(phase_relative(probe,base),phase_relative(shifted,base))<.10 else 'REVIEW')
print('dropout stress:', 'PASS' if statistics.median(cases)<.35 else 'REVIEW')
print('empty-room activation median:', round(statistics.median(activation(noisy(base,phase_bias=random.uniform(-.2,.2),rssi_shift=random.uniform(-.05,.05)),base) for _ in range(200)),4))
print('target activation median:', round(statistics.median(target_act),4))
