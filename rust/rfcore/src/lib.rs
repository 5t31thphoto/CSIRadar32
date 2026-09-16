#![no_std]

use core::panic::PanicInfo;

const MAX_B: usize = 6;
const FEAT_PER_B: usize = 13;
const FEATS: usize = MAX_B * FEAT_PER_B;
const MAX_NODES: usize = 72;
const MAX_EDGES: usize = 144;
const MAX_TRACKS: usize = 4;
const FIELD: usize = 20;
const MAX_CAND: usize = 4;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! { loop {} }

#[repr(C)]
#[derive(Clone, Copy)]
pub struct RfNode {
    pub x: f32, pub y: f32, pub s: f32, pub weight: f32,
    pub kind: u8, pub nfeat: u8, pub _pad: [u8; 2],
    pub feat: [f32; FEATS],
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct RfTrack {
    pub active: u8, pub direction_valid: u8, pub _pad: [u8; 2],
    pub x: f32, pub y: f32, pub vx: f32, pub vy: f32,
    pub score: f32, pub confidence: f32, pub last_ms: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct RfSnapshot {
    pub frame_ms: u32,
    pub motion: f32,
    pub occupancy: f32,
    pub kind: u8,
    pub bearing_valid: u8,
    pub track_count: u8,
    pub _pad: u8,
    pub perimeter_s: f32,
    pub perimeter_ds: f32,
    pub bearing: f32,
    pub bearing_conf: f32,
    pub field: [u8; FIELD * FIELD],
    pub tracks: [RfTrack; MAX_TRACKS],
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct RfCheck {
    pub valid: u8,
    pub kind: u8,
    pub _pad: [u8; 2],
    pub x: f32, pub y: f32, pub score: f32,
}

#[derive(Clone, Copy)] struct Edge { a: u8, b: u8 }

struct Engine {
    nodes: [RfNode; MAX_NODES], node_n: usize,
    edges: [Edge; MAX_EDGES], edge_n: usize,
    active: bool, ready: bool,
    tracks: [RfTrack; MAX_TRACKS],
    trail: [u8; FIELD * FIELD],
    last_ms: u32, last_x: f32, last_y: f32, last_s: f32,
    baseline: [f32; FEATS], base_valid: bool,
    observation_count: u32,
    last_check_x: f32, last_check_y: f32, check_valid: bool,
    feat_w: [f32; FEATS],
    sensitivity: f32,
}

impl Engine {
    const fn new() -> Self {
        const N: RfNode = RfNode { x:0.0,y:0.0,s:0.0,weight:1.0,kind:0,nfeat:0,_pad:[0;2],feat:[0.0;FEATS] };
        const T: RfTrack = RfTrack { active:0,direction_valid:0,_pad:[0;2],x:0.0,y:0.0,vx:0.0,vy:0.0,score:0.0,confidence:0.0,last_ms:0 };
        Self { nodes:[N;MAX_NODES],node_n:0,edges:[Edge{a:0,b:0};MAX_EDGES],edge_n:0,
                active:false,ready:false,tracks:[T;MAX_TRACKS],trail:[0;FIELD*FIELD],last_ms:0,
                last_x:0.0,last_y:0.0,last_s:0.0,baseline:[0.0;FEATS],base_valid:false,
                observation_count:0,last_check_x:0.0,last_check_y:0.0,check_valid:false,
                feat_w:[1.0;FEATS],sensitivity:1.0 }
    }
    fn reset(&mut self) { *self=Self::new(); }

    fn feat_distance(a:&[f32;FEATS],b:&[f32;FEATS],nb:usize)->f32 {
        let mut sum=0.0; let mut w=0.0; let mut k=0;
        while k<nb && k<MAX_B {
            let o=k*FEAT_PER_B; let q=a[o+4].max(0.03);
            let da=a[o]-b[o];
            let dp=(a[o+1]*b[o+1]+a[o+2]*b[o+2]).clamp(-1.0,1.0);
            let dt=a[o+3]-b[o+3];
            let dr=(a[o+5]-b[o+5])*0.5;
            let dad=(a[o+8]-b[o+8])*0.5;
            let drd=(a[o+9]-b[o+9])*0.5;
            let dn=(a[o+10]-b[o+10])*0.45;
            let dpr=(a[o+11]-b[o+11])*0.35;
            let dcad=(a[o+12]-b[o+12])*0.30;
            let dqa=a[o+4]-b[o+4];
            let an=(a[o+6]*a[o+6]+a[o+7]*a[o+7]).sqrt();
            let bn=(b[o+6]*b[o+6]+b[o+7]*b[o+7]).sqrt();
            let daoa=if an>0.12 && bn>0.12 {(a[o+6]*b[o+6]+a[o+7]*b[o+7])/(an*bn)} else {1.0};
            sum += q*(da*da*0.28 + (1.0-dp)*0.18 + dt*dt*0.04 + dr*dr*0.035 + dad*dad*0.13 + drd*drd*0.12 + dn*dn*0.07 + dpr*dpr*0.05 + dcad*dcad*0.05 + dqa*dqa*0.05 + (1.0-daoa.clamp(-1.0,1.0))*0.18);
            w += q; k+=1;
        }
        if w>0.0 {(sum/w).sqrt()} else {9.0}
    }

    fn similarity(&self,a:&[f32;FEATS],b:&[f32;FEATS],nb:usize)->f32 {
        let mut sum=0.0; let mut weight=0.0; let mut k=0;
        while k<nb && k<MAX_B {
            let o=k*FEAT_PER_B; let q=a[o+4].max(0.03)*b[o+4].max(0.03);
            let amp=(1.0-(a[o]-b[o]).abs()).max(0.0);
            let phase=(a[o+1]*b[o+1]+a[o+2]*b[o+2]).clamp(-1.0,1.0)*0.5+0.5;
            let temporal=(1.0-(a[o+3]-b[o+3]).abs()).max(0.0);
            let rssi=(1.0-(a[o+5]-b[o+5]).abs()).max(0.0);
            let amp_diff=(1.0-(a[o+8]-b[o+8]).abs()).max(0.0);
            let rssi_diff=(1.0-(a[o+9]-b[o+9]).abs()).max(0.0);
            let noise=(1.0-(a[o+10]-b[o+10]).abs()).max(0.0);
            let phy=(1.0-(a[o+11]-b[o+11]).abs()).max(0.0);
            let cadence=(1.0-(a[o+12]-b[o+12]).abs()).max(0.0);
            let an=(a[o+6]*a[o+6]+a[o+7]*a[o+7]).sqrt();
            let bn=(b[o+6]*b[o+6]+b[o+7]*b[o+7]).sqrt();
            let aoa=if an>0.12 && bn>0.12 {(a[o+6]*b[o+6]+a[o+7]*b[o+7])/(an*bn)} else {1.0};
            let aoa=aoa.clamp(-1.0,1.0)*0.5+0.5;
            sum += q*(
                self.feat_w[o+0]*0.30*amp +
                self.feat_w[o+1]*0.09*phase + self.feat_w[o+2]*0.09*phase +
                self.feat_w[o+3]*0.035*temporal + self.feat_w[o+5]*0.04*rssi +
                self.feat_w[o+8]*0.10*amp_diff + self.feat_w[o+9]*0.12*rssi_diff +
                self.feat_w[o+4]*0.05*(1.0-(a[o+4]-b[o+4]).abs()) +
                (self.feat_w[o+6]+self.feat_w[o+7])*0.08*aoa);
            weight += q; k+=1;
        }
        if weight>0.0 {sum/weight} else {0.0}
    }

    // Runtime target matching is deliberately less sensitive to total target
    // strength than node-to-node chart construction.  A person's posture,
    // aspect and exact body position can scale the perturbation without
    // changing the underlying multi-beacon signature.  Estimate a bounded
    // nuisance gain (alpha) from amplitude energy, then score both absolute
    // and differential structure after removing that gain.
    fn target_similarity(&self,a:&[f32;FEATS],b:&[f32;FEATS],nb:usize)->f32 {
        let mut num=0.0; let mut den=0.0; let mut k=0;
        while k<nb&&k<MAX_B {
            let o=k*FEAT_PER_B; let q=a[o+4].max(0.03)*b[o+4].max(0.03);
            num+=q*a[o]*b[o]; den+=q*b[o]*b[o]; k+=1;
        }
        let alpha=if den>0.0004{(num/den).sqrt().clamp(0.35,2.8)}else{1.0};
        let mut sum=0.0; let mut weight=0.0; k=0;
        while k<nb&&k<MAX_B {
            let o=k*FEAT_PER_B;
            let q=a[o+4].max(0.03)*b[o+4].max(0.03);
            let amp=(1.0-(a[o]-alpha*b[o]).abs()).max(0.0);
            let phase=(a[o+1]*b[o+1]+a[o+2]*b[o+2]).clamp(-1.0,1.0)*0.5+0.5;
            let temporal=(1.0-(a[o+3]-b[o+3]).abs()).max(0.0);
            let rssi=(1.0-(a[o+5]-b[o+5]).abs()).max(0.0);
            let amp_diff=(1.0-(a[o+8]-b[o+8]).abs()).max(0.0);
            let rssi_diff=(1.0-(a[o+9]-b[o+9]).abs()).max(0.0);
            let noise=(1.0-(a[o+10]-b[o+10]).abs()).max(0.0);
            let phy=(1.0-(a[o+11]-b[o+11]).abs()).max(0.0);
            let cadence=(1.0-(a[o+12]-b[o+12]).abs()).max(0.0);
            let an=(a[o+6]*a[o+6]+a[o+7]*a[o+7]).sqrt();
            let bn=(b[o+6]*b[o+6]+b[o+7]*b[o+7]).sqrt();
            let aoa=if an>0.12&&bn>0.12{((a[o+6]*b[o+6]+a[o+7]*b[o+7])/(an*bn)).clamp(-1.0,1.0)*0.5+0.5}else{1.0};
            sum+=q*(self.feat_w[o]*0.23*amp+self.feat_w[o+1]*0.08*phase+self.feat_w[o+2]*0.08*phase+self.feat_w[o+3]*0.035*temporal+self.feat_w[o+5]*0.035*rssi+self.feat_w[o+8]*0.14*amp_diff+self.feat_w[o+9]*0.14*rssi_diff+self.feat_w[o+10]*0.07*noise+self.feat_w[o+11]*0.05*phy+self.feat_w[o+12]*0.05*cadence+self.feat_w[o+4]*0.05*(1.0-(a[o+4]-b[o+4]).abs())+(self.feat_w[o+6]+self.feat_w[o+7])*0.08*aoa);
            weight+=q; k+=1;
        }
        if weight>0.0{(sum/weight).clamp(0.0,1.0)}else{0.0}
    }

    // Predict a local RF response in the learned chart.  This is intentionally
    // local IDW rather than a global physical path-loss model: the chart's
    // coordinates are empirical, so interpolation is only trusted near
    // observed nodes.
    fn predict_local(&self,x:f32,y:f32,out:&mut [f32;FEATS],nb:usize){
        *out=[0.0;FEATS]; let mut wsum=0.0;
        let mut i=0; while i<self.node_n {
            let dx=x-self.nodes[i].x; let dy=y-self.nodes[i].y; let d2=dx*dx+dy*dy;
            let w=self.nodes[i].weight.max(0.1)/(d2+0.018);
            let mut k=0; while k<nb&&k<MAX_B { let o=k*FEAT_PER_B; let mut j=0; while j<FEAT_PER_B { out[o+j]+=w*self.nodes[i].feat[o+j]; j+=1; } k+=1; }
            wsum+=w; i+=1;
        }
        if wsum>0.0 { let inv=1.0/wsum; let mut k=0; while k<nb&&k<MAX_B {let o=k*FEAT_PER_B;let mut j=0;while j<FEAT_PER_B{out[o+j]*=inv;j+=1;}let ph=(out[o+1]*out[o+1]+out[o+2]*out[o+2]).sqrt();if ph>0.05{out[o+1]/=ph;out[o+2]/=ph;}k+=1;} }
    }

    // A few scale-free local probes turn the weighted node centroid into a
    // sub-node estimate.  This is a bounded 9-point search, not an assumed
    // meter-scale optimizer.
    fn refine_position(&self,a:&[f32;FEATS],x0:f32,y0:f32,nb:usize)->(f32,f32,f32){
        let mut bx=x0; let mut by=y0;
        let mut base=[0.0f32;FEATS]; self.predict_local(bx,by,&mut base,nb);
        let mut bs=self.target_similarity(a,&base,nb);
        let mut step=0.10f32; let mut pass=0;
        while pass<2 {
            let mut best=bs; let mut xx=-1;
            while xx<=1 {let mut yy=-1;while yy<=1 {let x=(bx+(xx as f32)*step).clamp(-1.0,1.0);let y=(by+(yy as f32)*step).clamp(-1.0,1.0);let mut pred=[0.0;FEATS];self.predict_local(x,y,&mut pred,nb);let sc=self.target_similarity(a,&pred,nb);if sc>best{best=sc;bx=x;by=y;}yy+=1;}xx+=1;}
            bs=best; step*=0.42; pass+=1;
        }
        (bx,by,bs)
    }

    fn add_node(&mut self,n:&RfNode){
        if self.node_n < MAX_NODES { self.nodes[self.node_n]=*n; self.node_n+=1; return; }
        // Adaptive coreset: replace the least useful node only when the new
        // response is genuinely novel. This prevents a long walk from simply
        // becoming "last 72 samples wins".
        let mut nearest=9.0; let mut replace=0usize; let mut i=0;
        while i<self.node_n {
            let d=Self::feat_distance(&n.feat,&self.nodes[i].feat,MAX_B);
            if d<nearest {nearest=d;replace=i;}
            i+=1;
        }
        if nearest>0.22 { self.nodes[replace]=*n; }
    }
    fn add_edge(&mut self,a:u8,b:u8){
        if a==b{return;}
        let mut i=0;while i<self.edge_n{let e=self.edges[i];if (e.a==a&&e.b==b)||(e.a==b&&e.b==a){return;}i+=1;}
        if self.edge_n<MAX_EDGES{self.edges[self.edge_n]=Edge{a,b};self.edge_n+=1;}
    }

    fn build_local_edges(&mut self){
        let mut i=0;
        while i<self.node_n {
            let mut best=[9.0f32;3]; let mut ids=[255u8;3]; let mut j=0;
            while j<self.node_n {if i!=j {let d=Self::feat_distance(&self.nodes[i].feat,&self.nodes[j].feat,MAX_B);let mut p=0;while p<3&&d>=best[p]{p+=1;}if p<3{let mut q=2;while q>p{best[q]=best[q-1];ids[q]=ids[q-1];q-=1;}best[p]=d;ids[p]=j as u8;}}j+=1;}
            let mut p=0;while p<3{if ids[p]!=255{self.add_edge(i as u8,ids[p]);}p+=1;} i+=1;
        }
    }

    fn update_track(&mut self,slot:usize,now:u32,x:f32,y:f32,score:f32,conf:f32){
        let t=&mut self.tracks[slot];
        if t.active==0 { t.active=1;t.x=x;t.y=y;t.vx=0.0;t.vy=0.0; }
        else {
            let dt=((now.saturating_sub(t.last_ms)) as f32/1000.0).clamp(0.01,0.6);
            let dx=x-t.x;let dy=y-t.y;let gain=(0.18+0.68*conf).clamp(0.16,0.82);
            t.vx=t.vx*0.62+(dx/dt)*0.38;t.vy=t.vy*0.62+(dy/dt)*0.38;
            t.x+=dx*gain;t.y+=dy*gain;
            t.direction_valid=((t.vx*t.vx+t.vy*t.vy)>0.00045 && conf>0.46) as u8;
        }
        t.score=score;t.confidence=conf;t.last_ms=now;
    }

    fn deposit(&mut self,x:f32,y:f32,value:u8){
        let gx=(((x+1.0)*0.5*((FIELD-1)as f32)).round() as isize).clamp(0,(FIELD-1)as isize);
        let gy=(((1.0-y)*0.5*((FIELD-1)as f32)).round() as isize).clamp(0,(FIELD-1)as isize);
        let mut yy=-2;while yy<=2{let mut xx=-2;while xx<=2{let q=xx*xx+yy*yy;if q<=4{let px=(gx+xx).clamp(0,(FIELD-1)as isize)as usize;let py=(gy+yy).clamp(0,(FIELD-1)as isize)as usize;let idx=py*FIELD+px;self.trail[idx]=self.trail[idx].saturating_add(value/(q as u8+1));}xx+=1;}yy+=1;}
    }


    fn learn_feature_weights(&mut self){
        // Estimate which feature channels actually carry stable spatial
        // information in this deployment. A feature that is nearly constant
        // contributes little; a feature that changes violently among nearby
        // RF nodes is treated as unreliable. This is a tiny reliability
        // learner, not a neural model, and costs only 60 floats of state.
        let mut global=[0.0f32;FEATS]; let mut i=0;
        while i<self.node_n{let mut k=0;while k<FEATS{global[k]+=self.nodes[i].feat[k];k+=1;}i+=1;}
        if self.node_n<3{return;}
        let invn=1.0/(self.node_n as f32); let mut gvar=[0.0f32;FEATS];
        i=0;while i<self.node_n{let mut k=0;while k<FEATS{let d=self.nodes[i].feat[k]-global[k]*invn;gvar[k]+=d*d;k+=1;}i+=1;}
        let mut k=0;while k<FEATS{gvar[k]=(gvar[k]*invn).sqrt().clamp(0.005,1.0);k+=1;}
        k=0;while k<FEATS{
            let mut local=0.0; let mut cnt=0usize; let mut a=0;
            while a<self.node_n{
                let mut best=9.0; let mut bi=0usize; let mut b=0;
                while b<self.node_n{if a!=b{let d=Self::feat_distance(&self.nodes[a].feat,&self.nodes[b].feat,MAX_B);if d<best{best=d;bi=b;}}b+=1;}
                local+=(self.nodes[a].feat[k]-self.nodes[bi].feat[k]).abs();cnt+=1;a+=1;
            }
            let local_mean=if cnt>0{local/(cnt as f32)}else{1.0};
            self.feat_w[k]=(gvar[k]/(0.025+local_mean)).clamp(0.35,2.2);
            k+=1;
        }
        let mut mean=0.0;k=0;while k<FEATS{mean+=self.feat_w[k];k+=1;}mean/=FEATS as f32;
        if mean>0.001{k=0;while k<FEATS{self.feat_w[k]=(self.feat_w[k]/mean).clamp(0.35,2.2);k+=1;}}
    }

    fn embed_chart(&mut self) {
        if self.node_n < 2 { return; }
        // Seed from independent response projections; relax pairwise RF
        // dissimilarities. This creates a relative chart, never metres.
        let mut i=0; while i<self.node_n {
            let mut a=0.0; let mut b=0.0; let mut k=0;
            while k<MAX_B { let o=k*FEAT_PER_B; a+=self.nodes[i].feat[o]*(1.0+(k as f32)*0.13); b+=self.nodes[i].feat[o+3]*(1.0+((MAX_B-k) as f32)*0.09); k+=1; }
            self.nodes[i].x=(a*2.0-1.0).clamp(-1.0,1.0); self.nodes[i].y=(b*2.0-1.0).clamp(-1.0,1.0);
            // The perimeter walk is the one piece of geometry we actually
            // know: its samples occurred in traversal order. Seed those
            // samples around a normalized closed curve. This is topology,
            // not a meter-scale assumption, and prevents a low-dimensional
            // RF chart from folding the perimeter across itself.
            if self.nodes[i].kind==5 {
                let a2=2.0*PI*self.nodes[i].s.rem_euclid(1.0);
                self.nodes[i].x=0.90*a2.cos(); self.nodes[i].y=0.90*a2.sin();
            }
            i+=1;
        }
        let mut it=0; while it<22 {
            let mut nx=[0.0f32;MAX_NODES]; let mut ny=[0.0f32;MAX_NODES]; let mut i=0;
            while i<self.node_n { let mut gx=0.0;let mut gy=0.0;let mut w=0.0;let mut j=0;
                while j<self.node_n { if i!=j { let d=Self::feat_distance(&self.nodes[i].feat,&self.nodes[j].feat,MAX_B).clamp(0.03,1.4); let dx=self.nodes[i].x-self.nodes[j].x;let dy=self.nodes[i].y-self.nodes[j].y;let r=(dx*dx+dy*dy).sqrt().max(0.02);let e=(r-d)/(r+0.05);gx+=e*dx;gy+=e*dy;w+=1.0;} j+=1; }
                nx[i]=(self.nodes[i].x-0.055*gx/w.max(1.0)).clamp(-1.0,1.0); ny[i]=(self.nodes[i].y-0.055*gy/w.max(1.0)).clamp(-1.0,1.0);
                if self.nodes[i].kind==5 {
                    let rr=(nx[i]*nx[i]+ny[i]*ny[i]).sqrt().max(0.001);
                    // Preserve the measured perimeter's cyclic topology while
                    // still allowing RF geometry to deform the ring.
                    let target=0.90;
                    nx[i]=0.78*nx[i]+0.22*target*nx[i]/rr;
                    ny[i]=0.78*ny[i]+0.22*target*ny[i]/rr;
                }
                i+=1; }
            let mut i=0;while i<self.node_n{self.nodes[i].x=nx[i];self.nodes[i].y=ny[i];i+=1;} it+=1;
        }
        let mut anchor=0usize;let mut ai=0;while ai<self.node_n{if self.nodes[ai].kind==7{anchor=ai;break;}ai+=1;}
        let ax=self.nodes[anchor].x;let ay=self.nodes[anchor].y;let mut mx=0.05f32;let mut i=0;while i<self.node_n{self.nodes[i].x-=ax;self.nodes[i].y-=ay;mx=mx.max(self.nodes[i].x.abs()).max(self.nodes[i].y.abs());i+=1;}i=0;while i<self.node_n{self.nodes[i].x=(self.nodes[i].x/mx).clamp(-1.0,1.0);self.nodes[i].y=(self.nodes[i].y/mx).clamp(-1.0,1.0);i+=1;}
    }
    fn delta_features(&self,f:&[f32;FEATS],nb:usize)->[f32;FEATS]{
        let mut d=*f;
        let mut b=0;
        while b<nb&&b<MAX_B{
            let o=b*FEAT_PER_B;
            d[o]=(f[o]-self.baseline[o]).abs().min(1.0);
            // Circular phase delta: rotate the live phase vector by the
            // negative empty-room phase rather than subtracting components.
            let cs=self.baseline[o+2]; let sn=self.baseline[o+1];
            let fs=f[o+2]; let fn_=f[o+1];
            d[o+1]=(fn_*cs-fs*sn).clamp(-1.0,1.0);
            d[o+2]=(fs*cs+fn_*sn).clamp(-1.0,1.0);
            d[o+3]=(f[o+3]-self.baseline[o+3]).abs().min(1.0);
            d[o+5]=(f[o+5]-self.baseline[o+5]).abs().min(1.0);
            d[o+10]=(f[o+10]-self.baseline[o+10]).abs().min(1.0);
            d[o+11]=(f[o+11]-self.baseline[o+11]).abs().min(1.0);
            d[o+12]=(f[o+12]-self.baseline[o+12]).abs().min(1.0);
            // AoA is an observation-space direction, not an empty-room
            // perturbation, so retain it unchanged.
            b+=1;
        }
        d
    }

    fn observe(&mut self,frame:u32,f:&[f32;FEATS],nb:usize,plen:f32)->RfSnapshot {
        let z=RfTrack{active:0,direction_valid:0,_pad:[0;2],x:0.0,y:0.0,vx:0.0,vy:0.0,score:0.0,confidence:0.0,last_ms:0};
        let mut out=RfSnapshot{frame_ms:frame,motion:0.0,occupancy:0.0,kind:0,bearing_valid:0,track_count:0,_pad:0,perimeter_s:0.0,perimeter_ds:0.0,bearing:0.0,bearing_conf:0.0,field:[0;FIELD*FIELD],tracks:[z;MAX_TRACKS]};
        if !self.ready || self.node_n<2 {return out;}
        self.observation_count=self.observation_count.wrapping_add(1);
        let df=if self.base_valid{self.delta_features(f,nb)}else{*f};

        // Calibration signatures are converted to the same baseline-relative
        // response space by mantis_rf_set_baseline(). That makes the live
        // matcher compare like-for-like rather than comparing a perturbation
        // against an absolute RF fingerprint.
        let match_f=df;

        // Cheap activation gate. It rejects a quiet room before expensive
        // spatial work, but does not kill stationary occupancy: a stationary
        // target may continue matching its learned response node.
        let mut activation=0.0; let mut qsum=0.0; let mut bb=0;
        let sens=self.sensitivity.clamp(0.25,4.0);
        let sens_gain=sens.sqrt();
        while bb<nb&&bb<MAX_B{
            let o=bb*FEAT_PER_B; let q=f[o+4].max(0.0);
            // Common-mode changes across all beacons are poor evidence of a
            // person. Differential channels are deliberately included in the
            // gate so a room-wide RF shift does not look like a target.
            let common_amp=(df[o]-df[o+8].abs()*0.55).max(0.0);
            activation += q*(0.34*common_amp+0.12*df[o]+0.16*df[o+3]+0.18*(1.0-df[o+2].abs())+0.20*df[o+8].abs());
            qsum += q; bb+=1;
        }
        if qsum>0.0 { activation/=qsum; }

        let mut ids=[0usize;MAX_CAND]; let mut scores=[0.0f32;MAX_CAND];
        let mut i=0;
        while i<self.node_n {
            let n=&self.nodes[i];
            let d=Self::feat_distance(&match_f,&n.feat,nb); let sim=self.target_similarity(&match_f,&n.feat,nb);
            let continuity=1.0/(1.0+((n.x-self.last_x)*(n.x-self.last_x)+(n.y-self.last_y)*(n.y-self.last_y)).sqrt()*1.6);
            let score=sim*(0.72+0.28*continuity)/(1.0+0.8*d)*n.weight.max(0.1);
            let mut far=true;let mut k=0;while k<MAX_CAND{if scores[k]>0.0{let dx=n.x-self.nodes[ids[k]].x;let dy=n.y-self.nodes[ids[k]].y;if dx*dx+dy*dy<0.055{far=false;break;}}k+=1;}
            if far {let mut w=0;let mut j=1;while j<MAX_CAND{if scores[j]<scores[w]{w=j;}j+=1;}if score>scores[w]{ids[w]=i;scores[w]=score;}}
            i+=1;
        }

        // Matching-pursuit style residual extraction. We cannot assume RF
        // responses add linearly, so the subtraction is deliberately partial;
        // it is only used to expose a second independent spatial mode.
        let mut residual=match_f;
        let mut pass=1;while pass<2 {
            let mut best_i=0usize;let mut best_s=0.0;let mut j=0;
            while j<self.node_n {let s=self.target_similarity(&residual,&self.nodes[j].feat,nb);if s>best_s{best_s=s;best_i=j;}j+=1;}
            if best_s<0.58{break;}
            let mut far=true;let mut c=0;while c<MAX_CAND{if scores[c]>0.0{let dx=self.nodes[ids[c]].x-self.nodes[best_i].x;let dy=self.nodes[ids[c]].y-self.nodes[best_i].y;if dx*dx+dy*dy<0.055{far=false;break;}}c+=1;}
            if far {let mut w=0;while w<MAX_CAND{if scores[w]<0.5{break;}w+=1;}if w<MAX_CAND{ids[w]=best_i;scores[w]=(best_s*0.82).min(1.0);}}
            let mut k=0;while k<nb&&k<MAX_B{let o=k*FEAT_PER_B;let q=0.32;residual[o]=(residual[o]-self.nodes[best_i].feat[o]*q).max(0.0);residual[o+3]=(residual[o+3]-self.nodes[best_i].feat[o+3]*q).max(0.0);k+=1;}
            pass+=1;
        }

        let mut best=0.0;let mut second=0.0;let mut best_i=0usize;let mut count=0;let mut cx=0.0;let mut cy=0.0;let mut sw=0.0;
        i=0;while i<MAX_CAND{if scores[i]>0.0{let n=&self.nodes[ids[i]];if scores[i]>best{second=best;best=scores[i];best_i=ids[i];}else if scores[i]>second{second=scores[i];}cx+=n.x*scores[i];cy+=n.y*scores[i];sw+=scores[i];count+=1;}i+=1;}
        if sw>1e-5{cx/=sw;cy/=sw}else{cx=self.last_x;cy=self.last_y;}
        // Estimate positional ambiguity directly from the candidate cloud.
        // A high score with candidates spread across the chart is not a
        // high-confidence fix; it is an alias/low-information region.
        let mut spread2=0.0; if sw>1e-5 {i=0;while i<MAX_CAND{if scores[i]>0.0{let n=&self.nodes[ids[i]];let dx=n.x-cx;let dy=n.y-cy;spread2+=scores[i]*(dx*dx+dy*dy);}i+=1;}spread2/=sw;}
        let ambiguity=(spread2.sqrt()/0.42).clamp(0.0,1.0);
        let (rx,ry,rs)=self.refine_position(&match_f,cx,cy,nb);
        if rs>best*0.94 {cx=rx;cy=ry;best=0.58*best+0.42*rs;}
        let fit=(best/(best+0.22)).min(1.0);let margin=(best-second).max(0.0)/(best+0.02);
        // Presence evidence is a birth/confidence gate, not a hard tracker
        // gate.  A stationary target is allowed to persist on its learned
        // spatial response after the initial perturbation has settled.
        let presence=((activation-(0.012/sens_gain))/(0.045/sens_gain)).clamp(0.0,1.0);
        let conf=(0.47*fit+0.25*margin+0.18*(count.min(3) as f32/3.0)+0.10*(1.0-ambiguity))
            .min(1.0)*(0.30+0.70*presence)*(1.0-0.42*ambiguity);
        let spatial_motion=((cx-self.last_x).hypot(cy-self.last_y)*10.0).min(1.0);
        let mut temporal=0.0;let mut k=0;while k<nb&&k<MAX_B{temporal+=f[k*FEAT_PER_B+3];k+=1;}if nb>0{temporal/=nb as f32;}
        out.motion=(0.45*activation+0.35*spatial_motion+0.20*temporal).min(1.0);
        out.occupancy=(0.72*fit+0.28*(1.0-(out.motion*0.35))).min(1.0)* (0.20+0.80*presence);

        // Predictive association: candidate slots are not track identities.
        // Each live track predicts forward, then candidates are assigned by a
        // small greedy global cost using position, velocity consistency and
        // RF score. This prevents the old "candidate #0 owns track #0"
        // identity swap when two targets cross.
        i=0;while i<MAX_TRACKS{
            if self.tracks[i].active!=0 {
                let age=frame.saturating_sub(self.tracks[i].last_ms);
                if age>1100 {
                    self.tracks[i].active=0;
                } else if age>90 {
                    // A missing observation must not leave a frozen ghost at
                    // full confidence. Predict briefly, then decay. This is
                    // the standard track-maintenance failure mode we want to
                    // make graceful rather than visually surprising.
                    let dt=(age as f32/1000.0).clamp(0.0,0.6);
                    self.tracks[i].x=(self.tracks[i].x+self.tracks[i].vx*dt).clamp(-1.2,1.2);
                    self.tracks[i].y=(self.tracks[i].y+self.tracks[i].vy*dt).clamp(-1.2,1.2);
                    self.tracks[i].confidence*=0.93;
                    self.tracks[i].score*=0.94;
                    self.tracks[i].last_ms=frame;
                }
            }
            i+=1;
        }
        let mut used=[false;MAX_TRACKS];
        let mut ci=0;
        while ci<MAX_CAND{
            if scores[ci]>0.0{
                let n=&self.nodes[ids[ci]];
                // The strongest hypothesis receives the sub-node RF refinement.
                // Secondary hypotheses remain anchored to their distinct chart
                // nodes so multi-target separation is not destroyed.
                let (nx,ny) = if ids[ci]==best_i {(cx,cy)} else {(n.x,n.y)};
                let mut slot=MAX_TRACKS; let mut bestc=9.0;
                let mut t=0;
                while t<MAX_TRACKS{
                    if self.tracks[t].active!=0 && !used[t]{
                        let tr=self.tracks[t];
                        let dt=((frame.saturating_sub(tr.last_ms)) as f32/1000.0).clamp(0.01,0.45);
                        let px=tr.x+tr.vx*dt; let py=tr.y+tr.vy*dt;
                        let dx=nx-px; let dy=ny-py; let d=(dx*dx+dy*dy).sqrt();
                        let speed=(tr.vx*tr.vx+tr.vy*tr.vy).sqrt();
                        let mut motion_cons=1.0;
                        if speed>0.018 && d>0.005 {
                            let dot=(dx*tr.vx+dy*tr.vy)/(d*speed);
                            motion_cons=0.5+0.5*dot.clamp(-1.0,1.0);
                        }
                        let gate=0.10+0.34*(1.0-tr.confidence*0.55)+speed*0.55;
                        let cost=(d/gate).min(3.0)*0.58+(1.0-motion_cons)*0.17+(1.0-scores[ci])*0.25;
                        if d<gate*1.55 && cost<bestc{bestc=cost;slot=t;}
                    }
                    t+=1;
                }
                if slot<MAX_TRACKS{
                    used[slot]=true;
                    let cf=(scores[ci]/(scores[ci]+0.36)).min(1.0);
                    self.update_track(slot,frame,nx,ny,scores[ci],cf);
                } else {
                    // Births go into the least confident/inactive slot.
                    let mut birth=MAX_TRACKS; let mut weakest=9.0;
                    let mut t=0;while t<MAX_TRACKS{let c=if self.tracks[t].active==0{0.0}else{self.tracks[t].confidence};if !used[t]&&c<weakest{weakest=c;birth=t;}t+=1;}
                    if birth<MAX_TRACKS && (activation>(0.025/sens_gain) || self.tracks.iter().any(|t| t.active!=0)){used[birth]=true;let cf=(scores[ci]/(scores[ci]+0.36)).min(1.0)*(0.30+0.70*presence);self.update_track(birth,frame,nx,ny,scores[ci],cf);}
                }
            }
            ci+=1;
        }
        i=0;while i<MAX_TRACKS{self.tracks[i].x=self.tracks[i].x.clamp(-1.2,1.2);self.tracks[i].y=self.tracks[i].y.clamp(-1.2,1.2);i+=1;}
        out.tracks=self.tracks;for t in self.tracks.iter(){if t.active!=0{out.track_count+=1;}}

        // Boundary is a learned topological property, not distance from the
        // receiver. The chart is translated to the Anchor reference, so a
        // radial threshold around (0,0) would classify one side of the same
        // perimeter differently from the other. Estimate boundary likelihood
        // from proximity/weight of nodes explicitly observed on the perimeter.
        let mut near_perim=9.0f32;
        let mut perim_score=0.0f32;
        let mut perim_s=0.0f32;
        let mut pi=0;
        while pi<self.node_n {
            if self.nodes[pi].kind==5 {
                let d=(self.nodes[pi].x-cx).hypot(self.nodes[pi].y-cy);
                if d<near_perim {near_perim=d;}
                let w=1.0/(1.0+d*7.0);
                perim_score+=w;
                perim_s+=w*self.nodes[pi].s.rem_euclid(1.0);
            }
            pi+=1;
        }
        let boundary=(best_i<self.node_n && self.nodes[best_i].kind==5)
            || (near_perim<0.20 && perim_score>0.65);
        if boundary && conf>0.20 {
            out.kind=2;
            out.perimeter_s=if perim_score>0.01{(perim_s/perim_score).rem_euclid(1.0)}else{self.nodes[best_i].s.rem_euclid(1.0)};
            let mut ds=out.perimeter_s-self.last_s;if ds>0.5{ds-=1.0;}if ds< -0.5{ds+=1.0;}out.perimeter_ds=ds;
        } else if conf<0.24 {out.kind=3;} else {out.kind=1;}

        // Bearing requires independent asymmetry: multi-beacon response plus
        // optional AoA encoded in the feature vector. Never manufacture it
        // from display coordinates when the RF evidence is weak.
        if conf>0.50 && boundary {
            let mut bx=0.0;let mut by=0.0;let mut qsum=0.0;let mut b=0;
            while b<nb&&b<MAX_B{let o=b*FEAT_PER_B;let q=f[o+4].max(0.0);let a=f[o+6].atan2(f[o+7]);bx+=q*a.cos();by+=q*a.sin();qsum+=q;b+=1;}
            if qsum>0.6 {out.bearing=by.atan2(bx);out.bearing_conf=(conf*(qsum/(nb.max(1) as f32))).min(1.0);out.bearing_valid=(out.bearing_conf>0.52) as u8;}
        }

        for v in self.trail.iter_mut(){*v=((*v as u16*238)/256) as u8;}
        i=0;while i<MAX_TRACKS{if self.tracks[i].active!=0{let t=self.tracks[i];let val=(38.0+205.0*t.confidence) as u8;self.deposit(t.x,t.y,val);}i+=1;}
        out.field=self.trail;
        self.last_x=cx;self.last_y=cy;self.last_s=out.perimeter_s;self.last_ms=frame;
        let _=plen;let _=count;
        out
    }

    fn next_check(&mut self)->RfCheck {
        let mut out=RfCheck{valid:0,kind:0,_pad:[0;2],x:0.0,y:0.0,score:0.0};
        if !self.ready || self.node_n<3{return out;}

        // Do not merely point the operator at an existing sample. Search the
        // learned chart for *holes* between informative RF nodes. The midpoint
        // becomes a genuine new coordinate only after the operator measures it.
        // This is active sampling: the device asks for information where the
        // current model is weakest instead of spending all three checks
        // re-measuring places it already knows.
        let mut best=-1.0; let mut bx=0.0; let mut by=0.0;
        let mut i=0;
        while i<self.node_n{
            let mut j=i+1;
            while j<self.node_n{
                let a=&self.nodes[i]; let b=&self.nodes[j];
                let dx=a.x-b.x; let dy=a.y-b.y; let sep=(dx*dx+dy*dy).sqrt();
                if sep>0.24 && sep<1.75{
                    let mx=(a.x+b.x)*0.5; let my=(a.y+b.y)*0.5;
                    let mut near=9.0; let mut k=0;
                    while k<self.node_n{
                        let dd=(self.nodes[k].x-mx).hypot(self.nodes[k].y-my); if dd<near{near=dd;} k+=1;
                    }
                    let rf_gap=Self::feat_distance(&a.feat,&b.feat,MAX_B).min(1.0);
                    // Penalize redundant checks near the already well-sampled
                    // perimeter using the learned boundary nodes, never a
                    // receiver-centered radius.
                    let mut near_boundary=9.0f32;
                    let mut pb=0;
                    while pb<self.node_n {
                        if self.nodes[pb].kind==5 {
                            near_boundary=near_boundary.min((self.nodes[pb].x-mx).hypot(self.nodes[pb].y-my));
                        }
                        pb+=1;
                    }
                    let boundary_penalty=if near_boundary<0.16{0.58}else{1.0};
                    let separation=if self.check_valid && (mx-self.last_check_x).hypot(my-self.last_check_y)<0.24{0.0}else{1.0};
                    let score=(0.48*sep.min(1.0)+0.32*rf_gap+0.20*(near.min(0.8)/0.8))*boundary_penalty*separation;
                    if score>best{best=score;bx=mx;by=my;}
                }
                j+=1;
            }
            i+=1;
        }
        if best<0.0{return out;}
        out.valid=1; out.kind=3; out.x=bx.clamp(-1.0,1.0); out.y=by.clamp(-1.0,1.0); out.score=best;
        self.last_check_x=out.x; self.last_check_y=out.y; self.check_valid=true; out
    }
}

static mut E:Engine=Engine::new();
#[no_mangle]pub extern "C"fn mantis_rf_sizeof_node()->usize{core::mem::size_of::<RfNode>()}
#[no_mangle]pub extern "C"fn mantis_rf_sizeof_snapshot()->usize{core::mem::size_of::<RfSnapshot>()}
#[no_mangle]pub extern "C"fn mantis_rf_sizeof_check()->usize{core::mem::size_of::<RfCheck>()}
#[no_mangle]pub unsafe extern "C"fn mantis_rf_reset(){E.reset()}
#[no_mangle]pub unsafe extern "C"fn mantis_rf_begin(){E.active=true;E.ready=false}
#[no_mangle]pub unsafe extern "C"fn mantis_rf_add_node(n:*const RfNode){if !n.is_null(){E.add_node(&*n)}}
// Add a post-calibration observation in the same baseline-relative space as
// runtime frames. This is deliberately separate from mantis_rf_add_node():
// deployment samples arrive before the baseline exists and are converted once
// by mantis_rf_set_baseline(), while active-check samples arrive afterwards.
#[no_mangle]pub unsafe extern "C"fn mantis_rf_add_live_node(n:*const RfNode){
    if n.is_null(){return}
    let mut live=*n;
    live.feat=E.delta_features(&live.feat,MAX_B);
    E.add_node(&live);
}
#[no_mangle]pub unsafe extern "C"fn mantis_rf_add_edge(a:u8,b:u8){E.add_edge(a,b)}
#[no_mangle]pub unsafe extern "C"fn mantis_rf_finalize()->u8{E.learn_feature_weights();E.embed_chart();E.build_local_edges();E.ready=E.node_n>=2;E.ready as u8}
// Refresh statistics/graph after an active check without re-embedding the
// chart.  Check coordinates are operator-measured coordinates in the already
// established chart; re-embedding here would silently move that measurement
// and corrupt the very calibration point we just paid to collect.
#[no_mangle]pub unsafe extern "C"fn mantis_rf_refresh_live_model(){if E.node_n>=2{E.learn_feature_weights();E.build_local_edges();E.ready=true;}}
#[no_mangle]pub unsafe extern "C"fn mantis_rf_set_sensitivity(v:f32){E.sensitivity=v.clamp(0.25,4.0);}
#[no_mangle]pub unsafe extern "C"fn mantis_rf_set_baseline(f:*const f32,n:usize){
    if f.is_null(){return}
    let m=n.min(FEATS);
    let src=core::slice::from_raw_parts(f,m);
    E.baseline[..m].copy_from_slice(src);
    // Convert the already-collected deployment signatures into the same
    // response space used by live observations. This is done once, not on
    // every frame, so the runtime remains cheap.
    let mut i=0;
    while i<E.node_n{
        let raw=E.nodes[i].feat;
        E.nodes[i].feat=E.delta_features(&raw,MAX_B);
        i+=1;
    }
    E.base_valid=true;
}
#[no_mangle]pub unsafe extern "C"fn mantis_rf_next_check(out:*mut RfCheck)->u8{if out.is_null(){return 0}*out=E.next_check();(*out).valid}
#[no_mangle]pub unsafe extern "C"fn mantis_rf_observe(frame:u32,features:*const f32,nb:u8,plen:f32,out:*mut RfSnapshot)->u8{if features.is_null()||out.is_null(){return 0}let mut f=[0.0;FEATS];let n=(nb as usize).min(MAX_B)*FEAT_PER_B;let src=core::slice::from_raw_parts(features,n);f[..n].copy_from_slice(src);*out=E.observe(frame,&f,nb as usize,plen);1}
