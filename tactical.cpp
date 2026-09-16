#include "tactical.h"
#include "scene.h"
#include "lgfx_tdisplay_s3.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#ifndef PI
#define PI 3.14159265358979323846f
#endif
#include "rfcore.h"
#include "rfarchive.h"   // PSRAM cold tier

extern LovyanGFX &gfx_sprite();
static inline float tclamp(float v,float lo,float hi){return v<lo?lo:(v>hi?hi:v);}
static TacticalStatus S{};
static TacticalCaptureKind cap_kind=TCAP_NONE;
static LandmarkId cap_a=LM_RX,cap_b=LM_RX;
static uint32_t cap_start=0;
static float cap_ax=0,cap_ay=0,cap_bx=0,cap_by=0;
static uint8_t cap_last_node=0;
static float cap_sum[MANTIS_RF_FEATS]={0};
static float walk_feat[8][MANTIS_RF_FEATS]={{0}};
static uint16_t walk_n=0; static uint8_t walk_write=0; static uint16_t cap_n=0;
static uint8_t deploy_idx=0; static uint8_t deploy_done=0;
static int8_t lm_node[LM_COUNT];
static bool model_started=false;
static bool check_pending=false;
static float check_x=0,check_y=0;
static const char *check_cue="";
static uint8_t check_count=0;
// Probe budget.  A deployment walk sampled the room the model's own way,
// so three checks is a top-up.  An ADOPTED chart was built from a
// different ceremony -- no perimeter circuit, no PHY-metadata axis,
// response time-averaged rather than walked -- so spot probing is the
// only thing that closes that difference, and it is by far the cheapest
// way to do it.  Adoption raises this to 6.
static uint8_t s_check_budget=3;
static float node_q[3]={999,999,999};
static float base_mean[MANTIS_RF_FEATS]={0};
static float base_var[MANTIS_RF_FEATS]={0};
static uint16_t base_n=0;
static float last_feat[MANTIS_RF_FEATS]={0};
static bool have_last=false;
static bool last_capture_committed=false;
static float last_sensitivity=-1.0f;

static void feat_from_obs(const FrameObservation&o,float*f){
  memset(f,0,sizeof(float)*MANTIS_RF_FEATS);
  float amp_sum=0.0f, rssi_sum=0.0f, wsum=0.0f;
  for(int b=0;b<MAX_BEACONS;b++){
    if(b>=o.n_beacons) continue;
    const auto&p=o.beacon[b]; if(!p.fresh) continue;
    const float q=tclamp(p.quality,0,1);
    amp_sum += q*tclamp(p.amp_perturbation,0,1);
    rssi_sum += q*tclamp((p.rssi+100.0f)/60.0f,0,1);
    wsum += q;
  }
  const float amp_ref=wsum>0.01f?amp_sum/wsum:0.0f;
  const float rssi_ref=wsum>0.01f?rssi_sum/wsum:0.0f;
  for(int b=0;b<MAX_BEACONS;b++){
    const int k=b*MANTIS_RF_FEAT_PER_B;
    f[k+2]=1.0f; f[k+7]=1.0f; f[k+8]=0.0f; f[k+9]=0.0f; f[k+10]=0.0f; f[k+11]=0.0f; f[k+12]=0.33f;
    if(b>=o.n_beacons) continue;
    const auto&p=o.beacon[b]; if(!p.fresh) continue;
    f[k+0]=tclamp(p.amp_perturbation,0,1);
    const float ph=p.phase_perturbation;
    f[k+1]=sinf(ph); f[k+2]=cosf(ph);
    f[k+3]=tclamp(p.temporal_delta/30.0f,0,1);
    f[k+4]=tclamp(p.quality,0,1);
    // RSSI remains an independent old-school cue; it is normalized only for
    // comparison and is never interpreted as a physical distance.
    f[k+5]=tclamp((p.rssi+100.0f)/60.0f,0,1);
    if(p.have_aoa && p.aoa_conf>0.15f){f[k+6]=sinf(p.aoa_rad)*p.aoa_conf;f[k+7]=cosf(p.aoa_rad)*p.aoa_conf;}
    // Cross-beacon differential cues are deliberately primitive. They often
    // survive common-mode RF drift better than absolute values do.
    f[k+8]=tclamp((f[k+0]-amp_ref)*2.0f,-1,1);
    f[k+9]=tclamp((f[k+5]-rssi_ref)*2.0f,-1,1);
    // Non-CSI RF metadata: noise-floor and received PHY-rate response.
    // They are environmental observables, not distance estimates.
    // Keep raw PHY metadata as bounded encodings. The ESP RX-control
    // representation is version-dependent (and noise-floor may be zero on
    // some silicon/driver combinations), so we do not pretend these are
    // calibrated dBm or Mbps measurements.
    f[k+10]=tclamp((p.noise_floor+128.0f)/255.0f,0,1);
    f[k+11]=tclamp(p.phy_rate,0,1);
    // Packet cadence is another RF observable: blockage/interference can
    // change the arrival process even when a single CSI frame looks benign.
    f[k+12]=tclamp(p.interarrival_ms/100.0f,0,1);
    // Keep PHY metadata in its own observation dimensions. Baseline-relative
    // conversion happens once inside Rust, so the same representation is
    // used for deployment, active checks, and runtime observations.
  }
}
// live_space defaults to false.
//
// v1.7 shipped this with six required parameters and a five-argument
// call on the walk path, so the file did not compile.  The default is
// the correct repair rather than passing false explicitly at the call
// site: walk/deploy/return/circuit samples are DEPLOYMENT geometry and
// belong in the model space, whereas only an active CHECK adds into the
// live space.  Defaulting to model space makes that the safe case and
// keeps the exception explicit at the one site that needs it.
static void add_node(float x,float y,float s,const float*f,uint8_t kind,bool live_space=false){
  MantisRfNode n{}; n.x=x;n.y=y;n.s=s;n.weight=1;n.kind=kind;n.nfeat=MANTIS_RF_FEATS;memcpy(n.feat,f,sizeof(n.feat));

  // ── COLD TIER ─────────────────────────────────────────────────
  // The Rust coreset is bounded and evicts by novelty, which is right
  // for a scorer that must stay fast but does throw away real
  // measurements.  Archive every node in PSRAM first, so a later
  // re-fit can choose its coreset from the FULL population rather than
  // from whichever samples happened to arrive before it filled.
  //
  // Runtime is unaffected: nothing in the solve path reads the archive.
  // If PSRAM is absent this is a no-op and behaviour is unchanged.
  rf_archive_add(&n);

  if(live_space) mantis_rf_add_live_node(&n); else mantis_rf_add_node(&n);
  cap_last_node++;
  if(cap_kind==TCAP_STAND&&cap_a<LM_COUNT)lm_node[cap_a]=(int8_t)(cap_last_node-1);
}
void tactical_begin(){mantis_rf_reset();rf_archive_clear();s_check_budget=3;memset(lm_node,-1,sizeof(lm_node));memset(node_q,0,sizeof(node_q));mantis_rf_begin();S={};S.active=true;cap_last_node=0;cap_kind=TCAP_NONE;deploy_idx=0;deploy_done=0;model_started=true;check_count=0;check_pending=false;have_last=false;last_capture_committed=false;last_sensitivity=-1.0f;base_n=0;memset(base_mean,0,sizeof(base_mean));memset(base_var,0,sizeof(base_var));}
void tactical_reset_model(){tactical_begin();}
void tactical_begin_deploy(uint8_t beacon_index){
  deploy_idx=beacon_index;
  tactical_begin_capture(TCAP_DEPLOY,LM_RX,LM_RX);
}
void tactical_begin_return(){
  tactical_begin_capture(TCAP_RETURN,LM_RX,LM_RX);
}
void tactical_begin_check(float x,float y){
  last_capture_committed=false;
  cap_kind=TCAP_CHECK; cap_a=LM_CENTROID; cap_b=LM_CENTROID; cap_start=millis();
  cap_n=0; walk_n=0; walk_write=0; memset(cap_sum,0,sizeof(cap_sum)); memset(walk_feat,0,sizeof(walk_feat));
  cap_ax=x; cap_ay=y; cap_bx=x; cap_by=y;
}
void tactical_begin_capture(TacticalCaptureKind k,LandmarkId a,LandmarkId b){last_capture_committed=false;cap_kind=k;cap_a=a;cap_b=b;cap_start=millis();cap_n=0;walk_n=0;memset(cap_sum,0,sizeof(cap_sum));memset(walk_feat,0,sizeof(walk_feat));walk_write=0;scene_landmark_pos(a,&cap_ax,&cap_ay);scene_landmark_pos(b,&cap_bx,&cap_by);}
bool tactical_end_capture(){
  last_capture_committed=false;
  if((cap_kind==TCAP_STAND || cap_kind==TCAP_CHECK) && cap_n>4){
    float avg[MANTIS_RF_FEATS];for(int i=0;i<MANTIS_RF_FEATS;i++)avg[i]=cap_sum[i]/(float)cap_n;
    add_node(cap_ax,cap_ay,0,avg,cap_kind==TCAP_STAND?1:3,cap_kind==TCAP_CHECK);
    last_capture_committed=true;
  }
  if((cap_kind==TCAP_WALK || cap_kind==TCAP_DEPLOY || cap_kind==TCAP_RETURN || cap_kind==TCAP_CIRCUIT) && walk_n>0){
    for(int q=0;q<walk_n;q++){
      const uint8_t idx=(walk_n<8)?(uint8_t)q:(uint8_t)((walk_write+q)%8);
      float x=0,y=0,sn=0; uint8_t kind=2;
      if(cap_kind==TCAP_DEPLOY){ kind=(uint8_t)(4+(deploy_idx&7)); }
      else if(cap_kind==TCAP_RETURN){ kind=6; }
      else if(cap_kind==TCAP_CIRCUIT){ kind=5; sn=(float)q/(float)(walk_n>1?walk_n-1:1); }
      else { float t=(q+1)/((float)walk_n+1); x=cap_ax+(cap_bx-cap_ax)*t; y=cap_ay+(cap_by-cap_ay)*t; }
      if(cap_kind==TCAP_RETURN && q==walk_n-1) kind=7; // PROBE returned to ANCHOR: chart origin
      add_node(x,y,sn,walk_feat[idx],kind);
      last_capture_committed=true;
    }
    if(cap_kind==TCAP_DEPLOY) deploy_done++;
  }
  cap_kind=TCAP_NONE;
  return last_capture_committed;
}
void tactical_observe(const FrameObservation&o){if(!model_started)return;float f[MANTIS_RF_FEATS];feat_from_obs(o,f);memcpy(last_feat,f,sizeof(f));have_last=true;
  if(cap_kind==TCAP_BASELINE){
    bool bad=false; float motion_e=0.0f; float quality_e=0.0f;
    for(int b=0;b<MAX_BEACONS;b++){
      const int k=b*MANTIS_RF_FEAT_PER_B;
      if(f[k+4] < 0.12f) continue;
      const float amp=f[k+0];
      const float temp=f[k+3];
      motion_e += 0.70f*amp*amp + 0.30f*temp*temp;
      if(amp>0.18f || temp>0.32f) bad=true;
      quality_e += f[k+4];
    }
    S.contaminated=bad;
    if(!bad){
      S.contaminated=false; base_n++;
      for(int i=0;i<MANTIS_RF_FEATS;i++){
        float v=f[i];
        // Circular phase components are averaged as vectors; the Rust side
        // renormalizes the resulting phase reference before differencing.
        float d=v-base_mean[i]; base_mean[i]+=d/base_n; base_var[i]+=d*(v-base_mean[i]);
      }
    }
    // Normalize by the beacons actually contributing.  MAX_BEACONS is a
    // storage ceiling, not the deployed sensor count; dividing by six would
    // make a perfectly healthy three-beacon deployment look half-dead.
    int active_b=0;
    for(int b=0;b<MAX_BEACONS;b++){ if(f[b*MANTIS_RF_FEAT_PER_B+4] >= 0.12f) active_b++; }
    const float denom=(active_b>0)?(float)active_b:1.0f;
    S.baseline_quality=tclamp(1.0f-motion_e/(denom*0.055f),0,1)*tclamp((quality_e/denom)/0.55f,0,1);
    return;
  }
  if((cap_kind==TCAP_STAND || cap_kind==TCAP_CHECK) && o.n_beacons>=1){for(int i=0;i<MANTIS_RF_FEATS;i++)cap_sum[i]+=f[i];cap_n++;}
  if((cap_kind==TCAP_WALK || cap_kind==TCAP_DEPLOY || cap_kind==TCAP_RETURN || cap_kind==TCAP_CIRCUIT) && o.n_beacons>=1){
    memcpy(walk_feat[walk_write],f,sizeof(f));
    walk_write=(uint8_t)((walk_write+1u)%8u);
    if(walk_n<8) walk_n++;
    cap_n++;
  }
}
void tactical_begin_baseline(){cap_kind=TCAP_BASELINE;S.baseline_ready=false;S.contaminated=false;base_n=0;S.baseline_quality=0;}
void tactical_accept_baseline(){
  if(base_n<30)return;
  float b[MANTIS_RF_FEATS]={};
  memcpy(b,base_mean,sizeof(b));
  // Quality is a confidence weight, not a physical baseline feature.
  for(int i=0;i<MAX_BEACONS;i++){
    const int k=i*MANTIS_RF_FEAT_PER_B;
    b[k+4]=1.0f;
    // AoA has no useful empty-room direction. Keep it neutral.
    b[k+6]=0.0f; b[k+7]=1.0f;
    const float phn=sqrtf(b[k+1]*b[k+1]+b[k+2]*b[k+2]);
    if(phn>0.05f){b[k+1]/=phn;b[k+2]/=phn;} else {b[k+1]=0;b[k+2]=1;}
  }
  mantis_rf_set_baseline(b,MANTIS_RF_FEATS);
  S.baseline_ready=true; cap_kind=TCAP_NONE;
}
bool tactical_baseline_ready(){return S.baseline_ready;}float tactical_baseline_quality(){return S.baseline_quality;}bool tactical_baseline_contaminated(){return S.contaminated;}bool tactical_ready(){return S.ready;}
// ═══════════════════════════════════════════════════════════════
//  ADOPT AN EXISTING FULL-MODE CALIBRATION
// ═══════════════════════════════════════════════════════════════
// The Full Mode walk already visited known positions and recorded the
// per-beacon response at each one.  That IS a response graph -- it was
// just being stored in a different shape for a different solver.
//
// What transfers cleanly: position, mean amplitude perturbation, the
// circular-mean phase, AoA when the walk saw it, and a per-beacon
// confidence derived from how many frames actually contributed.
//
// What does NOT transfer: RSSI, noise floor, PHY rate and packet
// cadence.  The kernel never recorded them, so they are written at the
// same neutral constants feat_from_obs() uses for an absent beacon.
// They will be CONSTANT across every adopted node, which means the
// weight learner sees zero variance and down-weights them to nothing.
// That is the correct degradation: an adopted chart simply does not get
// the PHY-metadata axis, and nothing pretends otherwise.
bool tactical_can_adopt(){
  return scene_cal_complete() && scene_kernel_sample_count() >= 8;
}

int tactical_adopt_full_cal(){
  if(!tactical_can_adopt()) return 0;
  const int n = scene_kernel_sample_count();

  tactical_begin();            // clears the model AND the archive

  int seeded = 0;
  for(int i=0;i<n;i++){
    const KernelSample *ks = scene_kernel_sample(i);
    if(!ks) continue;

    float f[MANTIS_RF_FEATS];
    memset(f,0,sizeof(f));
    int contributing = 0;
    for(int b=0;b<MAX_BEACONS;b++){
      const int k=b*MANTIS_RF_FEAT_PER_B;
      // Neutral defaults first, identical to an absent beacon.
      f[k+2]=1.0f; f[k+7]=1.0f; f[k+12]=0.33f;
      const auto &pb = ks->b[b];
      if(pb.sample_count==0) continue;
      contributing++;
      f[k+0]=tclamp(pb.mean_amp,0,1);
      f[k+1]=sinf(pb.mean_phase);
      f[k+2]=cosf(pb.mean_phase);
      // No temporal delta in a kernel sample: it is a time-average.
      f[k+3]=0.0f;
      // Confidence from how much evidence this beacon contributed here,
      // saturating at ~40 frames.  A landmark seen by one frame should
      // not weigh the same as one seen by a hundred.
      f[k+4]=tclamp(0.25f+0.75f*((float)pb.sample_count/40.0f),0,1);
      // RSSI/noise/PHY/cadence: unknown, held at the neutral constants.
      f[k+5]=0.0f; f[k+8]=0.0f; f[k+9]=0.0f; f[k+10]=0.0f; f[k+11]=0.0f;
      if(pb.saw_aoa>0){
        const float c = tclamp((float)pb.saw_aoa/20.0f,0,1);
        f[k+6]=sinf(pb.mean_aoa_dev)*c;
        f[k+7]=cosf(pb.mean_aoa_dev)*c;
      }
    }
    if(contributing==0) continue;   // an empty kernel sample teaches nothing

    // ── KIND MATTERS ──────────────────────────────────────────────
    // Tagging every adopted sample as a plain stand would cost the chart
    // its topology.  The Rust boundary test keys on proximity to nodes
    // observed ON the perimeter (kind 5), and the origin on kind 7 -- so
    // an undifferentiated blob of stands has NO boundary, which means no
    // exterior detection and no perimeter coordinate at all.
    //
    // A Full Mode walk already visited the perimeter: the beacon
    // landmarks ARE the ring, and LM_OPPOSITE_RX is a deliberate
    // outside-the-ring point.  Map them across, and give the ring
    // samples a cyclic s from their angular order so the 1-D perimeter
    // coordinate is real rather than invented.
    uint8_t kind = 1;          // stand
    float   sn   = 0.0f;
    const uint8_t lm = ks->landmark_id;
    if (lm == LM_RX) {
        kind = 7;              // chart origin
    } else if (lm >= LM_BEACON_1 && lm < (uint8_t)(LM_BEACON_1 + MAX_BEACONS)) {
        kind = 5;              // on the perimeter
        const int ring_i = (int)lm - (int)LM_BEACON_1;
        const int ring_n = (g_app.beacon_count > 0) ? g_app.beacon_count : 1;
        sn = (float)ring_i / (float)ring_n;
    } else if (lm == LM_OPPOSITE_RX) {
        kind = 5;              // outside the ring, still a boundary sample
        sn   = 0.5f;           // opposite side of the cycle
    }
    // Transits and midpoints stay kind 1: they are interior response
    // samples, which is exactly what they were.

    MantisRfNode nd{};
    nd.x=ks->pos[0]; nd.y=ks->pos[1]; nd.s=sn; nd.weight=1;
    nd.kind=kind; nd.nfeat=MANTIS_RF_FEATS;
    memcpy(nd.feat,f,sizeof(nd.feat));
    rf_archive_add(&nd);
    mantis_rf_add_node(&nd);
    seeded++;
  }

  if(seeded < 8){ return 0; }   // not enough to be worth trusting

  // The empty-room reference transfers too: Full Mode measured it, and
  // it is the same physical quantity.  Build it from a zero-perturbation
  // observation, which is exactly what "empty" means in this feature
  // space.
  float base[MANTIS_RF_FEATS];
  memset(base,0,sizeof(base));
  for(int b=0;b<MAX_BEACONS;b++){
    const int k=b*MANTIS_RF_FEAT_PER_B;
    base[k+1]=0.0f; base[k+2]=1.0f;      // zero phase offset
    base[k+4]=1.0f;                      // quality is a weight, not a feature
    base[k+6]=0.0f; base[k+7]=1.0f;      // neutral AoA
    base[k+12]=0.33f;
  }
  mantis_rf_set_baseline(base,MANTIS_RF_FEATS);
  S.baseline_ready=true;
  S.ready = mantis_rf_finalize()!=0;
  check_count = 0;
  s_check_budget = 6;      // adopted: probe harder
  MSLOG("[tac] adopted full cal: %d of %d kernel samples -> %s (probe budget %u)\n",
        seeded, n, S.ready?"ready":"not ready", (unsigned)s_check_budget);
  return seeded;
}

void tactical_finalize_model(){if(base_n>=30&&!S.baseline_ready)tactical_accept_baseline();S.ready=mantis_rf_finalize()!=0;}
// Deliberately does nothing, and that is the design rather than an
// omission.
//
// Full Mode separates observe() from update() because its solver is
// expensive and has to be rate-limited below the observation rate.  The
// RF chart has no such split: mantis_rf_observe() IS the solve, and it
// already runs at the observation rate inside tactical_runtime_observe().
// Adding a second entry point here would either duplicate that work or
// sit empty.
//
// Kept as a symbol so the main loop's shape matches Full Mode's and a
// reader does not go hunting for a missing per-frame call.
void tactical_update(){}
const TacticalStatus&tactical_status(){return S;}

bool tactical_next_check(float*x,float*y,const char**cue,LandmarkId*lm){
  if(!S.ready||check_count>=s_check_budget)return false;
  // The Rust core scores coverage holes, local disagreement and boundary
  // weakness from the actual empirical response graph. This is deliberately
  // not a fixed landmark quota.
  MantisRfCheck c{};
  if(!mantis_rf_next_check(&c) || !c.valid || c.score < 0.34f) return false;
  if(check_pending) return false;
  check_pending=true; check_x=c.x; check_y=c.y;
  *x=check_x; *y=check_y;
  if(c.kind==2) check_cue="CHECK PERIMETER";
  else if(c.kind==3) check_cue="CHECK AMBIGUOUS ZONE";
  else check_cue="CHECK RF FIELD";
  *cue=check_cue;
  if(lm)*lm=LM_CENTROID;
  return true;
}
void tactical_complete_check(){if(!check_pending)return;check_pending=false;check_count++;mantis_rf_refresh_live_model();}
void tactical_cancel_check(){check_pending=false;}

// Called by csi_update_spatial. This is the single Rust inference boundary.
void tactical_runtime_observe(const FrameObservation&o){if(!S.baseline_ready||!S.ready)return;if(fabsf(g_app.sensitivity-last_sensitivity)>0.001f){mantis_rf_set_sensitivity(g_app.sensitivity);last_sensitivity=g_app.sensitivity;}float f[MANTIS_RF_FEATS];feat_from_obs(o,f);MantisRfSnapshot r{};if(!mantis_rf_observe(o.frame_ms,f,o.n_beacons,1.0f,&r))return;S.motion=r.motion;S.occupancy=r.occupancy;S.kind=r.kind;S.track_count=r.track_count;S.perimeter_s=r.perimeter_s;S.perimeter_ds=r.perimeter_ds;S.bearing=r.bearing;S.bearing_conf=r.bearing_conf;memcpy(S.tracks,r.tracks,sizeof(S.tracks));memcpy(S.field,r.field,sizeof(S.field));}

// ── SNAPSHOT SANITISER ────────────────────────────────────────
// Track coordinates come straight out of the Rust core and are converted
// to pixels with a cast.  (int) of a NaN is UNDEFINED BEHAVIOUR in C++,
// and a chart embedding CAN produce NaN -- stress relaxation on
// degenerate input (all nodes coincident, a single-node chart, a divide
// by a zero span) is exactly how that happens.
//
// The result would be a draw call at an arbitrary coordinate with an
// arbitrary radius: at best a corrupted frame, at worst a very long
// fillCircle or an out-of-bounds write inside the graphics library.
//
// The model is new and unproven.  Render defensively: a track the
// sanitiser rejects is simply not drawn, which is honest -- we do not
// know where it is.
static inline bool tac_finite(float v){ return v==v && v<1e6f && v>-1e6f; }
static inline float tac_clampf(float v,float lo,float hi){
  return v<lo?lo:(v>hi?hi:v);
}
// Chart coordinates are normalised to roughly [-1,1]; allow generous
// slack for a track just outside the ring, then clamp.
static bool tac_track_px(float x,float y,float conf,
                         int cx,int cy,float sx,float sy,
                         int*out_px,int*out_py,float*out_conf){
  if(!tac_finite(x)||!tac_finite(y)||!tac_finite(conf)) return false;
  const float cxf=tac_clampf(x,-2.0f,2.0f), cyf=tac_clampf(y,-2.0f,2.0f);
  *out_px = cx + (int)(cxf*sx);
  *out_py = cy - (int)(cyf*sy);
  *out_conf = tac_clampf(conf,0.0f,1.0f);
  return true;
}

void tactical_render_field(){
  auto&g=gfx_sprite();
  const int top=CONTENT_Y+4, w=SCREEN_W, h=230;
  g.drawRect(0,top,w,h,COL_MS_DIM);
  // Same learned RF field as radar, but rendered as a top-down tactical
  // situation map.  There are no kernel/debug overlays here.
  for(int y=0;y<MANTIS_RF_FIELD;y++) for(int x=0;x<MANTIS_RF_FIELD;x++){
    uint8_t v=S.field[y*MANTIS_RF_FIELD+x]; if(v<6) continue;
    int px=x*w/MANTIS_RF_FIELD, py=top+(MANTIS_RF_FIELD-1-y)*h/MANTIS_RF_FIELD;
    uint16_t c=(v>175)?COL_MS_LIME:(v>72?COL_MS_TEAL:COL_MS_VIOLET);
    g.fillRect(px,py,(w/MANTIS_RF_FIELD)+1,(h/MANTIS_RF_FIELD)+1,c);
  }
  const int cx=w/2, cy=top+h/2;
  for(int i=0;i<MANTIS_RF_MAX_TRACKS;i++){
    const auto&t=S.tracks[i]; if(!t.active) continue;
    int px,py; float cf;
    if(!tac_track_px(t.x,t.y,t.confidence,cx,cy,w*.44f,h*.43f,&px,&py,&cf))
      continue;   // not finite: we do not know where this is, so do not draw it
    uint16_t c=cf>.68f?COL_MS_LIME:(cf>.36f?COL_MS_TEAL:COL_MS_VIOLET);
    int r=3+(int)(3.0f*cf);
    g.drawCircle(px,py,r,c);
    g.fillCircle(px,py,2,c);
    if(t.direction_valid){float n=sqrtf(t.vx*t.vx+t.vy*t.vy);if(n>.001f){int dx=(int)(t.vx/n*12),dy=(int)(-t.vy/n*12);g.drawLine(px,py,px+dx,py+dy,c);}}
  }
  g.setFont(&fonts::Font2);
  g.setTextColor(S.occupancy>.35f?COL_MS_LIME:COL_MS_DIM,COL_MS_BG);
  g.setCursor(4,top+2);
  if(S.kind==2)g.print("PERIMETER"); else if(S.kind==1)g.print("INTERIOR"); else if(S.kind==3)g.print("SEARCHING"); else g.print("CLEAR");
  if(S.track_count>0){char tb[16];snprintf(tb,sizeof(tb),"%u TGT",(unsigned)S.track_count);g.setCursor(w-62,top+2);g.print(tb);}
}

void tactical_render_radar(){auto&g=gfx_sprite();const int top=CONTENT_Y+18,w=SCREEN_W,h=190;g.drawRect(0,top,w,h,COL_MS_DIM);int cx=w/2,cy=top+h/2;for(int y=0;y<MANTIS_RF_FIELD;y++)for(int x=0;x<MANTIS_RF_FIELD;x++){uint8_t v=S.field[y*MANTIS_RF_FIELD+x];if(v<8)continue;int px=x*w/MANTIS_RF_FIELD,py=top+(MANTIS_RF_FIELD-1-y)*h/MANTIS_RF_FIELD;uint16_t c=(v>170)?COL_MS_LIME:(v>70?COL_MS_TEAL:COL_MS_VIOLET);g.fillRect(px,py,(w/MANTIS_RF_FIELD)+1,(h/MANTIS_RF_FIELD)+1,c);}for(int i=0;i<MANTIS_RF_MAX_TRACKS;i++){auto&t=S.tracks[i];if(!t.active)continue;
    int px,py; float cf;
    // Same sanitiser as the field view: a non-finite coordinate would be
    // cast to int (UB) and drawn at an arbitrary pixel.
    if(!tac_track_px(t.x,t.y,t.confidence,cx,cy,w*.42f,h*.42f,&px,&py,&cf)) continue;
    uint16_t c=cf>.65f?COL_MS_LIME:(cf>.35f?COL_MS_TEAL:COL_MS_VIOLET);g.fillCircle(px,py,4,c);
    // Velocity feeds three drawLine calls; a NaN there produces three
    // garbage lines rather than one bad pixel.
    if(t.direction_valid&&tac_finite(t.vx)&&tac_finite(t.vy)){float n=sqrtf(t.vx*t.vx+t.vy*t.vy);if(n>.001f){float dx=t.vx/n*10,dy=t.vy/n*10;g.drawLine(px,py,px+(int)dx,py-(int)dy,c);g.drawLine(px+(int)dx,py-(int)dy,px+(int)dx-(int)(dx*.35f-dy*.35f),py-(int)dy+(int)(dy*.35f+dx*.35f),c);g.drawLine(px+(int)dx,py-(int)dy,px+(int)dx-(int)(dx*.35f+dy*.35f),py-(int)dy+(int)(dy*.35f-dx*.35f),c);}}}g.setTextColor(S.motion>.3f?COL_MS_LIME:COL_MS_DIM,COL_MS_BG);g.setCursor(4,CONTENT_Y+2);if(S.kind==1)g.print("TACTICAL / INSIDE");else if(S.kind==2)g.print("TACTICAL / PERIMETER");else if(S.kind==3)g.print("TACTICAL / AMBIGUOUS");else g.print("TACTICAL / QUIET");}
