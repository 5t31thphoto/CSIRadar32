#pragma once
#include <stdint.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif
#define MANTIS_RF_MAX_B 6
#define MANTIS_RF_FEAT_PER_B 13
#define MANTIS_RF_FEATS 78
#define MANTIS_RF_MAX_TRACKS 4
#define MANTIS_RF_FIELD 20
typedef struct {float x,y,s,weight;uint8_t kind,nfeat,_pad[2];float feat[MANTIS_RF_FEATS];} MantisRfNode;
typedef struct {uint8_t active,direction_valid,_pad[2];float x,y,vx,vy,score,confidence;uint32_t last_ms;} MantisRfTrack;
typedef struct {uint32_t frame_ms;float motion,occupancy;uint8_t kind,bearing_valid,track_count,_pad;float perimeter_s,perimeter_ds,bearing,bearing_conf;uint8_t field[MANTIS_RF_FIELD*MANTIS_RF_FIELD];MantisRfTrack tracks[MANTIS_RF_MAX_TRACKS];} MantisRfSnapshot;
typedef struct {uint8_t valid,kind,_pad[2];float x,y,score;} MantisRfCheck;
size_t mantis_rf_sizeof_node(void);size_t mantis_rf_sizeof_snapshot(void);size_t mantis_rf_sizeof_check(void);
void mantis_rf_reset(void);void mantis_rf_begin(void);void mantis_rf_add_node(const MantisRfNode*);void mantis_rf_add_live_node(const MantisRfNode*);void mantis_rf_add_edge(uint8_t,uint8_t);uint8_t mantis_rf_finalize(void);void mantis_rf_refresh_live_model(void);void mantis_rf_set_baseline(const float*,size_t);void mantis_rf_set_sensitivity(float);uint8_t mantis_rf_next_check(MantisRfCheck*);uint8_t mantis_rf_observe(uint32_t,const float*,uint8_t,float,MantisRfSnapshot*);
#ifdef __cplusplus
}
#endif
