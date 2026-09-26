// Weak fallback for local Arduino builds when the Xtensa Rust toolchain is
// unavailable. CI links the real Rust staticlib and its strong symbols win.
#include "rfcore.h"
#include <string.h>
extern "C" {
__attribute__((weak)) size_t mantis_rf_sizeof_node(void){return sizeof(MantisRfNode);}
__attribute__((weak)) size_t mantis_rf_sizeof_snapshot(void){return sizeof(MantisRfSnapshot);}
__attribute__((weak)) size_t mantis_rf_sizeof_check(void){return sizeof(MantisRfCheck);}
__attribute__((weak)) void mantis_rf_reset(void){}
__attribute__((weak)) void mantis_rf_begin(void){}
__attribute__((weak)) void mantis_rf_add_node(const MantisRfNode*){}
__attribute__((weak)) void mantis_rf_add_live_node(const MantisRfNode*){}
__attribute__((weak)) void mantis_rf_add_edge(uint8_t,uint8_t){}
__attribute__((weak)) uint8_t mantis_rf_finalize(void){return 0;}
__attribute__((weak)) void mantis_rf_refresh_live_model(void){}
__attribute__((weak)) uint8_t mantis_rf_next_check(MantisRfCheck*out){if(out)memset(out,0,sizeof(*out));return 0;}
__attribute__((weak)) void mantis_rf_set_baseline(const float*,size_t){}
__attribute__((weak)) void mantis_rf_set_sensitivity(float){}
__attribute__((weak)) uint8_t mantis_rf_observe(uint32_t,const float*,uint8_t,float,MantisRfSnapshot*out){if(out)memset(out,0,sizeof(*out));return 0;}
}
