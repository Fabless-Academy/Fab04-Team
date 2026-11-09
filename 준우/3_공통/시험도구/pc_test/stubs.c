#include <stdio.h>
float nc_meas_get_latency_ms(void);
void nc_rec_trigger(int id);
int g_rec_cnt = 0;
float nc_meas_get_latency_ms(void) { return 100.0f; }
void nc_rec_trigger(int id) { g_rec_cnt++; printf("    >> nc_rec_trigger(%d) call #%d\n", id, g_rec_cnt); }
