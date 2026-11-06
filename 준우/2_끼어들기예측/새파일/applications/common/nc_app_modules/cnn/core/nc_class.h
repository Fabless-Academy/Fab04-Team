#ifndef NC_CLASS_H
#define NC_CLASS_H

/*
 * nc_class : class id helpers per network (used by nc_traj).
 * Made by Junwoo from Junyi's nc_traj.c call (nc_class_is_vehicle);
 * replace with Junyi's nc_class.c/h when the team code is merged.
 */

#include "nc_cnn_network_includes.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 1 if class_id is a car / bus / truck for this network (ini class_name order) */
int nc_class_is_vehicle(E_NETWORK_UID net_id, int class_id);

#ifdef __cplusplus
}
#endif

#endif /* NC_CLASS_H */
