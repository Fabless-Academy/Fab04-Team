#include "nc_class.h"

int nc_class_is_vehicle(E_NETWORK_UID net_id, int class_id)
{
    switch (net_id) {
    case NETWORK_TRI_CHIMERA:
        /* yolov8n_ncdl ini: per, car, bus, tru, cycle, mot */
        return class_id == 1 || class_id == 2 || class_id == 3;
    case NETWORK_YOLOV8_DET:
        /* yolov8n_coco ini: per, bik, car, mot, bus, tru */
        return class_id == 2 || class_id == 4 || class_id == 5;
    default:
        return 0;
    }
}
