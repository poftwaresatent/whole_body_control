#ifndef WBC_M3_CTRL_QH_H
#define WBC_M3_CTRL_QH_H

#define WBC_M3_CTRL_M2S_PORT "9876"

namespace wbc_m3_ctrl {
  
  struct master_to_slave {
    double eepos_x, eepos_y, eepos_z;
  };
  
}

#endif // WBC_M3_CTRL_QH_H
