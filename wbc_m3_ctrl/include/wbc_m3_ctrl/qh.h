#ifndef WBC_M3_CTRL_QH_H
#define WBC_M3_CTRL_QH_H

#define WBC_M3_CTRL_M2S_PORT "9876"
#define WBC_M3_CTRL_S2M_PORT "6789"

namespace wbc_m3_ctrl {
  
  struct m2s_data {
    double eepos_x, eepos_y, eepos_z;
  };
  
  struct s2m_data {
    double eepos_x, eepos_y, eepos_z;
  };
  
}

#endif // WBC_M3_CTRL_QH_H
