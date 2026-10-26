#pragma once

#if defined(ETHERNET_ENABLED)
  #if defined(ETHERNET_USE_CH390)
    #include "helpers/ethernet/ch390/CH390EthernetInterface.h"
  #elif defined(ETHERNET_USE_RAK13800)
    #include "helpers/ethernet/RAK13800/RAK13800EthernetInterface.h"
  #elif defined(ETHERNET_USE_W5500)
    #include "helpers/ethernet/W5500/W5500EthernetInterface.h"
  #else
    #error "ETHERNET_ENABLED is defined, but no specific driver flag (e.g. ETHERNET_USE_CH390) was provided!"
  #endif
#endif
