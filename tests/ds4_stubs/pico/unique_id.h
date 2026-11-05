#pragma once
#include <stdint.h>
#define PICO_UNIQUE_BOARD_ID_SIZE_BYTES 8
struct pico_unique_board_id_t { uint8_t id[PICO_UNIQUE_BOARD_ID_SIZE_BYTES]; };
inline void pico_get_unique_board_id(pico_unique_board_id_t* id) {
  for (unsigned i=0; i<PICO_UNIQUE_BOARD_ID_SIZE_BYTES; ++i) id->id[i]=i+1;
}
