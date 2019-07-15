#ifndef SNES_H
#define SNES_H

#define UIO_SNES_MSU_TRACKOUT    0x61

uint8_t* snes_get_header(fileTYPE *f);
char* snes_read_track_out(void);
void snes_poll(void);

#endif
