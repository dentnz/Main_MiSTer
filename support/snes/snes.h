#ifndef SNES_H
#define SNES_H

uint8_t* snes_get_header(fileTYPE *f);
void snes_msu_init(const char* name);
char* snes_read_track_out(void);
void snes_poll(void);

#endif
