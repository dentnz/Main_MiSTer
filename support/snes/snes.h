#ifndef SNES_H
#define SNES_H

uint8_t* snes_get_header(fileTYPE *f);
void snes_patch_bs_header(fileTYPE *f, uint8_t *buf);
void snes_msu_init(const char* name);
char* snes_read_track_out(void);
void snes_poll(void);
void snes_sd_handling(uint64_t *buffer_lba, fileTYPE *sd_image, int fio_size);

#endif
