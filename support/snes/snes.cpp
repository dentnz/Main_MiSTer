
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <limits.h>

#include "../../file_io.h"
#include "../../user_io.h"
#include "../../menu.h"
#include "ringbuffer.h"

static uint8_t hdr[512];
char snes_msu_currenttrack(0x00);

enum HeaderField {
	CartName = 0x00,
	Mapper = 0x15,
	RomType = 0x16,
	RomSize = 0x17,
	RamSize = 0x18,
	CartRegion = 0x19,
	Company = 0x1a,
	Version = 0x1b,
	Complement = 0x1c,  //inverse checksum
	Checksum = 0x1e,
	ResetVector = 0x3c,
};

static uint32_t score_header(const uint8_t *data, uint32_t size, uint32_t addr)
{
	if (size < addr + 64) return 0;  //image too small to contain header at this location?
	int score = 0;

	uint16_t resetvector = data[addr + ResetVector] | (data[addr + ResetVector + 1] << 8);
	uint16_t checksum = data[addr + Checksum] | (data[addr + Checksum + 1] << 8);
	uint16_t complement = data[addr + Complement] | (data[addr + Complement + 1] << 8);

	uint8_t resetop = data[(addr & ~0x7fff) | (resetvector & 0x7fff)];  //first opcode executed upon reset
	uint8_t mapper = data[addr + Mapper] & ~0x10;                      //mask off irrelevent FastROM-capable bit

																	   //$00:[0000-7fff] contains uninitialized RAM and MMIO.
																	   //reset vector must point to ROM at $00:[8000-ffff] to be considered valid.
	if (resetvector < 0x8000) return 0;

	//some images duplicate the header in multiple locations, and others have completely
	//invalid header information that cannot be relied upon.
	//below code will analyze the first opcode executed at the specified reset vector to
	//determine the probability that this is the correct header.

	//most likely opcodes
	if (resetop == 0x78  //sei
		|| resetop == 0x18  //clc (clc; xce)
		|| resetop == 0x38  //sec (sec; xce)
		|| resetop == 0x9c  //stz $nnnn (stz $4200)
		|| resetop == 0x4c  //jmp $nnnn
		|| resetop == 0x5c  //jml $nnnnnn
		) score += 8;

	//plausible opcodes
	if (resetop == 0xc2  //rep #$nn
		|| resetop == 0xe2  //sep #$nn
		|| resetop == 0xad  //lda $nnnn
		|| resetop == 0xae  //ldx $nnnn
		|| resetop == 0xac  //ldy $nnnn
		|| resetop == 0xaf  //lda $nnnnnn
		|| resetop == 0xa9  //lda #$nn
		|| resetop == 0xa2  //ldx #$nn
		|| resetop == 0xa0  //ldy #$nn
		|| resetop == 0x20  //jsr $nnnn
		|| resetop == 0x22  //jsl $nnnnnn
		) score += 4;

	//implausible opcodes
	if (resetop == 0x40  //rti
		|| resetop == 0x60  //rts
		|| resetop == 0x6b  //rtl
		|| resetop == 0xcd  //cmp $nnnn
		|| resetop == 0xec  //cpx $nnnn
		|| resetop == 0xcc  //cpy $nnnn
		) score -= 4;

	//least likely opcodes
	if (resetop == 0x00  //brk #$nn
		|| resetop == 0x02  //cop #$nn
		|| resetop == 0xdb  //stp
		|| resetop == 0x42  //wdm
		|| resetop == 0xff  //sbc $nnnnnn,x
		) score -= 8;

	//at times, both the header and reset vector's first opcode will match ...
	//fallback and rely on info validity in these cases to determine more likely header.

	//a valid checksum is the biggest indicator of a valid header.
	if ((checksum + complement) == 0xffff && (checksum != 0) && (complement != 0)) score += 4;

	if (addr == 0x007fc0 && mapper == 0x20) score += 2;  //0x20 is usually LoROM
	if (addr == 0x00ffc0 && mapper == 0x21) score += 2;  //0x21 is usually HiROM
	if (addr == 0x007fc0 && mapper == 0x22) score += 2;  //0x22 is usually SDD1
	if (addr == 0x40ffc0 && mapper == 0x25) score += 2;  //0x25 is usually ExHiROM

	if (data[addr + Company] == 0x33) score += 2;        //0x33 indicates extended header
	if (data[addr + RomType] < 0x08) score++;
	if (data[addr + RomSize] < 0x10) score++;
	if (data[addr + RamSize] < 0x08) score++;
	if (data[addr + CartRegion] < 14) score++;

	if (score < 0) score = 0;
	return score;
}

static uint32_t find_header(const uint8_t *data, uint32_t size)
{
	uint32_t score_lo = score_header(data, size, 0x007fc0);
	uint32_t score_hi = score_header(data, size, 0x00ffc0);
	uint32_t score_ex = score_header(data, size, 0x40ffc0);
	if (score_ex) score_ex += 4;  //favor ExHiROM on images > 32mbits

	if (score_lo >= score_hi && score_lo >= score_ex)
	{
		return score_lo ? 0x007fc0 : 0;
	}
	else if (score_hi >= score_ex)
	{
		return score_hi ? 0x00ffc0 : 0;
	}

	return score_ex ? 0x40ffc0 : 0;
}

uint8_t* snes_get_header(fileTYPE *f)
{
	memset(hdr, 0, sizeof(hdr));
	uint32_t size = f->size;
	uint8_t *prebuf = (uint8_t*)malloc(size);
	if (prebuf)
	{
		FileSeekLBA(f, 0);
		if (FileReadAdv(f, prebuf, size))
		{
			uint8_t *buf = prebuf;

			if (size & 512)
			{
				buf += 512;
				size -= 512;
			}

			*(uint32_t*)(&hdr[8]) = size;

			uint32_t addr = find_header(buf, size);
			if (addr)
			{
				uint8_t ramsz = buf[addr + RamSize];
				if (ramsz >= 0x08) ramsz = 0;

				//re-calc rom size
				uint8_t romsz = 15;
				size--;
				if (!(size & 0xFF000000))
				{
					while (!(size & 0x1000000))
					{
						romsz--;
						size <<= 1;
					}
				}

				//Rom type: 0-Low, 1-High, 2-ExHigh
				hdr[1] = (addr == 0x00ffc0) ? 1 : (addr == 0x40ffc0) ? 2 : 0;

				//DSPn types 8..B
				if ((buf[addr + Mapper] == 0x20 || buf[addr + Mapper] == 0x21) && buf[addr + RomType] == 0x03)
				{	//DSP1
					hdr[1] |= 0x80;
				}
				else if (buf[addr + Mapper] == 0x30 && buf[addr + RomType] == 0x05 && buf[addr + Company] != 0xb2)
				{	//DSP1
					hdr[1] |= 0x80;
				}
				else if (buf[addr + Mapper] == 0x31 && (buf[addr + RomType] == 0x03 || buf[addr + RomType] == 0x05))
				{	//DSP1
					hdr[1] |= 0x80;
				}
				else if (buf[addr + Mapper] == 0x20 && buf[addr + RomType] == 0x05)
				{	//DSP2
					hdr[1] |= 0x90;
				}
				else if (buf[addr + Mapper] == 0x30 && buf[addr + RomType] == 0x05 && buf[addr + Company] == 0xb2)
				{	//DSP3
					hdr[1] |= 0xA0;
				}
				else if (buf[addr + Mapper] == 0x30 && buf[addr + RomType] == 0x03)
				{	//DSP4
					hdr[1] |= 0xB0;
				}
				else if (buf[addr + Mapper] == 0x30 && buf[addr + RomType] == 0xf6)
				{	//ST010
					hdr[1] |= 0x88;
					ramsz = 1;
					if(buf[addr + RomSize] < 10) hdr[1] |= 0x20; // ST011
				}
				else if (buf[addr + Mapper] == 0x30 && buf[addr + RomType] == 0x25)
				{	//OBC1
					hdr[1] |= 0xC0;
				}

				if (buf[addr + Mapper] == 0x3a && (buf[addr + RomType] == 0xf5 || buf[addr + RomType] == 0xf9)) {
					//SPC7110
					hdr[1] |= 0xD0;
					if(buf[addr + RomType] == 0xf9) hdr[1] |= 0x08; // with RTC
				}

				if (buf[addr + Mapper] == 0x35 && buf[addr + RomType] == 0x55)
				{
					//S-RTC (+ExHigh)
					hdr[1] |= 0x08;
				}

				//CX4 4
				if (buf[addr + Mapper] == 0x20 && buf[addr + RomType] == 0xf3)
				{
					hdr[1] |= 0x40;
				}

				//SDD1 5
				if (buf[addr + Mapper] == 0x32 && (buf[addr + RomType] == 0x43 || buf[addr + RomType] == 0x45))
				{
					if (romsz < 14) hdr[1] |= 0x50; // except Star Ocean un-SDD1
				}

				//SA1 6
				if (buf[addr + Mapper] == 0x23 && (buf[addr + RomType] == 0x32 || buf[addr + RomType] == 0x34 || buf[addr + RomType] == 0x35))
				{
					hdr[1] |= 0x60;
				}

				//GSU 7
				if (buf[addr + Mapper] == 0x20 && (buf[addr + RomType] == 0x13 || buf[addr + RomType] == 0x14 || buf[addr + RomType] == 0x15 || buf[addr + RomType] == 0x1a))
				{
					ramsz = buf[addr - 3];
					if (ramsz == 0xFF) ramsz = 5; //StarFox
					if (ramsz > 6) ramsz = 6;
					hdr[1] |= 0x70;
				}

				//1..3,E..F - reserved for other mappers.

				hdr[2] = 0;

				//PAL Regions
				if ((buf[addr + CartRegion] >= 0x02 && buf[addr + CartRegion] <= 0x0C) || buf[addr + CartRegion] == 0x11)
				{
					hdr[3] |= 1;
				}

				hdr[0] = (ramsz << 4) | romsz;
				printf("Size from header: 0x%X, calculated size: 0x%X\n", buf[addr + RomSize], romsz);
			}
			*(uint32_t*)(&hdr[4]) = addr;
		}
		FileSeekLBA(f, 0);
		free(prebuf);
	}
	return hdr;
}

// This gets set by snes_msu_init for use by MSU-1 support later
char snes_romFileName[1024] = { 0 };

//uint8_t msu_data_array[0x2000000];
uint8_t msu_data_loaded = 0x00;

void snes_msu_init(const char* name)
{
	fileTYPE f = {};
	static char msuFileName[1024] = { 0 };
	// Clear our our rom file name
	memset(snes_romFileName, 0, 1024);
	strncpy(snes_romFileName, name, strlen(name) - 4);
	
	printf("SNES MSU - Rom named '%s' initialised\n", name);
	snes_msu_currenttrack = 0x00;
	
	sprintf(msuFileName, "%s.msu", snes_romFileName);
	printf("SNES MSU - Checking for MSU datafile: %s\n", msuFileName);
	if (!FileOpen(&f, msuFileName)) {
		printf("SNES MSU - MSU datafile not found");
		return;
	}
	else user_io_file_mount(msuFileName, 2);
	msu_data_loaded = 0x01;
}

// Tell the FPGA that the dataseek is finished on the HPS side
void snes_write_dataseek_finished(void)
{
	spi_uio_cmd_cont(0x54);
	spi8(1);
	DisableIO();
	printf("SNES MSU - MSU dataseek finished\n");
}

// Get the FPGA's msu_trackout number
char snes_read_msu_trackout(void)
{
	char msu_trackout;
	spi_uio_cmd_cont(0x50);
	msu_trackout = spi_in();
	DisableIO();
	
	return(msu_trackout);
}

char snes_msu_read_dataseek(void)
{
	char msu_data_seek;
	spi_uio_cmd_cont(0x53);
	msu_data_seek = spi_in();
	DisableIO();

	return(msu_data_seek);	
}

ringbuffer<uint8_t> buf(1048576 * 8);
uint8_t msu_data_array[0x800000];
uint8_t topup_buffer = 0;

// void snes_msu_do_dataseek()
// {
// 	printf("SNES MSU - Seeking");

// 	if (msu_data_loaded == 1) {
// 		memset(msu_data_array, 0, 1048576 * 8);
// 		buf.clear();
// 		printf("SNES MSU - Loading 8mb of MSU datafile into a temp array...\n");
// 		msu_data_loaded = 0;
// 		FileSeekLBA(&sd_image[2], 0);
// 		//FileReadAdv(&sd_image[2], msu_data_array, 0x2000000);
// 		// load 2mb into ram, and then copy 1mb of that into the ring buffer
// 		FileReadAdv(&sd_image[2], msu_data_array, 0x800000);
// 		printf("SNES MSU - Putting 8mb of that temp array into the ringbuffer\n");
// 		buf.write(msu_data_array, 1048576 * 8);
// 		topup_buffer = 1;

// 		snes_write_dataseek_finished();
// 	}
// }

void snes_sd_handling(uint64_t *buffer_lba, fileTYPE *sd_image, int fio_size)
{
	static uint8_t buffer[4][512];
	static uint8_t buffer_big[4096];

	uint32_t lba;
	uint16_t c = user_io_sd_get_status(&lba, 0);
	__off64_t size = sd_image[1].size>>9;
	
	// TODO put this out to a separate topup function
	if (topup_buffer == 0x01 && buf.getFree() <= (uint32_t)1048576 * 2 && buf.getFree() >= (uint32_t)1048576 && sd_image[2].size) {
		printf("SNES MSU - Topping up the ringbuffer...\n");
		uint8_t tempBuffer[0x100000];
		FileReadAdv(&sd_image[2], tempBuffer, 0x100000);
		buf.write(tempBuffer, 1048576);
	}

	char msu_data_seek;
	uint16_t msu_data_seek_addr_high;
	uint16_t msu_data_seek_addr_low;
	spi_uio_cmd_cont(0x53);
	msu_data_seek = spi_in();
	msu_data_seek_addr_low = spi_in();
	msu_data_seek_addr_high = spi_in();
	DisableIO();

	if (msu_data_seek)
	{
		// TODO put this out to a separate seek function
		//snes_msu_do_dataseek();
		//printf("SNES MSU - Seeking - msu_data_loaded: 0x%X\n", msu_data_loaded);
		uint32_t offset = msu_data_seek_addr_high << 16 | msu_data_seek_addr_low;
		
		//if (msu_data_loaded == 0x01) {
			//memset(msu_data_array, 0, 1048576 * 8);
			buf.clear();
			printf("SNES MSU - Loading 8mb of MSU datafile into a temp array...\n");
			msu_data_loaded = 0x00;
			printf("SNES MSU - Seeking to address: %lu\n", (unsigned long)offset);
			printf("SNES MSU - address high: %hx\n", msu_data_seek_addr_high);
			printf("SNES MSU - address low: %hx\n", msu_data_seek_addr_low);
			FileSeek(&sd_image[2], offset, SEEK_SET);
			FileReadAdv(&sd_image[2], msu_data_array, 0x800000);
			printf("SNES MSU - Putting 8mb of that temp array into the ringbuffer\n");
			buf.write(msu_data_array, 1048576 * 8);
			topup_buffer = 0x01;

			snes_write_dataseek_finished();
		//}
	}

	// valid sd commands start with "5x" to avoid problems with
	// cores that don't implement this command
	if ((c & 0xf0) == 0x50)
	{
		// check if core requests configuration
		if (c & 0x08)
		{
			printf("core requests SD config\n");
			user_io_sd_set_config();
		}

		if(c & 0x3802)
		{
			int disk = 3;
			if (c & 0x0002) disk = 0;
			else if (c & 0x0800) disk = 1;
			else if (c & 0x1000) disk = 2;

			// only write if the inserted card is not sdhc or
			// if the core uses sdhc
			if(c & 0x04)
			{
				//printf("SD WR %d on %d\n", lba, disk);
				int done = 0;
				buffer_lba[disk] = lba;

				// Fetch sector data from FPGA ...
				spi_uio_cmd_cont(UIO_SECTOR_WR);
				spi_block_read(buffer[disk], fio_size);
				DisableIO();

				if (sd_image[disk].type == 2 && !lba)
				{
					//Create the file
					if (FileOpenEx(&sd_image[disk], sd_image[disk].path, O_CREAT | O_RDWR | O_SYNC))
					{
						diskled_on();
						if (FileWriteSec(&sd_image[disk], buffer[disk]))
						{
							sd_image[disk].size = 512;
							done = 1;
						}
					}
					else
					{
						printf("Error in creating file: %s\n", sd_image[disk].path);
					}
				}
				else
				{
					// ... and write it to disk
					__off64_t size = sd_image[disk].size>>9;
					if (size && size>=lba)
					{
						if (FileSeekLBA(&sd_image[disk], lba))
						{
							if (FileWriteSec(&sd_image[disk], buffer[disk]))
							{
								done = 1;
								if (size == lba)
								{
									size++;
									sd_image[disk].size = size << 9;
								}
							}
						}
					}
				}

				if (!done) buffer_lba[disk] = -1;
			}
		}
		else if (c & 0x0701)
		{
			// Reads
			int disk = 3;
			if (c & 0x0001) disk = 0;
			else if (c & 0x0100) disk = 1;
			else if (c & 0x0200) disk = 2;

			int done = 0;
			
			if (buffer_lba[disk] != lba)
			{
				if (sd_image[disk].size)
				{
					// if (disk==2 && (lba*512)<0x2000000 ) {	// MSU Data track reading...
					// 	memcpy(buffer[2], msu_data_array+(lba*512), 512);
					// 	done = 1;
					// }
					if (disk==2) {
						// Read
						buf.read(buffer_big, 4096);
						done = 1;
					} 
					else {
						// Other track reading (usually MSU audio track streaming)...
						if (FileSeekLBA(&sd_image[disk], lba))
						{
							if (FileReadSec(&sd_image[disk], buffer[disk]))
							{
								done = 1;
							}
						}
					}
				}

				// Even after error we have to provide the block to the core
				// Give an empty block.
				if (!done) memset(buffer[disk], 0, sizeof(buffer[disk]));
				buffer_lba[disk] = lba;
			}

			if(buffer_lba[disk] == lba)
			{
				// data is now stored in buffer. send it to fpga
				spi_uio_cmd_cont(UIO_SECTOR_RD);
				if (disk == 2) {
					// Send a big sector for MSU data file
					spi_write(buffer_big, 4096, 1);
				} else {
					spi_block_write(buffer[disk], fio_size);
				}
				DisableIO();
			}

			// just load the next sector now, so it may be prefetched
			// for the next request already
			done = 0;
			if (sd_image[disk].size)
			{
				diskled_on();
				if (disk == 1 && lba + 1 == size - 2) 
				{
					printf("SNES MSU - Track reaching end of file\n");
				} 
				
				if (disk != 2 && FileSeekLBA(&sd_image[disk], lba + 1))
				{					
					if (FileReadSec(&sd_image[disk], buffer[disk]))
					{
						done = 1;
					}
				}
			}

			if(done) buffer_lba[disk] = lba + 1;	

			if (sd_image[disk].type == 2)
			{
				buffer_lba[disk] = -1;
			}
		}
	}
}

void snes_poll(void)
{
	fileTYPE f = {};
    static char SelectedPath[1024] = { 0 };
	static char msuErrorMessage[256] = { 0 };
    
    char msu_trackout;
	char msu_trackrequest;
	
	// Tell FPGA to send me msu_trackout and msu_trackrequest
	spi_uio_cmd_cont(0x50);
	msu_trackout = spi_in();
	msu_trackrequest = spi_in();
	// finish up
	DisableIO();

	// New MSU1 Track?
	if (msu_trackrequest == 1) 
	{
        printf("SNES MSU - New track selected: 0x%X\n", msu_trackout);
        snes_msu_currenttrack = msu_trackout;
        
        sprintf(SelectedPath, "%s-%d.pcm", snes_romFileName, msu_trackout);
        printf("SNES MSU - Full MSU track path is: %s\n", SelectedPath);

		if (strlen(snes_romFileName) == 0) {
			sprintf(msuErrorMessage, "MSU1 - No romname\nReload the rom or core");
			Info(msuErrorMessage, 5000);
			return;
		}

		if (!FileOpen(&f, SelectedPath)) {
			// Tell FPGA we couldn't mount the file correctly (trackmissing status will go high, audio busy will go low)
			spi_uio_cmd_cont(0x4f);
        	spi8(1);
        	DisableIO();
			sprintf(msuErrorMessage, "SNES MSU - Track not found: %d", msu_trackout);
			Info(msuErrorMessage, 3000);
			return;
		}

		// Track wasn't missing! Let's make it available to the FPGA
        // Tell FPGA we are mounting the file
        spi_uio_cmd_cont(0x52);
        spi8(1);
        DisableIO();

        user_io_file_mount(SelectedPath, 1);
		// Tell FPGA we have finished mounting the file, trackmissing will go low also
        spi_uio_cmd_cont(0x51);
        spi8(1);
        DisableIO();
    }
}
