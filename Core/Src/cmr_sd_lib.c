#include "fatfs.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "bsp_driver_sd.h"

typedef struct cmr_can_csv_t {
	uint32_t timestamp;
	uint16_t can_id;
	char data[8];
} cmr_can_csv_t;

extern char SDPath[4];
FATFS fs;
BSP_SD_CardInfo myCardInfo;

int sd_get_space_kb(void) {
	FATFS *pfs;
	DWORD fre_clust, tot_sect, fre_sect, total_kb, free_kb;
	FRESULT res = f_getfree(SDPath, &fre_clust, &pfs);
	if (res != FR_OK) return res;

	tot_sect = (pfs->n_fatent - 2) * pfs->csize;
	fre_sect = fre_clust * pfs->csize;
	total_kb = tot_sect / 2;
	free_kb = fre_sect / 2;
	printf("Total: %lu KB, Free: %lu KB\r\n", total_kb, free_kb);
	return FR_OK;
}

int sd_mount(void) {
	FRESULT res;

	printf("Attempting mount at %s...\r\n", SDPath);
	res = f_mount(&fs, SDPath, 1);
	HAL_Delay(100);
	if (res == FR_OK)
	{
		printf("SD card mounted successfully at %s\r\n", SDPath);

		// Capacity and free space reporting
		sd_get_space_kb();

		// Get Card Info
		BSP_SD_GetCardInfo(&myCardInfo);
		printf("Card Type: %s\r\n", myCardInfo.CardType ? "SDSC" : "SDHC/SDXC");
		printf("Card Version: %s\r\n", myCardInfo.CardVersion ? "CARD_V1_X" : "CARD_V2_X");
		printf("Card Class: %lu\r\n", myCardInfo.Class);
		return FR_OK;
	}

	// Any other mount error
	printf("Mount failed with code: %d\r\n", res);
	return res;
}


int sd_unmount(void) {
	FRESULT res = f_mount(NULL, SDPath, 1);
	printf("SD card unmounted: %s\r\n\r\n\r\n", (res == FR_OK) ? "OK" : "Failed");
	return res;
}

int sd_write_file(const char *filename, const char *text) {
	FIL file;
	UINT bw;
	FRESULT res = f_open(&file, filename, FA_CREATE_ALWAYS | FA_WRITE);
	if (res != FR_OK) return res;

	res = f_write(&file, text, strlen(text), &bw);
	f_close(&file);
	printf("Write %u bytes to %s\r\n", bw, filename);
	return (res == FR_OK && bw == strlen(text)) ? FR_OK : FR_DISK_ERR;
}

int sd_append_file(const char *filename, const char *text) {
	FIL file;
	UINT bw;
	FRESULT res = f_open(&file, filename, FA_OPEN_ALWAYS | FA_WRITE);
	if (res != FR_OK) return res;

	res = f_lseek(&file, f_size(&file));
	if (res != FR_OK) {
		f_close(&file);
		return res;
	}

	res = f_write(&file, text, strlen(text), &bw);
	f_close(&file);
	printf("Appended %u bytes to %s\r\n", bw, filename);
	return (res == FR_OK && bw == strlen(text)) ? FR_OK : FR_DISK_ERR;
}

int sd_read_file(const char *filename, char *buffer, UINT bufsize, UINT *bytes_read) {
	FIL file;
	*bytes_read = 0;

	FRESULT res = f_open(&file, filename, FA_READ);
	if (res != FR_OK) {
		printf("f_open failed with code: %d\r\n", res);
		return res;
	}

	res = f_read(&file, buffer, bufsize - 1, bytes_read);
	if (res != FR_OK) {
		printf("f_read failed with code: %d\r\n", res);
		f_close(&file);
		return res;
	}

	buffer[*bytes_read] = '\0';

	res = f_close(&file);
	if (res != FR_OK) {
		printf("f_close failed with code: %d\r\n", res);
		return res;
	}

	printf("Read %u bytes from %s\r\n", *bytes_read, filename);
	return FR_OK;
}

int sd_delete_file(const char *filename) {
	FRESULT res = f_unlink(filename);
	printf("Delete %s: %s\r\n", filename, (res == FR_OK ? "OK" : "Failed"));
	return res;
}

int sd_rename_file(const char *oldname, const char *newname) {
	FRESULT res = f_rename(oldname, newname);
	printf("Rename %s to %s: %s\r\n", oldname, newname, (res == FR_OK ? "OK" : "Failed"));
	return res;
}

FRESULT sd_create_directory(const char *path) {
	FRESULT res = f_mkdir(path);
	printf("Create directory %s: %s\r\n", path, (res == FR_OK ? "OK" : "Failed"));
	return res;
}

void sd_list_directory_recursive(const char *path, int depth) {
	DIR dir;
	FILINFO fno;
//	char lfn[256];
//	fno.fname = lfn;
//	fno.fsize = sizeof(lfn);
	FRESULT res = f_opendir(&dir, path);
	if (res != FR_OK) {
		printf("%*s[ERR] Cannot open: %s\r\n", depth * 2, "", path);
		return;
	}

	while (1) {
		res = f_readdir(&dir, &fno);
		if (res != FR_OK || fno.fname[0] == 0) break;

		const char *name = (*fno.fname) ? fno.fname : fno.fname;

		if (fno.fattrib & AM_DIR) {
			if (strcmp(name, ".") && strcmp(name, "..")) {
				printf("%*sD %s\r\n", depth * 2, "", name);
				char newpath[128];
				snprintf(newpath, sizeof(newpath), "%s/%s", path, name);
				sd_list_directory_recursive(newpath, depth + 1);
			}
		} else {
			printf("%*sF %s (%lu bytes)\r\n", depth * 2, "", name, (unsigned long)fno.fsize);
		}
	}
	f_closedir(&dir);
}

void sd_list_files(void) {
	printf("Files on SD Card:\r\n");
	sd_list_directory_recursive(SDPath, 0);
	printf("\r\n\r\n");
}
