#pragma once

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <dirent.h>
#include "esp_err.h"
#include "esp_log.h"
// the fatfs headers use MAX()/MIN() in a preprocessor #if. On some targets
// (e.g. ESP32-P4) sys/param.h has already been included by an earlier
// header and AP_Math.h has #undef'd its MAX/MIN macros in favour of C++
// templates, leaving ffconf.h's #include <sys/param.h> a no-op. Provide
// fallbacks around just the fatfs include so compilation succeeds
// regardless of include order, then restore the previous state so the
// fallbacks cannot shadow AP_Math's C++ MIN/MAX templates.
#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define AP_FATFS_RESTORE_MINMAX
#endif
#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define AP_FATFS_RESTORE_MINMAX
#endif
#include "esp_vfs_fat.h"
#ifdef AP_FATFS_RESTORE_MINMAX
#undef MAX
#undef MIN
#undef AP_FATFS_RESTORE_MINMAX
#endif
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"

#include "AP_Filesystem_backend.h"

class AP_Filesystem_ESP32 : public AP_Filesystem_Backend
{
public:
    // functions that closely match the equivalent posix calls
    int open(const char *fname, int flags, bool allow_absolute_paths = false) override;
    int close(int fd) override;
    int32_t read(int fd, void *buf, uint32_t count) override;
    int32_t write(int fd, const void *buf, uint32_t count) override;
    int fsync(int fd) override;
    int32_t lseek(int fd, int32_t offset, int whence) override;
    int stat(const char *pathname, struct stat *stbuf) override;
    int unlink(const char *pathname) override;
    int mkdir(const char *pathname) override;
    void *opendir(const char *pathname) override;
    struct dirent *readdir(void *dirp) override;
    int closedir(void *dirp) override;
    int rename(const char *oldpath, const char *newpath) override;

    // return free disk space in bytes, -1 on error
    int64_t disk_free(const char *path) override;

    // return total disk space in bytes, -1 on error
    int64_t disk_space(const char *path) override;

    // set modification time on a file
    bool set_mtime(const char *filename, const uint32_t mtime_sec) override;
};

