
#include "ruben_sdk_lib/IO/FileIO.h"

#ifdef _WIN32
#include <direct.h>
#include <io.h>
#define MKDIR(path) _mkdir(path)
#else
#include <sys/stat.h>
#include <unistd.h>
#define MKDIR(path) mkdir(path, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)
#endif

bool MakeDirectories(const std::string directories) {
    return (0 == MKDIR(directories.c_str()));
}