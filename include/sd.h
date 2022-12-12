#ifndef SD_H_
#define SD_H_

#include <FS.h>

#include "hardware_defs.h"
#include "packets.h"

#define SAVING_PERIOD 5

// if testing is needed, raising the value above may help the analisys


char file_name[20];
File root;

#endif