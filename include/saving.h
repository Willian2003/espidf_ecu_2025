#ifndef SAVING_H_
#define SAVING_H_

#include <FS.h>

#include "hardware_defs.h"
#include "packets.h"

#define BOX_MSG     2
#define COMB_STOP   3
#define FAST        4
#define SLOW        5 

#define SAVING_PERIOD 5

// if testing is needed, raising the value above may help the analisys

char file_name[20];
File root;
File dataFile;

#endif