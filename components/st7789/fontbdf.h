#ifndef WATCH_FONTBDF_H
#define WATCH_FONTBDF_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    const char* filename;
    int* chars;
    uint32_t* index;
    uint8_t* lengths;
} bdf_t;

typedef struct {
    uint8_t width;
    uint8_t height;
    int8_t xOffset;
    int8_t yOffset;
    uint32_t bitmap[64];
    uint8_t chars;
} BDFChar;

int loadFileDynamic(bdf_t * file_data);
int loadBDFChar(const char * filename, uint8_t ascii, BDFChar * outChar);

#endif //WATCH_FONTBDF_H
