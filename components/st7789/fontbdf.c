#include "fontbdf.h"
#include "esp_log.h"

int loadFileDynamic(bdf_t *fd) {
    FILE *file = fopen(fd->filename, "r");
    if (!file) return -1;


    char line[256];

    int pos = 0;
    int pos_index = 0;
    int encode = -1;

    while (fgets(line, sizeof(line), file)) {

        // save first index of the new character block

        if (strncmp(line, "ENCODING", 8) == 0) {
            sscanf(line, "ENCODING %d", &encode);
        }

        if (strncmp(line, "BBX", 3) == 0) {
            fd->index[pos_index] = pos;
            sscanf(line, "BBX %d %d %d %d", &fd->chars[pos + 0], &fd->chars[pos + 1],  &fd->chars[pos + 2],  &fd->chars[pos + 3]); // width, height
            pos += 4;
        }

        if (strncmp(line, "BITMAP", 6) == 0) {
            int max = fd->chars[pos - 3];
            for (int i = 0; i < max; i++) {
                if (fgets(line, sizeof(line), file)) {
                    if (i == 0)
                        sscanf(line, "%x%n", &fd->chars[pos], (int*)&fd->lengths[pos_index]);
                    else
                        sscanf(line, "%x", &fd->chars[pos]);
                    pos++;
                }
            }
            pos_index++;
        }
    }

    fclose(file);
    return 0;
}

int loadBDFChar(const char *filename, uint8_t ascii, BDFChar *outChar) {
    FILE *file = fopen(filename, "r");
    if (!file) return -1;

    char line[128];
    int foundChar = 0;

    while (fgets(line, sizeof(line), file)) {
        if (strncmp(line, "STARTCHAR", 9) == 0) {
            foundChar = 0;
        }

        // Find character encoding
        if (strncmp(line, "ENCODING", 8) == 0) {
            int encoding;
            sscanf(line, "ENCODING %d", &encoding);
            if (encoding == ascii) {
                foundChar = 1;
            }
        }

        // Parse bounding box for width, height, offsets if this is the correct character
        if (foundChar && strncmp(line, "BBX", 3) == 0) {
            sscanf(line, "BBX %hhu %hhu %hhd %hhd", &outChar->width, &outChar->height, &outChar->xOffset, &outChar->yOffset);
        }

        // Parse bitmap data
        if (foundChar && strncmp(line, "BITMAP", 6) == 0) {
            for (int i = 0; i < outChar->height; i++) {
                if (fgets(line, sizeof(line), file)) {
                    sscanf(line, "%lx%n", &outChar->bitmap[i], (int*)&outChar->chars);
                }
            }
            fclose(file);
            return 0;  // Character found and loaded
        }
    }

    fclose(file);
    return -1;  // Character not found
}


