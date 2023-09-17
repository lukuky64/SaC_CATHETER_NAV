#include <stdint.h>
#include <stdio.h>

struct Image
{
    uint8_t* data  = NULL;
    size_t size = 0; // size of the dat storage
    int w;
    int h;
    int channels; // number of channels (1 for greyscale)

    // constructors
    Image(const char* filename);
    Image(int w, int h , int channels);
    Image(const Image& img);
    ~Image();

    bool read(const char* filename);
    bool write(const char* filename);
};
