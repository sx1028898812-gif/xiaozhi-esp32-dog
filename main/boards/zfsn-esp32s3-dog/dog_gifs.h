#ifndef _DOG_GIFS_H_
#define _DOG_GIFS_H_

#include <cstddef>
#include <cstdint>

struct DogGifAsset {
    const char* name;
    const uint8_t* data;
    size_t size;
};

const DogGifAsset* GetDogGif(const char* name);

#endif // _DOG_GIFS_H_
