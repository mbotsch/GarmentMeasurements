#include <cstdio>

#include "Viewer.h"

int main(int argc, char** argv)
{
    Viewer viewer("Characterfitting", 1080, 720);

    (void)argc;
    (void)argv;

    return viewer.run();
}
