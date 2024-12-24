#include <SDL2/SDL.h>
#include <iostream>

#include "CPU/CPU.h"

int main(int argc, char* argv[]) {
    PSX::R3000A_CPU* cpu = new PSX::R3000A_CPU();

    if (!cpu) {
        std::cerr << "Failed to initialize CPU" << std::endl;
        return -1;
    }

    SDL_Init(SDL_INIT_EVERYTHING);

    SDL_Window* window = NULL;
    window = SDL_CreateWindow(
            "Title",
            SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
            640, 480,
            SDL_WINDOW_SHOWN
            );

    SDL_Renderer* renderer = NULL;
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    
    SDL_bool quit = SDL_FALSE;
    while(!quit)
    {
        SDL_Event event;
        while(SDL_PollEvent(&event))
        {
            if(event.type == SDL_QUIT)
                quit = SDL_TRUE;
        }
        SDL_RenderClear(renderer);
        SDL_RenderPresent(renderer);
    }

    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}