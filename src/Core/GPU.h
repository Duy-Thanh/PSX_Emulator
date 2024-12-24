#pragma once

#include <cstdint>
#include <array>

namespace PSX {
    class GPU {
    private:
        // VRAM: 1MB (1024x512x16-bit)
        static constexpr uint32_t VRAM_WIDTH = 1024;
        static constexpr uint32_t VRAM_HEIGHT = 512;
        std::array<uint16_t, VRAM_WIDTH * VRAM_HEIGHT> vram;

        // GPU Status Register (GPUSTAT)
        uint32_t status = 0x14802000;

        // GPU Command Buffer
        std::array<uint32_t, 16> cmd_buffer;
        uint8_t cmd_buffer_index = 0;
        uint8_t cmd_buffer_size = 0;

        // Drawing area
        uint16_t drawing_area_left = 0;
        uint16_t drawing_area_top = 0;
        uint16_t drawing_area_right = 0;
        uint16_t drawing_area_bottom = 0;

        // Display area
        uint16_t display_area_x = 0;
        uint16_t display_area_y = 0;
        uint16_t display_area_width = 640;
        uint16_t display_area_height = 480;

        // VRAM transfer state
        uint16_t transfer_x = 0;
        uint16_t transfer_y = 0;
        uint16_t transfer_width = 0;
        uint16_t transfer_height = 0;

        // GPU busy state
        bool busy;

        // Helper functions
        void ExecuteGP0Command();
        void ExecuteGP1Command(uint32_t command);
        void DrawPixel(uint16_t x, uint16_t y, uint16_t color);

        struct Color {
            uint8_t r, g, b;
            Color() : r(0), g(0), b(0) {}
            Color(uint8_t r, uint8_t g, uint8_t b) : r(r), g(g), b(b) {}
        };

        struct Vertex {
            int16_t x, y;
            Color color;
            uint16_t u, v;
        };

        // Drawing functions
        void DrawLine(const Vertex& v0, const Vertex& v1);
        void DrawTriangle(const Vertex& v0, const Vertex& v1, const Vertex& v2, bool textured = false);
        void DrawFlatTriangle(const Vertex& v0, const Vertex& v1, const Vertex& v2);
        void DrawTexturedTriangle(const Vertex& v0, const Vertex& v1, const Vertex& v2);
        Color InterpolateColor(const Color& c1, const Color& c2, float t);

        // Move BlendTextureWithColor to private section
        uint16_t BlendTextureWithColor(uint16_t texel, const Color& color);

    public:
        GPU();
        ~GPU();

        void Reset();

        // GPU Register access
        uint32_t ReadGPUSTAT() const { return status; }
        uint32_t ReadGPUREAD() const;
        uint32_t ReadGPUDATA();
        void WriteGP0(uint32_t data);
        void WriteGP1(uint32_t command);

        // VRAM access
        uint16_t ReadVRAM(uint16_t x, uint16_t y) const;
        void WriteVRAM(uint16_t x, uint16_t y, uint16_t color);
        
        // DMA access
        void DMAIn(uint32_t* data, uint32_t count);
        void DMAOut(uint32_t* data, uint32_t count);

        bool IsBusy() const { return busy; }
    };
}