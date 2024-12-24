#include "GPU.h"
#include <cstring>
#include <iostream>

namespace PSX {
    GPU::GPU() {
        Reset();
    }

    GPU::~GPU() {
    }

    void GPU::Reset() {
        vram.fill(0);
        status = 0x14802000;
        cmd_buffer.fill(0);
        cmd_buffer_index = 0;
        cmd_buffer_size = 0;

        drawing_area_left = 0;
        drawing_area_top = 0;
        drawing_area_right = 0;
        drawing_area_bottom = 0;

        display_area_x = 0;
        display_area_y = 0;
        display_area_width = 640;
        display_area_height = 480;
    }
    uint32_t GPU::ReadGPUREAD() const {
        // TODO: Implement VRAM read functionality
        return 0;
    }

    void GPU::WriteGP0(uint32_t data) {
        if (cmd_buffer_size == 0) {
            // New command
            uint8_t opcode = (data >> 24) & 0xFF;
            switch (opcode) {
                case 0x00:  // NOP
                    break;
                    
                case 0x01:  // Clear Cache
                    break;
                    
                case 0x02:  // Fill Rectangle
                    cmd_buffer_size = 3;
                    break;
                    
                case 0x28:  // Monochrome 4-point polygon
                    cmd_buffer_size = 5;
                    break;
                    
                case 0x2C:  // Textured 4-point polygon
                    cmd_buffer_size = 9;
                    break;
                    
                case 0x30:  // Shaded 3-point polygon
                    cmd_buffer_size = 6;
                    break;
                    
                case 0x38:  // Shaded 4-point polygon
                    cmd_buffer_size = 8;
                    break;
                    
                case 0xA0:  // Image Load
                    cmd_buffer_size = 3;
                    break;
                    
                case 0xC0:  // Image Store
                    cmd_buffer_size = 3;
                    break;
                    
                case 0xE1:  // Draw Mode setting
                    status = (status & ~0x7FF) | (data & 0x7FF);
                    break;
                    
                case 0xE2:  // Texture Window setting
                    break;
                    
                case 0xE3:  // Drawing Area top left
                    drawing_area_left = data & 0x3FF;
                    drawing_area_top = (data >> 10) & 0x3FF;
                    break;
                    
                case 0xE4:  // Drawing Area bottom right
                    drawing_area_right = data & 0x3FF;
                    drawing_area_bottom = (data >> 10) & 0x3FF;
                    break;
                    
                case 0xE5:  // Drawing Offset
                    break;
                    
                case 0xE6:  // Mask Bit setting
                    status = (status & ~0x1000000) | ((data & 1) << 24);
                    break;
                    
                default:
                    std::cerr << "Unhandled GP0 command: 0x" << std::hex << (int)opcode << std::endl;
                    break;
            }
        }

        if (cmd_buffer_size > 0) {
            cmd_buffer[cmd_buffer_index++] = data;
            if (cmd_buffer_index >= cmd_buffer_size) {
                ExecuteGP0Command();
                cmd_buffer_index = 0;
                cmd_buffer_size = 0;
            }
        }
    }

    void GPU::WriteGP1(uint32_t command) {
        uint8_t opcode = (command >> 24) & 0xFF;
        
        switch (opcode) {
            case 0x00:  // Reset GPU
                Reset();
                break;
                
            case 0x01:  // Reset Command Buffer
                cmd_buffer_index = 0;
                cmd_buffer_size = 0;
                break;
                
            case 0x02:  // Acknowledge IRQ
                status &= ~0x80000000;
                break;
                
            case 0x03:  // Display Enable
                status = (status & ~0x00800000) | ((command & 1) << 23);
                break;
                
            case 0x04:  // DMA Direction
                status = (status & ~0x60000000) | ((command & 3) << 29);
                break;
                
            case 0x05:  // Display Area Start
                display_area_x = command & 0x3FF;
                display_area_y = (command >> 10) & 0x1FF;
                break;
                
            case 0x06:  // Horizontal Display Range
                break;
                
            case 0x07:  // Vertical Display Range
                break;
                
            case 0x08:  // Display Mode
                status = (status & ~0x7F4000) | ((command & 0x3F) << 17) | ((command & 0x40) << 10);
                break;
                
            default:
                std::cerr << "Unhandled GP1 command: 0x" << std::hex << (int)opcode << std::endl;
                break;
        }
    }

    void GPU::ExecuteGP0Command() {
        uint8_t opcode = (cmd_buffer[0] >> 24) & 0xFF;
        switch (opcode) {
            case 0x20:  // Monochrome triangle
            case 0x22:  // Monochrome triangle with transparency
            case 0x24:  // Textured triangle
            case 0x25:  // Textured triangle with transparency
            case 0x26:  // Textured triangle with transparency + raw texture
            case 0x28:  // Monochrome quad
            case 0x2A:  // Monochrome quad with transparency
            case 0x2C:  // Textured quad
            case 0x2D:  // Textured quad with transparency
            case 0x2E:  // Textured quad with transparency + raw texture
                {
                    Vertex v0, v1, v2;
                    bool textured = (opcode & 0x04) != 0;
                    
                    // Extract vertices from command buffer
                    v0.x = cmd_buffer[1] & 0xFFFF;
                    v0.y = (cmd_buffer[1] >> 16) & 0xFFFF;
                    v0.color.r = cmd_buffer[0] & 0xFF;
                    v0.color.g = (cmd_buffer[0] >> 8) & 0xFF;
                    v0.color.b = (cmd_buffer[0] >> 16) & 0xFF;
                    
                    if (textured) {
                        v0.u = cmd_buffer[2] & 0xFF;
                        v0.v = (cmd_buffer[2] >> 8) & 0xFF;
                    }
                    
                    // Similar for v1 and v2...
                    DrawTriangle(v0, v1, v2, textured);
                }
                break;
                
            case 0xA0:  // Image load
                // Implement VRAM image load
                break;
                
            case 0xC0:  // Image store
                // Implement VRAM image store
                break;
        }
    }

    uint16_t GPU::ReadVRAM(uint16_t x, uint16_t y) const {
        if (x >= VRAM_WIDTH || y >= VRAM_HEIGHT) return 0;
        return vram[y * VRAM_WIDTH + x];
    }

    void GPU::WriteVRAM(uint16_t x, uint16_t y, uint16_t color) {
        if (x >= VRAM_WIDTH || y >= VRAM_HEIGHT) return;
        vram[y * VRAM_WIDTH + x] = color;
    }

    void GPU::DrawPixel(uint16_t x, uint16_t y, uint16_t color) {
        if (x < drawing_area_left || x > drawing_area_right ||
            y < drawing_area_top || y > drawing_area_bottom) return;
        WriteVRAM(x, y, color);
    }

    void GPU::DMAIn(uint32_t* data, uint32_t count) {
        for (uint32_t i = 0; i < count; i++) {
            WriteGP0(data[i]);
        }
    }

    void GPU::DMAOut(uint32_t* data, uint32_t count) {
        // Check if VRAM to CPU transfer is active
        if ((status & 0x60000000) == 0x20000000) {
            // Copy VRAM data to CPU memory
            for (uint32_t i = 0; i < count; i++) {
                uint16_t x = (transfer_x + (i * 2) % transfer_width) % VRAM_WIDTH;
                uint16_t y = (transfer_y + (i * 2) / transfer_width) % VRAM_HEIGHT;
                
                // Read two pixels (32-bit word)
                uint16_t pixel1 = ReadVRAM(x, y);
                uint16_t pixel2 = ReadVRAM(x + 1, y);
                
                // Pack into 32-bit word
                data[i] = (pixel2 << 16) | pixel1;
            }
        }
    }

    void GPU::DrawLine(const Vertex& v0, const Vertex& v1) {
        int x0 = v0.x, y0 = v0.y;
        int x1 = v1.x, y1 = v1.y;
        
        bool steep = std::abs(y1 - y0) > std::abs(x1 - x0);
        if (steep) {
            std::swap(x0, y0);
            std::swap(x1, y1);
        }
        
        if (x0 > x1) {
            std::swap(x0, x1);
            std::swap(y0, y1);
        }
        
        int dx = x1 - x0;
        int dy = std::abs(y1 - y0);
        int error = dx / 2;
        int ystep = (y0 < y1) ? 1 : -1;
        int y = y0;
        
        for (int x = x0; x <= x1; x++) {
            float t = (float)(x - x0) / dx;
            Color color = InterpolateColor(v0.color, v1.color, t);
            uint16_t pixel = ((color.r >> 3) << 11) | 
                            ((color.g >> 3) << 6) |
                            ((color.b >> 3) << 1);
                        
            if (steep) {
                DrawPixel(y, x, pixel);
            } else {
                DrawPixel(x, y, pixel);
            }
            
            error -= dy;
            if (error < 0) {
                y += ystep;
                error += dx;
            }
        }
    }

    void GPU::DrawTriangle(const Vertex& v0, const Vertex& v1, const Vertex& v2, bool textured) {
        if (textured) {
            DrawTexturedTriangle(v0, v1, v2);
        } else {
            DrawFlatTriangle(v0, v1, v2);
        }
    }

    void GPU::DrawFlatTriangle(const Vertex& v0, const Vertex& v1, const Vertex& v2) {
        // Sort vertices by y-coordinate
        Vertex vertices[3] = {v0, v1, v2};
        if (vertices[0].y > vertices[1].y) std::swap(vertices[0], vertices[1]);
        if (vertices[1].y > vertices[2].y) std::swap(vertices[1], vertices[2]);
        if (vertices[0].y > vertices[1].y) std::swap(vertices[0], vertices[1]);

        // Calculate slopes
        float slope1 = (float)(vertices[1].x - vertices[0].x) / (vertices[1].y - vertices[0].y);
        float slope2 = (float)(vertices[2].x - vertices[0].x) / (vertices[2].y - vertices[0].y);

        // Draw upper triangle
        float x1 = vertices[0].x;
        float x2 = vertices[0].x;

        for (int y = vertices[0].y; y <= vertices[1].y; y++) {
            int start_x = std::min((int)x1, (int)x2);
            int end_x = std::max((int)x1, (int)x2);

            for (int x = start_x; x <= end_x; x++) {
                float t = (float)(x - start_x) / (end_x - start_x);
                Color color = InterpolateColor(vertices[0].color, vertices[1].color, t);
                uint16_t pixel = ((color.r >> 3) << 11) | 
                                ((color.g >> 3) << 6) |
                                ((color.b >> 3) << 1);
                DrawPixel(x, y, pixel);
            }

            x1 += slope1;
            x2 += slope2;
        }

        // Calculate slopes for lower triangle
        slope1 = (float)(vertices[2].x - vertices[1].x) / (vertices[2].y - vertices[1].y);
        x1 = vertices[1].x;

        // Draw lower triangle
        for (int y = vertices[1].y; y <= vertices[2].y; y++) {
            int start_x = std::min((int)x1, (int)x2);
            int end_x = std::max((int)x1, (int)x2);

            for (int x = start_x; x <= end_x; x++) {
                float t = (float)(x - start_x) / (end_x - start_x);
                Color color = InterpolateColor(vertices[1].color, vertices[2].color, t);
                uint16_t pixel = ((color.r >> 3) << 11) | 
                                ((color.g >> 3) << 6) |
                                ((color.b >> 3) << 1);
                DrawPixel(x, y, pixel);
            }

            x1 += slope1;
            x2 += slope2;
        }
    }

    PSX::GPU::Color GPU::InterpolateColor(const Color& c1, const Color& c2, float t) {
        return Color{
            (uint8_t)(c1.r + t * (c2.r - c1.r)),
            (uint8_t)(c1.g + t * (c2.g - c1.g)),
            (uint8_t)(c1.b + t * (c2.b - c1.b))
        };
    }

    void GPU::DrawTexturedTriangle(const Vertex& v0, const Vertex& v1, const Vertex& v2) {
        // Sort vertices by y-coordinate
        Vertex vertices[3] = {v0, v1, v2};
        if (vertices[0].y > vertices[1].y) std::swap(vertices[0], vertices[1]);
        if (vertices[1].y > vertices[2].y) std::swap(vertices[1], vertices[2]);
        if (vertices[0].y > vertices[1].y) std::swap(vertices[0], vertices[1]);

        // Calculate slopes
        float slope1 = (float)(vertices[1].x - vertices[0].x) / (vertices[1].y - vertices[0].y);
        float slope2 = (float)(vertices[2].x - vertices[0].x) / (vertices[2].y - vertices[0].y);

        // Calculate texture coordinate slopes
        float uslope1 = (float)(vertices[1].u - vertices[0].u) / (vertices[1].y - vertices[0].y);
        float vslope1 = (float)(vertices[1].v - vertices[0].v) / (vertices[1].y - vertices[0].y);
        float uslope2 = (float)(vertices[2].u - vertices[0].u) / (vertices[2].y - vertices[0].y);
        float vslope2 = (float)(vertices[2].v - vertices[0].v) / (vertices[2].y - vertices[0].y);

        // Draw upper triangle
        float x1 = vertices[0].x;
        float x2 = vertices[0].x;
        float u_curr1 = vertices[0].u;
        float v_curr1 = vertices[0].v;
        float u_curr2 = vertices[0].u;
        float v_curr2 = vertices[0].v;

        for (int y = vertices[0].y; y <= vertices[1].y; y++) {
            int start_x = std::min((int)x1, (int)x2);
            int end_x = std::max((int)x1, (int)x2);

            for (int x = start_x; x <= end_x; x++) {
                float t = (float)(x - start_x) / (end_x - start_x);
                float u = u_curr1 + t * (u_curr2 - u_curr1);
                float v = v_curr1 + t * (v_curr2 - v_curr1);
                
                uint16_t texel = ReadVRAM((uint16_t)u, (uint16_t)v);
                Color color = InterpolateColor(vertices[1].color, vertices[2].color, t);
                uint16_t pixel = BlendTextureWithColor(texel, color);
                DrawPixel(x, y, pixel);
            }

            x1 += slope1;
            x2 += slope2;
            u_curr1 += uslope1;
            v_curr1 += vslope1;
            u_curr2 += uslope2;
            v_curr2 += vslope2;
        }

        // Draw lower triangle
        slope1 = (float)(vertices[2].x - vertices[1].x) / (vertices[2].y - vertices[1].y);
        uslope1 = (float)(vertices[2].u - vertices[1].u) / (vertices[2].y - vertices[1].y);
        vslope1 = (float)(vertices[2].v - vertices[1].v) / (vertices[2].y - vertices[1].y);
        x1 = vertices[1].x;
        u_curr1 = vertices[1].u;
        v_curr1 = vertices[1].v;

        for (int y = vertices[1].y; y <= vertices[2].y; y++) {
            int start_x = std::min((int)x1, (int)x2);
            int end_x = std::max((int)x1, (int)x2);

            for (int x = start_x; x <= end_x; x++) {
                float t = (float)(x - start_x) / (end_x - start_x);
                float u = u_curr1 + t * (u_curr2 - u_curr1);
                float v = v_curr1 + t * (v_curr2 - v_curr1);
                
                uint16_t texel = ReadVRAM((uint16_t)u, (uint16_t)v);
                Color color = InterpolateColor(vertices[1].color, vertices[2].color, t);
                uint16_t pixel = BlendTextureWithColor(texel, color);
                DrawPixel(x, y, pixel);
            }

            x1 += slope1;
            x2 += slope2;
            u_curr1 += uslope1;
            v_curr1 += vslope1;
            u_curr2 += uslope2;
            v_curr2 += vslope2;
        }
    }

    uint16_t GPU::BlendTextureWithColor(uint16_t texel, const Color& color) {
        // Extract RGB components from texel
        uint8_t tr = (texel >> 11) & 0x1F;
        uint8_t tg = (texel >> 6) & 0x1F;
        uint8_t tb = (texel >> 1) & 0x1F;
        
        // Blend with vertex color
        uint8_t r = (tr * color.r) >> 8;
        uint8_t g = (tg * color.g) >> 8;
        uint8_t b = (tb * color.b) >> 8;
        
        return ((r & 0x1F) << 11) | ((g & 0x1F) << 6) | ((b & 0x1F) << 1);
    }
}