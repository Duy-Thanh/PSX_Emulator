#pragma once

#include <cstdint>
#include <array>

namespace PSX {
    class GTE {
    private:
        // GTE Data Registers
        std::array<int32_t, 32> data_regs;  // VXY0, VZ0, VXY1, VZ1, VXY2, VZ2, RGBC, OTZ, IR0-3, SXY0-3, SZ0-3, etc.
        
        // GTE Control Registers
        std::array<int32_t, 32> ctrl_regs;  // Rotation matrix, Translation vector, Light source matrix, etc.

        // Internal state
        uint32_t flag;      // Error flag register
        bool sf;            // Shift factor (12.20 or 12.12)
        bool lm;            // Limit negative values to 0
        
        // Helper functions
        int32_t Limit(int64_t value, int32_t max, int32_t min, uint32_t flag_bit);
        void UpdateFlag(int64_t value, uint32_t max, uint32_t flag_bit);
        
    public:
        GTE();
        ~GTE();

        void Reset();
        
        // GTE Command execution
        uint32_t Execute(uint32_t command);
        
        // Register access
        uint32_t ReadData(uint8_t reg);
        void WriteData(uint8_t reg, uint32_t value);
        uint32_t ReadControl(uint8_t reg);
        void WriteControl(uint8_t reg, uint32_t value);

        // GTE Operations
        void RTPS(uint8_t v);      // Perspective Transformation (single)
        void RTPT();               // Perspective Transformation (triple)
        void MVMVA();              // Multiply Vector by Matrix and Vector Addition
        void NCLIP();              // Normal Clipping
        void OP();                 // Outer Product
        void DPCS();               // Depth Cueing (single)
        void INTPL();              // Interpolation
        void NCDS();               // Normal Color Depth Cue (single)
        void CDP();                // Color Depth Queue
        void NCDT();               // Normal Color Depth Cue (triple)
        void NCCS();               // Normal Color Color (single)
        void CC();                 // Color Color
        void NCS();                // Normal Color (single)
        void NCT();                // Normal Color (triple)
        void SQR();                // Square of Vector
        void DCPL();               // Depth Cue Color Light
        void DPCT();               // Depth Cueing (triple)
        void AVSZ3();              // Average of three Z values
        void AVSZ4();              // Average of four Z values
    };
}