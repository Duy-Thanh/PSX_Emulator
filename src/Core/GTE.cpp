#include "GTE.h"
#include <cstring>

namespace PSX {
    GTE::GTE() {
        Reset();
    }

    GTE::~GTE() {
    }

    void GTE::Reset() {
        data_regs.fill(0);
        ctrl_regs.fill(0);
        flag = 0;
        sf = false;
        lm = false;
    }

    uint32_t GTE::Execute(uint32_t command) {
        uint8_t op = (command >> 20) & 0x1F;
        sf = (command >> 19) & 1;
        lm = (command >> 10) & 1;
        
        switch(op) {
            case 0x01: RTPS(0); break;     // Perspective transformation (single)
            case 0x06: NCLIP(); break;     // Normal clipping
            case 0x0C: OP(); break;        // Outer product of 2 vectors
            case 0x10: DPCS(); break;      // Depth Cueing (single)
            case 0x11: INTPL(); break;     // Interpolation
            case 0x12: MVMVA(); break;     // Multiply vector by matrix and vector addition
            case 0x13: NCDS(); break;      // Normal color depth cue (single)
            case 0x14: CDP(); break;       // Color Depth Queue
            case 0x16: NCDT(); break;      // Normal color depth cue (triple)
            case 0x1B: NCCS(); break;      // Normal color color (single)
            case 0x1C: CC(); break;        // Color Color
            case 0x1E: NCS(); break;       // Normal color (single)
            case 0x20: NCT(); break;       // Normal color (triple)
            case 0x28: SQR(); break;       // Square of vector
            case 0x29: DCPL(); break;      // Depth Cue Color Light
            case 0x2A: DPCT(); break;      // Depth Cueing (triple)
            case 0x2D: AVSZ3(); break;     // Average of three Z values
            case 0x2E: AVSZ4(); break;     // Average of four Z values
            case 0x30: RTPT(); break;      // Perspective Transformation (triple)
            default:
                // Unknown GTE command
                return 0;
        }
        
        return 1;
    }

    void GTE::RTPS(uint8_t v) {
        // Get translation vector
        int32_t trx = ctrl_regs[5];
        int32_t try_ = ctrl_regs[6];
        int32_t trz = ctrl_regs[7];

        // Get rotation matrix
        int32_t r11 = ctrl_regs[0];
        int32_t r12 = ctrl_regs[1];
        int32_t r13 = ctrl_regs[2];
        int32_t r21 = ctrl_regs[3];
        int32_t r22 = ctrl_regs[4];
        int32_t r23 = ctrl_regs[5];
        int32_t r31 = ctrl_regs[6];
        int32_t r32 = ctrl_regs[7];
        int32_t r33 = ctrl_regs[8];

        // Get vertex coordinates
        int32_t vx = data_regs[v];
        int32_t vy = data_regs[v + 1];
        int32_t vz = data_regs[v + 2];

        // Perform matrix multiplication
        int64_t mac1 = ((int64_t)r11 * vx + (int64_t)r12 * vy + (int64_t)r13 * vz) >> 12;
        int64_t mac2 = ((int64_t)r21 * vx + (int64_t)r22 * vy + (int64_t)r23 * vz) >> 12;
        int64_t mac3 = ((int64_t)r31 * vx + (int64_t)r32 * vy + (int64_t)r33 * vz) >> 12;

        // Add translation
        mac1 += trx;
        mac2 += try_;
        mac3 += trz;

        // Add overflow checks
        UpdateFlag(mac1, 0x7FFFFFFF, 30);  // MAC1 overflow
        UpdateFlag(mac2, 0x7FFFFFFF, 29);  // MAC2 overflow
        UpdateFlag(mac3, 0x7FFFFFFF, 28);  // MAC3 overflow
        
        // Add divide overflow check
        if (mac3 == 0) {
            flag |= (1 << 17);  // Division by zero
        }

        // Store results
        data_regs[9] = Limit(mac1, 0x7FFFFFFF, -0x80000000, 12);   // IR1
        data_regs[10] = Limit(mac2, 0x7FFFFFFF, -0x80000000, 13);  // IR2
        data_regs[11] = Limit(mac3, 0x7FFFFFFF, -0x80000000, 14);  // IR3
    }

    int32_t GTE::Limit(int64_t value, int32_t max, int32_t min, uint32_t flag_bit) {
        if (value > max) {
            flag |= (1 << flag_bit);
            return max;
        }
        if (value < min) {
            flag |= (1 << flag_bit);
            return min;
        }
        return value;
    }

    void GTE::UpdateFlag(int64_t value, uint32_t max, uint32_t flag_bit) {
        if (value > max || value < -max) {
            flag |= (1 << flag_bit);
        }
    }

    // ... Additional GTE operation implementations ...

    void GTE::NCLIP() {
        // Calculate normal clip by computing cross product
        int32_t x0 = data_regs[12] - data_regs[14]; // SX0 - SX1
        int32_t y0 = data_regs[13] - data_regs[15]; // SY0 - SY1
        int32_t x1 = data_regs[16] - data_regs[14]; // SX2 - SX1
        int32_t y1 = data_regs[17] - data_regs[15]; // SY2 - SY1
        
        int64_t mac0 = (int64_t)x0 * y1 - (int64_t)x1 * y0;
        data_regs[24] = mac0; // MAC0
    }

    void GTE::OP() {
        // Outer product of IR1,IR2,IR3 with IR0
        int64_t mac1 = ((int64_t)data_regs[10] * data_regs[11]) - ((int64_t)data_regs[11] * data_regs[9]);
        int64_t mac2 = ((int64_t)data_regs[11] * data_regs[9]) - ((int64_t)data_regs[9] * data_regs[10]);
        int64_t mac3 = ((int64_t)data_regs[9] * data_regs[10]) - ((int64_t)data_regs[10] * data_regs[11]);
        
        data_regs[9] = Limit(mac1, 0x7FFFFFFF, -0x80000000, 12);
        data_regs[10] = Limit(mac2, 0x7FFFFFFF, -0x80000000, 13);
        data_regs[11] = Limit(mac3, 0x7FFFFFFF, -0x80000000, 14);
    }

    void GTE::DPCS() {
        // Depth Cueing (single)
        int32_t r = data_regs[6] & 0xFF;
        int32_t g = (data_regs[6] >> 8) & 0xFF;
        int32_t b = (data_regs[6] >> 16) & 0xFF;
        
        int64_t mac1 = ((int64_t)r * ctrl_regs[21]) >> 12;
        int64_t mac2 = ((int64_t)g * ctrl_regs[22]) >> 12;
        int64_t mac3 = ((int64_t)b * ctrl_regs[23]) >> 12;
        
        data_regs[9] = Limit(mac1, 0x7FFFFFFF, -0x80000000, 12);
        data_regs[10] = Limit(mac2, 0x7FFFFFFF, -0x80000000, 13);
        data_regs[11] = Limit(mac3, 0x7FFFFFFF, -0x80000000, 14);
    }

    void GTE::INTPL() {
        // Interpolation between IR1 and IR3 using IR2
        int32_t ir1 = data_regs[9];   // IR1
        int32_t ir2 = data_regs[10];  // IR2
        int32_t ir3 = data_regs[11];  // IR3
        
        int64_t mac1 = ((int64_t)ir1 * (4096 - ir2) + (int64_t)ir3 * ir2) >> 12;
        int64_t mac2 = mac1;
        int64_t mac3 = mac1;
        
        data_regs[9] = Limit(mac1, 0x7FFFFFFF, -0x80000000, 12);   // IR1
        data_regs[10] = Limit(mac2, 0x7FFFFFFF, -0x80000000, 13);  // IR2
        data_regs[11] = Limit(mac3, 0x7FFFFFFF, -0x80000000, 14);  // IR3
    }

    void GTE::MVMVA() {
        // Matrix multiplication and vector addition
        int32_t v1 = data_regs[9];    // IR1
        int32_t v2 = data_regs[10];   // IR2
        int32_t v3 = data_regs[11];   // IR3
        
        // Use rotation matrix by default
        int32_t m11 = ctrl_regs[0];
        int32_t m12 = ctrl_regs[1];
        int32_t m13 = ctrl_regs[2];
        int32_t m21 = ctrl_regs[3];
        int32_t m22 = ctrl_regs[4];
        int32_t m23 = ctrl_regs[5];
        int32_t m31 = ctrl_regs[6];
        int32_t m32 = ctrl_regs[7];
        int32_t m33 = ctrl_regs[8];
        
        int64_t mac1 = ((int64_t)m11 * v1 + (int64_t)m12 * v2 + (int64_t)m13 * v3) >> 12;
        int64_t mac2 = ((int64_t)m21 * v1 + (int64_t)m22 * v2 + (int64_t)m23 * v3) >> 12;
        int64_t mac3 = ((int64_t)m31 * v1 + (int64_t)m32 * v2 + (int64_t)m33 * v3) >> 12;
        
        data_regs[9] = Limit(mac1, 0x7FFFFFFF, -0x80000000, 12);
        data_regs[10] = Limit(mac2, 0x7FFFFFFF, -0x80000000, 13);
        data_regs[11] = Limit(mac3, 0x7FFFFFFF, -0x80000000, 14);
    }

    void GTE::NCDS() {
        // Normal Color Depth Cue (single)
        int32_t nx = data_regs[0];    // VX0
        int32_t ny = data_regs[1];    // VY0
        int32_t nz = data_regs[2];    // VZ0
        
        // Light matrix multiplication
        int64_t mac1 = ((int64_t)ctrl_regs[0] * nx + (int64_t)ctrl_regs[1] * ny + (int64_t)ctrl_regs[2] * nz) >> 12;
        int64_t mac2 = ((int64_t)ctrl_regs[3] * nx + (int64_t)ctrl_regs[4] * ny + (int64_t)ctrl_regs[5] * nz) >> 12;
        int64_t mac3 = ((int64_t)ctrl_regs[6] * nx + (int64_t)ctrl_regs[7] * ny + (int64_t)ctrl_regs[8] * nz) >> 12;
        
        // Store intermediate results
        data_regs[9] = Limit(mac1, 0x7FFFFFFF, -0x80000000, 12);   // IR1
        data_regs[10] = Limit(mac2, 0x7FFFFFFF, -0x80000000, 13);  // IR2
        data_regs[11] = Limit(mac3, 0x7FFFFFFF, -0x80000000, 14);  // IR3
    }

    void GTE::CDP() {
        // Color Depth Queue
        int32_t r = data_regs[6] & 0xFF;
        int32_t g = (data_regs[6] >> 8) & 0xFF;
        int32_t b = (data_regs[6] >> 16) & 0xFF;
        
        // Color matrix multiplication
        int64_t mac1 = ((int64_t)ctrl_regs[13] * r + (int64_t)ctrl_regs[14] * g + (int64_t)ctrl_regs[15] * b) >> 12;
        int64_t mac2 = ((int64_t)ctrl_regs[16] * r + (int64_t)ctrl_regs[17] * g + (int64_t)ctrl_regs[18] * b) >> 12;
        int64_t mac3 = ((int64_t)ctrl_regs[19] * r + (int64_t)ctrl_regs[20] * g + (int64_t)ctrl_regs[21] * b) >> 12;
        
        data_regs[9] = Limit(mac1, 0x7FFFFFFF, -0x80000000, 12);   // IR1
        data_regs[10] = Limit(mac2, 0x7FFFFFFF, -0x80000000, 13);  // IR2
        data_regs[11] = Limit(mac3, 0x7FFFFFFF, -0x80000000, 14);  // IR3
    }

    void GTE::NCDT() {
        // Normal Color Depth Cue (triple)
        for (int v = 0; v < 3; v++) {
            int32_t nx = data_regs[v * 3];     // VX0/VX1/VX2
            int32_t ny = data_regs[v * 3 + 1]; // VY0/VY1/VY2
            int32_t nz = data_regs[v * 3 + 2]; // VZ0/VZ1/VZ2
            
            // Light matrix multiplication
            int64_t mac1 = ((int64_t)ctrl_regs[0] * nx + (int64_t)ctrl_regs[1] * ny + (int64_t)ctrl_regs[2] * nz) >> 12;
            int64_t mac2 = ((int64_t)ctrl_regs[3] * nx + (int64_t)ctrl_regs[4] * ny + (int64_t)ctrl_regs[5] * nz) >> 12;
            int64_t mac3 = ((int64_t)ctrl_regs[6] * nx + (int64_t)ctrl_regs[7] * ny + (int64_t)ctrl_regs[8] * nz) >> 12;
            
            // Store results
            data_regs[9 + v * 3] = Limit(mac1, 0x7FFFFFFF, -0x80000000, 12);   // IR1/IR2/IR3
            data_regs[10 + v * 3] = Limit(mac2, 0x7FFFFFFF, -0x80000000, 13);  // IR1/IR2/IR3
            data_regs[11 + v * 3] = Limit(mac3, 0x7FFFFFFF, -0x80000000, 14);  // IR1/IR2/IR3
        }
    }

    void GTE::NCCS() {
        // Normal Color Color (single)
        int32_t nx = data_regs[0];    // VX0
        int32_t ny = data_regs[1];    // VY0
        int32_t nz = data_regs[2];    // VZ0
        
        // Light matrix multiplication
        int64_t mac1 = ((int64_t)ctrl_regs[0] * nx + (int64_t)ctrl_regs[1] * ny + (int64_t)ctrl_regs[2] * nz) >> 12;
        int64_t mac2 = ((int64_t)ctrl_regs[3] * nx + (int64_t)ctrl_regs[4] * ny + (int64_t)ctrl_regs[5] * nz) >> 12;
        int64_t mac3 = ((int64_t)ctrl_regs[6] * nx + (int64_t)ctrl_regs[7] * ny + (int64_t)ctrl_regs[8] * nz) >> 12;
        
        // Color matrix multiplication
        mac1 = ((int64_t)ctrl_regs[13] * mac1 + (int64_t)ctrl_regs[14] * mac2 + (int64_t)ctrl_regs[15] * mac3) >> 12;
        mac2 = ((int64_t)ctrl_regs[16] * mac1 + (int64_t)ctrl_regs[17] * mac2 + (int64_t)ctrl_regs[18] * mac3) >> 12;
        mac3 = ((int64_t)ctrl_regs[19] * mac1 + (int64_t)ctrl_regs[20] * mac2 + (int64_t)ctrl_regs[21] * mac3) >> 12;
        
        data_regs[9] = Limit(mac1, 0x7FFFFFFF, -0x80000000, 12);   // IR1
        data_regs[10] = Limit(mac2, 0x7FFFFFFF, -0x80000000, 13);  // IR2
        data_regs[11] = Limit(mac3, 0x7FFFFFFF, -0x80000000, 14);  // IR3
    }

    void GTE::CC() {
        // Color Color
        int32_t r = data_regs[6] & 0xFF;
        int32_t g = (data_regs[6] >> 8) & 0xFF;
        int32_t b = (data_regs[6] >> 16) & 0xFF;
        
        // Color matrix multiplication
        int64_t mac1 = ((int64_t)ctrl_regs[13] * r + (int64_t)ctrl_regs[14] * g + (int64_t)ctrl_regs[15] * b) >> 12;
        int64_t mac2 = ((int64_t)ctrl_regs[16] * r + (int64_t)ctrl_regs[17] * g + (int64_t)ctrl_regs[18] * b) >> 12;
        int64_t mac3 = ((int64_t)ctrl_regs[19] * r + (int64_t)ctrl_regs[20] * g + (int64_t)ctrl_regs[21] * b) >> 12;
        
        data_regs[9] = Limit(mac1, 0x7FFFFFFF, -0x80000000, 12);   // IR1
        data_regs[10] = Limit(mac2, 0x7FFFFFFF, -0x80000000, 13);  // IR2
        data_regs[11] = Limit(mac3, 0x7FFFFFFF, -0x80000000, 14);  // IR3
    }

    void GTE::NCS() {
        // Normal Color (single)
        int32_t nx = data_regs[0];    // VX0
        int32_t ny = data_regs[1];    // VY0
        int32_t nz = data_regs[2];    // VZ0
        
        // Light matrix multiplication
        int64_t mac1 = ((int64_t)ctrl_regs[0] * nx + (int64_t)ctrl_regs[1] * ny + (int64_t)ctrl_regs[2] * nz) >> 12;
        int64_t mac2 = ((int64_t)ctrl_regs[3] * nx + (int64_t)ctrl_regs[4] * ny + (int64_t)ctrl_regs[5] * nz) >> 12;
        int64_t mac3 = ((int64_t)ctrl_regs[6] * nx + (int64_t)ctrl_regs[7] * ny + (int64_t)ctrl_regs[8] * nz) >> 12;
        
        data_regs[9] = Limit(mac1, 0x7FFFFFFF, -0x80000000, 12);   // IR1
        data_regs[10] = Limit(mac2, 0x7FFFFFFF, -0x80000000, 13);  // IR2
        data_regs[11] = Limit(mac3, 0x7FFFFFFF, -0x80000000, 14);  // IR3
    }

    void GTE::NCT() {
        // Normal Color (triple)
        for (int v = 0; v < 3; v++) {
            int32_t nx = data_regs[v * 3];     // VX0/VX1/VX2
            int32_t ny = data_regs[v * 3 + 1]; // VY0/VY1/VY2
            int32_t nz = data_regs[v * 3 + 2]; // VZ0/VZ1/VZ2
            
            // Light matrix multiplication
            int64_t mac1 = ((int64_t)ctrl_regs[0] * nx + (int64_t)ctrl_regs[1] * ny + (int64_t)ctrl_regs[2] * nz) >> 12;
            int64_t mac2 = ((int64_t)ctrl_regs[3] * nx + (int64_t)ctrl_regs[4] * ny + (int64_t)ctrl_regs[5] * nz) >> 12;
            int64_t mac3 = ((int64_t)ctrl_regs[6] * nx + (int64_t)ctrl_regs[7] * ny + (int64_t)ctrl_regs[8] * nz) >> 12;
            
            data_regs[9 + v * 3] = Limit(mac1, 0x7FFFFFFF, -0x80000000, 12);   // IR1/IR2/IR3
            data_regs[10 + v * 3] = Limit(mac2, 0x7FFFFFFF, -0x80000000, 13);  // IR1/IR2/IR3
            data_regs[11 + v * 3] = Limit(mac3, 0x7FFFFFFF, -0x80000000, 14);  // IR1/IR2/IR3
        }
    }

    void GTE::SQR() {
        // Square of vector IR1,IR2,IR3
        int64_t mac1 = ((int64_t)data_regs[9] * data_regs[9]) >> 12;   // IR1 * IR1
        int64_t mac2 = ((int64_t)data_regs[10] * data_regs[10]) >> 12; // IR2 * IR2
        int64_t mac3 = ((int64_t)data_regs[11] * data_regs[11]) >> 12; // IR3 * IR3
        
        data_regs[9] = Limit(mac1, 0x7FFFFFFF, -0x80000000, 12);
        data_regs[10] = Limit(mac2, 0x7FFFFFFF, -0x80000000, 13);
        data_regs[11] = Limit(mac3, 0x7FFFFFFF, -0x80000000, 14);
    }

    void GTE::DCPL() {
        // Depth Cue Color Light
        int32_t r = data_regs[6] & 0xFF;
        int32_t g = (data_regs[6] >> 8) & 0xFF;
        int32_t b = (data_regs[6] >> 16) & 0xFF;
        
        int64_t mac1 = ((int64_t)ctrl_regs[21] * r) >> 12;
        int64_t mac2 = ((int64_t)ctrl_regs[22] * g) >> 12;
        int64_t mac3 = ((int64_t)ctrl_regs[23] * b) >> 12;
        
        data_regs[9] = Limit(mac1, 0x7FFFFFFF, -0x80000000, 12);
        data_regs[10] = Limit(mac2, 0x7FFFFFFF, -0x80000000, 13);
        data_regs[11] = Limit(mac3, 0x7FFFFFFF, -0x80000000, 14);
    }

    void GTE::DPCT() {
        // Depth Cueing (triple)
        for (int v = 0; v < 3; v++) {
            int32_t r = data_regs[6 + v * 3] & 0xFF;
            int32_t g = (data_regs[6 + v * 3] >> 8) & 0xFF;
            int32_t b = (data_regs[6 + v * 3] >> 16) & 0xFF;
            
            int64_t mac1 = ((int64_t)ctrl_regs[21] * r) >> 12;
            int64_t mac2 = ((int64_t)ctrl_regs[22] * g) >> 12;
            int64_t mac3 = ((int64_t)ctrl_regs[23] * b) >> 12;
            
            data_regs[9 + v * 3] = Limit(mac1, 0x7FFFFFFF, -0x80000000, 12);
            data_regs[10 + v * 3] = Limit(mac2, 0x7FFFFFFF, -0x80000000, 13);
            data_regs[11 + v * 3] = Limit(mac3, 0x7FFFFFFF, -0x80000000, 14);
        }
    }

    void GTE::AVSZ3() {
        // Average of three Z values (SZ1, SZ2, SZ3)
        int32_t sz1 = data_regs[17];  // SZ1
        int32_t sz2 = data_regs[18];  // SZ2
        int32_t sz3 = data_regs[19];  // SZ3
        
        // Calculate average using ZSF3 (ctrl_regs[29])
        int64_t mac0 = ((int64_t)sz1 + (int64_t)sz2 + (int64_t)sz3) * ctrl_regs[29];
        
        // Store result in OTZ
        data_regs[7] = Limit(mac0 >> 12, 0xFFFF, 0, 12);  // OTZ
    }

    void GTE::AVSZ4() {
        // Average of four Z values (SZ0, SZ1, SZ2, SZ3)
        int32_t sz0 = data_regs[16];  // SZ0
        int32_t sz1 = data_regs[17];  // SZ1
        int32_t sz2 = data_regs[18];  // SZ2
        int32_t sz3 = data_regs[19];  // SZ3
        
        // Calculate average using ZSF4 (ctrl_regs[30])
        int64_t mac0 = ((int64_t)sz0 + (int64_t)sz1 + (int64_t)sz2 + (int64_t)sz3) * ctrl_regs[30];
        
        // Store result in OTZ
        data_regs[7] = Limit(mac0 >> 12, 0xFFFF, 0, 12);  // OTZ
    }

    void GTE::RTPT() {
        // Perspective Transformation (triple)
        for (int v = 0; v < 3; v++) {
            int32_t vx = data_regs[v * 3];     // VX0/VX1/VX2
            int32_t vy = data_regs[v * 3 + 1]; // VY0/VY1/VY2
            int32_t vz = data_regs[v * 3 + 2]; // VZ0/VZ1/VZ2
            
            // Rotation matrix multiplication
            int64_t mac1 = ((int64_t)ctrl_regs[0] * vx + (int64_t)ctrl_regs[1] * vy + (int64_t)ctrl_regs[2] * vz) >> 12;
            int64_t mac2 = ((int64_t)ctrl_regs[3] * vx + (int64_t)ctrl_regs[4] * vy + (int64_t)ctrl_regs[5] * vz) >> 12;
            int64_t mac3 = ((int64_t)ctrl_regs[6] * vx + (int64_t)ctrl_regs[7] * vy + (int64_t)ctrl_regs[8] * vz) >> 12;
            
            // Translation vector addition
            mac1 += ctrl_regs[9];   // TRX
            mac2 += ctrl_regs[10];  // TRY
            mac3 += ctrl_regs[11];  // TRZ
            
            // Store intermediate results
            data_regs[9] = Limit(mac1, 0x7FFFFFFF, -0x80000000, 12);   // IR1
            data_regs[10] = Limit(mac2, 0x7FFFFFFF, -0x80000000, 13);  // IR2
            data_regs[11] = Limit(mac3, 0x7FFFFFFF, -0x80000000, 14);  // IR3
            
            // Perspective division (if Z != 0)
            if (mac3 != 0) {
                int32_t h = ctrl_regs[26];  // H
                mac1 = ((int64_t)h * mac1) / mac3;
                mac2 = ((int64_t)h * mac2) / mac3;
            }
            
            // Store screen coordinates
            data_regs[12 + v * 2] = Limit(mac1 + ctrl_regs[24], 0x3FF, -0x400, 14);  // SX0/SX1/SX2
            data_regs[13 + v * 2] = Limit(mac2 + ctrl_regs[25], 0x3FF, -0x400, 13);  // SY0/SY1/SY2
            data_regs[16 + v] = Limit(mac3, 0xFFFF, 0, 12);  // SZ0/SZ1/SZ2
        }
    }

    uint32_t GTE::ReadData(uint8_t reg) {
        if (reg >= 32) return 0;
        return data_regs[reg];
    }

    void GTE::WriteData(uint8_t reg, uint32_t value) {
        if (reg >= 32) return;
        data_regs[reg] = value;
    }

    uint32_t GTE::ReadControl(uint8_t reg) {
        if (reg >= 32) return 0;
        return ctrl_regs[reg];
    }

    void GTE::WriteControl(uint8_t reg, uint32_t value) {
        if (reg >= 32) return;
        ctrl_regs[reg] = value;
    }
}