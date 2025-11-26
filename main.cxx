/**
 * @file test_polygon.cxx
 * @brief CUBO 3D → PIRÂMIDE → CUBO (Morfismo + Rotação 3D)
 * 
 * O cubo gira nos 3 eixos enquanto se deforma em pirâmide e volta
 * Os 4 vértices superiores colapsam para o centro-topo
 * 
 * Cada comando VDP1 ocupa 32 bytes (16 words)
 */

#include <stdint.h>

// ============================================================================
// VDP1 REGISTERS
// ============================================================================
#define VDP1_TVMR   (*(volatile uint16_t*)0x25D00000)
#define VDP1_FBCR   (*(volatile uint16_t*)0x25D00002)
#define VDP1_PTMR   (*(volatile uint16_t*)0x25D00004)
#define VDP1_EWDR   (*(volatile uint16_t*)0x25D00006)
#define VDP1_EWLR   (*(volatile uint16_t*)0x25D00008)
#define VDP1_EWRR   (*(volatile uint16_t*)0x25D0000A)
#define VDP1_ENDR   (*(volatile uint16_t*)0x25D0000C)
#define VDP1_EDSR   (*(volatile uint16_t*)0x25D00010)
#define VDP1_VRAM   ((volatile uint16_t*)0x25C00000)

// ============================================================================
// VDP2 REGISTERS
// ============================================================================
#define VDP2_TVMD   (*(volatile uint16_t*)0x25F80000)
#define VDP2_EXTEN  (*(volatile uint16_t*)0x25F80002)
#define VDP2_TVSTAT (*(volatile uint16_t*)0x25F80004)
#define VDP2_BGON   (*(volatile uint16_t*)0x25F80020)
#define VDP2_BKTAU  (*(volatile uint16_t*)0x25F800AC)
#define VDP2_BKTAL  (*(volatile uint16_t*)0x25F800AE)
#define VDP2_SPCTL  (*(volatile uint16_t*)0x25F800E0)
#define VDP2_PRISA  (*(volatile uint16_t*)0x25F800F0)
#define VDP2_PRISB  (*(volatile uint16_t*)0x25F800F2)
#define VDP2_PRISC  (*(volatile uint16_t*)0x25F800F4)
#define VDP2_PRISD  (*(volatile uint16_t*)0x25F800F6)
#define VDP2_PRINA  (*(volatile uint16_t*)0x25F800F8)
#define VDP2_PRINB  (*(volatile uint16_t*)0x25F800FA)
#define VDP2_VRAM   ((volatile uint16_t*)0x25E00000)

// ============================================================================
// VDP1 COMMAND TYPES
// ============================================================================
#define VDP1_CMD_NORMAL_SPRITE     0x0000
#define VDP1_CMD_SCALED_SPRITE     0x0001
#define VDP1_CMD_DISTORTED_SPRITE  0x0002
#define VDP1_CMD_POLYGON           0x0004
#define VDP1_CMD_POLYLINE          0x0005
#define VDP1_CMD_LINE              0x0006
#define VDP1_CMD_USER_CLIP         0x0008
#define VDP1_CMD_SYSTEM_CLIP       0x0009
#define VDP1_CMD_LOCAL_COORD       0x000A
#define VDP1_CMD_END               0x8000

// ============================================================================
// COLORS (RGB1555)
// ============================================================================
#define RGB(r,g,b) ((uint16_t)((1<<15)|((b)<<10)|((g)<<5)|(r)))
#define DARK_BLUE RGB(4, 8, 16)
#define YELLOW    RGB(31, 31, 0)
#define RED       RGB(31, 0, 0)
#define GREEN     RGB(0, 31, 0)
#define WHITE     RGB(31, 31, 31)
#define CYAN      RGB(0, 31, 31)
#define ORANGE    RGB(31, 16, 0)
#define PURPLE    RGB(20, 0, 31)

// ============================================================================
// SINE TABLE (256 entries, -127 to +127)
// ============================================================================
static const int8_t sintab[256] = {
    0, 3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45,
    48, 51, 54, 57, 59, 62, 65, 67, 70, 73, 75, 78, 80, 82, 85, 87,
    89, 91, 94, 96, 98, 100, 102, 103, 105, 107, 108, 110, 112, 113, 114, 116,
    117, 118, 119, 120, 121, 122, 123, 123, 124, 125, 125, 126, 126, 126, 126, 126,
    127, 126, 126, 126, 126, 126, 125, 125, 124, 123, 123, 122, 121, 120, 119, 118,
    117, 116, 114, 113, 112, 110, 108, 107, 105, 103, 102, 100, 98, 96, 94, 91,
    89, 87, 85, 82, 80, 78, 75, 73, 70, 67, 65, 62, 59, 57, 54, 51,
    48, 45, 42, 39, 36, 33, 30, 27, 24, 21, 18, 15, 12, 9, 6, 3,
    0, -3, -6, -9, -12, -15, -18, -21, -24, -27, -30, -33, -36, -39, -42, -45,
    -48, -51, -54, -57, -59, -62, -65, -67, -70, -73, -75, -78, -80, -82, -85, -87,
    -89, -91, -94, -96, -98, -100, -102, -103, -105, -107, -108, -110, -112, -113, -114, -116,
    -117, -118, -119, -120, -121, -122, -123, -123, -124, -125, -125, -126, -126, -126, -126, -126,
    -127, -126, -126, -126, -126, -126, -125, -125, -124, -123, -123, -122, -121, -120, -119, -118,
    -117, -116, -114, -113, -112, -110, -108, -107, -105, -103, -102, -100, -98, -96, -94, -91,
    -89, -87, -85, -82, -80, -78, -75, -73, -70, -67, -65, -62, -59, -57, -54, -51,
    -48, -45, -42, -39, -36, -33, -30, -27, -24, -21, -18, -15, -12, -9, -6, -3
};

static inline int16_t fsin(uint8_t angle) { return sintab[angle]; }
static inline int16_t fcos(uint8_t angle) { return sintab[(angle + 64) & 255]; }

// ============================================================================
// ESTRUTURAS 3D
// ============================================================================
struct Vec3 {
    int16_t x, y, z;
};

struct Vec2 {
    int16_t x, y;
};

// ============================================================================
// CUBO - 8 vértices (BASE)
// ============================================================================
//       4-------5
//      /|      /|
//     / |     / |
//    0-------1  |
//    |  7----|--6
//    | /     | /
//    |/      |/
//    3-------2
//
// CUBO (forma inicial)
static const Vec3 cubeBase[8] = {
    {-40, -40, -40},  // 0: front top left
    { 40, -40, -40},  // 1: front top right
    { 40,  40, -40},  // 2: front bottom right
    {-40,  40, -40},  // 3: front bottom left
    {-40, -40,  40},  // 4: back top left
    { 40, -40,  40},  // 5: back top right
    { 40,  40,  40},  // 6: back bottom right
    {-40,  40,  40},  // 7: back bottom left
};

// PIRÂMIDE (vértices 0,1,4,5 colapsam no topo)
static const Vec3 pyramidTarget[8] = {
    {  0, -70,   0},  // 0: → topo central
    {  0, -70,   0},  // 1: → topo central
    { 50,  40, -50},  // 2: front bottom right (maior)
    {-50,  40, -50},  // 3: front bottom left (maior)
    {  0, -70,   0},  // 4: → topo central
    {  0, -70,   0},  // 5: → topo central
    { 50,  40,  50},  // 6: back bottom right (maior)
    {-50,  40,  50},  // 7: back bottom left (maior)
};

// Vértices animados (interpolados)
static Vec3 cubeVerts[8];

// 6 faces (4 vértices cada)
static const uint8_t cubeFaces[6][4] = {
    {0, 1, 2, 3},  // Front  (Z-)
    {5, 4, 7, 6},  // Back   (Z+)
    {4, 0, 3, 7},  // Left   (X-)
    {1, 5, 6, 2},  // Right  (X+)
    {4, 5, 1, 0},  // Top    (Y-)
    {3, 2, 6, 7},  // Bottom (Y+)
};

// Cores das faces
static const uint16_t faceColors[6] = {
    RGB(31, 0, 0),    // Front - Red
    RGB(0, 31, 0),    // Back - Green
    RGB(0, 0, 31),    // Left - Blue
    RGB(31, 31, 0),   // Right - Yellow
    RGB(31, 0, 31),   // Top - Magenta
    RGB(0, 31, 31),   // Bottom - Cyan
};

// Vértices transformados
static Vec3 transformed[8];
static Vec2 projected[8];

// ============================================================================
// VDP1 COMMAND STRUCTURE (32 bytes = 16 words)
// ============================================================================
struct VDP1Command {
    uint16_t ctrl;      // [0] Command control
    uint16_t link;      // [1] Link pointer
    uint16_t pmod;      // [2] Draw mode
    uint16_t colr;      // [3] Color
    uint16_t srca;      // [4] Source address
    uint16_t size;      // [5] Size
    int16_t  xa, ya;    // [6-7] Vertex A
    int16_t  xb, yb;    // [8-9] Vertex B
    int16_t  xc, yc;    // [10-11] Vertex C
    int16_t  xd, yd;    // [12-13] Vertex D
    uint16_t grda;      // [14] Gouraud table
    uint16_t reserved;  // [15] Reserved
};

// ============================================================================
// HELPERS
// ============================================================================
static inline void delay(volatile int n) {
    while (n-- > 0) __asm__ volatile("nop");
}

static inline void WaitVBlankIn(void) {
    while ((VDP2_TVSTAT & 0x0008) == 0) {}
}

static inline void WaitVBlankOut(void) {
    while ((VDP2_TVSTAT & 0x0008) != 0) {}
}

// ============================================================================
// INITIALIZATION
// ============================================================================
static void VDP2_Init(void) {
    VDP2_TVMD = 0x0000;  // Display off
    delay(1000);
    
    VDP2_EXTEN = 0x0000;
    VDP2_BGON = 0x0000;  // No backgrounds
    
    // Back screen color
    VDP2_VRAM[0] = DARK_BLUE;
    VDP2_BKTAU = 0x0000;
    VDP2_BKTAL = 0x0000;
    
    // SPCTL: Enable sprite display
    // Bit 5 (SPCLMD) = 1: RGB mode (use 16-bit colors directly)
    // Bits 3-0 (SPTYPE) = 0: Sprite type 0
    VDP2_SPCTL = 0x0020;
    
    // Sprite priorities (7 = highest)
    VDP2_PRISA = 0x0707;
    VDP2_PRISB = 0x0707;
    VDP2_PRISC = 0x0707;
    VDP2_PRISD = 0x0707;
    VDP2_PRINA = 0x0000;
    VDP2_PRINB = 0x0000;
    
    VDP2_TVMD = 0x8000;  // NTSC 320x224, display on
}

static void VDP1_Init(void) {
    // Stop drawing
    VDP1_PTMR = 0x0000;
    VDP1_ENDR = 0x0000;
    delay(10000);
    
    // TV Mode: 16-bit, no rotation
    VDP1_TVMR = 0x0000;
    
    // Frame Buffer: auto erase/change
    VDP1_FBCR = 0x0000;
    
    // Erase color
    VDP1_EWDR = DARK_BLUE;
    
    // Erase area: full screen
    VDP1_EWLR = 0x0000;           // Top-left (0,0)
    VDP1_EWRR = (39 << 9) | 223;  // Bottom-right (312,223)
    
    // Clear command area
    for (int i = 0; i < 512; i++) {
        VDP1_VRAM[i] = 0;
    }
}

// ============================================================================
// INTERPOLAÇÃO LINEAR
// ============================================================================
static inline int16_t lerp(int16_t a, int16_t b, int16_t t) {
    // t vai de 0 a 127
    return a + (((b - a) * t) >> 7);
}

// Interpolar forma do cubo para pirâmide
static void MorphShape(int16_t t) {
    for (int i = 0; i < 8; i++) {
        cubeVerts[i].x = lerp(cubeBase[i].x, pyramidTarget[i].x, t);
        cubeVerts[i].y = lerp(cubeBase[i].y, pyramidTarget[i].y, t);
        cubeVerts[i].z = lerp(cubeBase[i].z, pyramidTarget[i].z, t);
    }
}

// ============================================================================
// ROTAÇÃO 3D
// ============================================================================
static void RotateAndProject(uint8_t angleX, uint8_t angleY, uint8_t angleZ) {
    int16_t sx = fsin(angleX), cx = fcos(angleX);
    int16_t sy = fsin(angleY), cy = fcos(angleY);
    int16_t sz = fsin(angleZ), cz = fcos(angleZ);
    
    for (int i = 0; i < 8; i++) {
        int16_t x = cubeVerts[i].x;
        int16_t y = cubeVerts[i].y;
        int16_t z = cubeVerts[i].z;
        
        // Rotação em Z
        int16_t x1 = (x * cz - y * sz) >> 7;
        int16_t y1 = (x * sz + y * cz) >> 7;
        int16_t z1 = z;
        
        // Rotação em Y
        int16_t x2 = (x1 * cy + z1 * sy) >> 7;
        int16_t y2 = y1;
        int16_t z2 = (-x1 * sy + z1 * cy) >> 7;
        
        // Rotação em X
        int16_t x3 = x2;
        int16_t y3 = (y2 * cx - z2 * sx) >> 7;
        int16_t z3 = (y2 * sx + z2 * cx) >> 7;
        
        transformed[i].x = x3;
        transformed[i].y = y3;
        transformed[i].z = z3;
        
        // Projeção perspectiva
        // Z varia de -40 a +40, adicionamos offset para evitar divisão por zero
        int16_t zOffset = z3 + 150;  // Distância da câmera
        if (zOffset < 10) zOffset = 10;
        
        // Projeção: x' = x * focal / z
        const int16_t FOCAL = 200;
        projected[i].x = (x3 * FOCAL) / zOffset;
        projected[i].y = (y3 * FOCAL) / zOffset;
    }
}

// Calcular normal da face (para backface culling)
static int16_t FaceNormalZ(int face) {
    int i0 = cubeFaces[face][0];
    int i1 = cubeFaces[face][1];
    int i2 = cubeFaces[face][2];
    
    // Vetores da face
    int16_t ax = projected[i1].x - projected[i0].x;
    int16_t ay = projected[i1].y - projected[i0].y;
    int16_t bx = projected[i2].x - projected[i0].x;
    int16_t by = projected[i2].y - projected[i0].y;
    
    // Produto vetorial (apenas Z, que é o que nos interessa)
    return (ax * by - ay * bx);
}

// ============================================================================
// BUILD COMMAND TABLE
// ============================================================================
static void BuildCommands(void) {
    volatile VDP1Command* cmd = (volatile VDP1Command*)VDP1_VRAM;
    
    // Command 0: System Clip
    cmd[0].ctrl = VDP1_CMD_SYSTEM_CLIP;
    cmd[0].link = 0;
    cmd[0].pmod = 0;
    cmd[0].colr = 0;
    cmd[0].srca = 0;
    cmd[0].size = 0;
    cmd[0].xa = 0;
    cmd[0].ya = 0;
    cmd[0].xb = 0;
    cmd[0].yb = 0;
    cmd[0].xc = 319;
    cmd[0].yc = 223;
    cmd[0].xd = 0;
    cmd[0].yd = 0;
    cmd[0].grda = 0;
    cmd[0].reserved = 0;
    
    // Command 1: Local Coordinates
    cmd[1].ctrl = VDP1_CMD_LOCAL_COORD;
    cmd[1].link = 0;
    cmd[1].pmod = 0;
    cmd[1].colr = 0;
    cmd[1].srca = 0;
    cmd[1].size = 0;
    cmd[1].xa = 160;
    cmd[1].ya = 112;
    cmd[1].xb = 0;
    cmd[1].yb = 0;
    cmd[1].xc = 0;
    cmd[1].yc = 0;
    cmd[1].xd = 0;
    cmd[1].yd = 0;
    cmd[1].grda = 0;
    cmd[1].reserved = 0;
    
    // Commands 2-7: 6 faces do cubo (inicialmente vazias)
    for (int i = 0; i < 6; i++) {
        cmd[2 + i].ctrl = VDP1_CMD_POLYGON;
        cmd[2 + i].link = 0;
        cmd[2 + i].pmod = 0x00C0;
        cmd[2 + i].colr = faceColors[i];
        cmd[2 + i].srca = 0;
        cmd[2 + i].size = 0;
        cmd[2 + i].xa = 0;
        cmd[2 + i].ya = 0;
        cmd[2 + i].xb = 0;
        cmd[2 + i].yb = 0;
        cmd[2 + i].xc = 0;
        cmd[2 + i].yc = 0;
        cmd[2 + i].xd = 0;
        cmd[2 + i].yd = 0;
        cmd[2 + i].grda = 0;
        cmd[2 + i].reserved = 0;
    }
    
    // Command 8: End
    cmd[8].ctrl = VDP1_CMD_END;
    cmd[8].link = 0;
    cmd[8].pmod = 0;
    cmd[8].colr = 0;
    cmd[8].srca = 0;
    cmd[8].size = 0;
    cmd[8].xa = 0;
    cmd[8].ya = 0;
    cmd[8].xb = 0;
    cmd[8].yb = 0;
    cmd[8].xc = 0;
    cmd[8].yc = 0;
    cmd[8].xd = 0;
    cmd[8].yd = 0;
    cmd[8].grda = 0;
    cmd[8].reserved = 0;
}

// ============================================================================
// MAIN - Cubo 3D Rotacionando + Deformando
// ============================================================================
int main(void) {
    VDP2_Init();
    VDP1_Init();
    BuildCommands();
    
    volatile VDP1Command* cmd = (volatile VDP1Command*)VDP1_VRAM;
    uint32_t frame = 0;
    
    while (1) {
        WaitVBlankIn();
        
        // Trigger draw
        VDP1_PTMR = 0x0001;
        
        WaitVBlankOut();
        
        // ================================
        // MORFISMO: Cubo ↔ Pirâmide
        // ================================
        // Usa seno para ir e voltar suavemente
        uint8_t morphPhase = frame;  // ciclo completo a cada 256 frames
        int16_t morphT = (fsin(morphPhase) + 127) >> 1;  // 0 a 127
        
        // Aplicar deformação
        MorphShape(morphT);
        
        // ================================
        // ROTAÇÃO 3D
        // ================================
        uint8_t angleX = frame;
        uint8_t angleY = (frame * 2) / 3;
        uint8_t angleZ = frame / 2;
        
        // Transformar vértices
        RotateAndProject(angleX, angleY, angleZ);
        
        // ================================
        // RENDERIZAR FACES
        // ================================
        int cmdIdx = 2;
        for (int face = 0; face < 6; face++) {
            // Backface culling
            int16_t nz = FaceNormalZ(face);
            
            if (nz < 0) {
                // Face visível
                int i0 = cubeFaces[face][0];
                int i1 = cubeFaces[face][1];
                int i2 = cubeFaces[face][2];
                int i3 = cubeFaces[face][3];
                
                cmd[cmdIdx].ctrl = VDP1_CMD_POLYGON;
                
                // Cor varia com o morph (mais saturada quando pirâmide)
                uint16_t baseColor = faceColors[face];
                uint8_t r = (baseColor & 0x001F);
                uint8_t g = (baseColor >> 5) & 0x001F;
                uint8_t b = (baseColor >> 10) & 0x001F;
                
                // Aumentar brilho durante morph
                int16_t boost = morphT >> 3;  // 0 a 15
                r = (r + boost > 31) ? 31 : r + boost;
                g = (g + boost > 31) ? 31 : g + boost;
                b = (b + boost > 31) ? 31 : b + boost;
                
                cmd[cmdIdx].colr = RGB(r, g, b);
                cmd[cmdIdx].xa = projected[i0].x;
                cmd[cmdIdx].ya = projected[i0].y;
                cmd[cmdIdx].xb = projected[i1].x;
                cmd[cmdIdx].yb = projected[i1].y;
                cmd[cmdIdx].xc = projected[i2].x;
                cmd[cmdIdx].yc = projected[i2].y;
                cmd[cmdIdx].xd = projected[i3].x;
                cmd[cmdIdx].yd = projected[i3].y;
                cmdIdx++;
            }
        }
        
        // Terminar lista de comandos
        cmd[cmdIdx].ctrl = VDP1_CMD_END;
        
        frame++;
    }
    
    return 0;
}
