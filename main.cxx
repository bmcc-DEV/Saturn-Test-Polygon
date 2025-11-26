/**
 * @file test_polygon.cxx
 * @brief TESSERACT 4D - HIPERCUBO COM FÍSICA LAGRANGIANA
 * 
 * Tesseract (8-cell): Análogo 4D do cubo
 *   - 16 vértices (2⁴)
 *   - 32 arestas
 *   - 24 faces quadradas
 *   - 8 células cúbicas
 * 
 * 7 OPERADORES DO PROTOCOLO DE DINÂMICA COMPUTACIONAL:
 *   1. HIPOTENUSA  - Magnitude 4D dos vértices
 *   2. SATURAÇÃO   - Clamp de coordenadas e cores
 *   3. LOOP        - Ciclo de rotação angular
 *   4. AJUSTE      - Interpolação de projeção 4D→3D
 *   5. INTERSEÇÃO  - Frustum culling
 *   6. REFLEXÃO    - Oscilação de rotação 4D
 *   7. CRONOS      - Delta time para física
 * 
 * MECÂNICA ANALÍTICA/LAGRANGIANA:
 *   L = T - V (Lagrangiano)
 *   H = T + V (Hamiltoniano = Energia Total)
 *   S = ∫L dt (Ação - Princípio de Mínima Ação)
 *   p = ∂L/∂q̇ (Momento Generalizado)
 *   F = -∂V/∂q (Força Generalizada)
 * 
 * TOPOLOGIA 4D:
 *   χ = V - E + F - C + H = 0 (Característica de Euler 4D)
 *   Para Tesseract: 16 - 32 + 24 - 8 + 1 = 1 (fronteira)
 * 
 * Controles: START=ciclar FPS | A=30 | B=60 | C=120 | X=240
 * Autor: Bruno Monteiro Caldas da Cunha
 */

#include <stdint.h>

// ============================================================================
// VDP1/VDP2 REGISTERS
// ============================================================================
#define VDP1_TVMR   (*(volatile uint16_t*)0x25D00000)
#define VDP1_FBCR   (*(volatile uint16_t*)0x25D00002)
#define VDP1_PTMR   (*(volatile uint16_t*)0x25D00004)
#define VDP1_EWDR   (*(volatile uint16_t*)0x25D00006)
#define VDP1_EWLR   (*(volatile uint16_t*)0x25D00008)
#define VDP1_EWRR   (*(volatile uint16_t*)0x25D0000A)
#define VDP1_ENDR   (*(volatile uint16_t*)0x25D0000C)
#define VDP1_VRAM   ((volatile uint16_t*)0x25C00000)

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

// SMPC (Controle)
#define SMPC_SF     (*(volatile uint8_t*)0x20100063)
#define SMPC_SR     (*(volatile uint8_t*)0x20100061)
#define SMPC_COMREG (*(volatile uint8_t*)0x2010001F)
#define SMPC_PDR1   (*(volatile uint8_t*)0x20100075)
#define SMPC_PDR2   (*(volatile uint8_t*)0x20100077)
#define SMPC_DDR1   (*(volatile uint8_t*)0x20100079)
#define SMPC_DDR2   (*(volatile uint8_t*)0x2010007B)
#define SMPC_IOSEL  (*(volatile uint8_t*)0x2010007D)
#define SMPC_EXLE   (*(volatile uint8_t*)0x2010007F)

// Peripheral data (depois de INTBACK)
#define SMPC_OREG(n) (*(volatile uint8_t*)(0x20100021 + (n)*2))

// Botões do Saturn (active low)
#define PAD_RIGHT  0x8000
#define PAD_LEFT   0x4000
#define PAD_DOWN   0x2000
#define PAD_UP     0x1000
#define PAD_START  0x0800
#define PAD_A      0x0400
#define PAD_C      0x0200
#define PAD_B      0x0100
#define PAD_R      0x0080
#define PAD_X      0x0040
#define PAD_Y      0x0020
#define PAD_Z      0x0010
#define PAD_L      0x0008

// ============================================================================
// CONSTANTS
// ============================================================================
#define VDP1_CMD_POLYGON      0x0004
#define VDP1_CMD_SYSTEM_CLIP  0x0009
#define VDP1_CMD_LOCAL_COORD  0x000A
#define VDP1_CMD_END          0x8000

#define RGB(r,g,b) ((uint16_t)((1<<15)|((b)<<10)|((g)<<5)|(r)))
#define DARK_BLUE RGB(4, 8, 16)

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

static inline int16_t fsin(uint8_t a) { return sintab[a]; }
static inline int16_t fcos(uint8_t a) { return sintab[(a + 64) & 255]; }

// ============================================================================
// PROTOCOLO DE DINÂMICA COMPUTACIONAL (7 OPERADORES)
// ============================================================================

// 1. HIPOTENUSA (Magnitude 4D)
static int32_t Op_Hipotenusa(int32_t x, int32_t y, int32_t z, int32_t w) {
    uint32_t sum = (x*x) + (y*y) + (z*z) + (w*w);
    if (sum == 0) return 0;
    uint32_t res = 0, bit = 1 << 30;
    while (bit > sum) bit >>= 2;
    while (bit != 0) {
        if (sum >= res + bit) { sum -= res + bit; res = (res >> 1) + bit; }
        else { res >>= 1; }
        bit >>= 2;
    }
    return (int32_t)res;
}

// 2. SATURAÇÃO (Clamp)
static int32_t Op_Saturacao(int32_t val, int32_t minV, int32_t maxV) {
    if (val < minV) return minV;
    if (val > maxV) return maxV;
    return val;
}

// 3. LOOP (Modulo seguro)
static int32_t Op_Loop(int32_t val, int32_t len) {
    if (len <= 0) return 0;
    val = val % len;
    if (val < 0) val += len;
    return val;
}

// 4. AJUSTE (Interpolação Linear)
static int32_t Op_Ajuste(int32_t a, int32_t b, int32_t t, int32_t shift) {
    return a + (((b - a) * t) >> shift);
}

// 5. INTERSEÇÃO (Teste AABB 1D)
static int32_t Op_Intersecao(int32_t minA, int32_t maxA, int32_t minB, int32_t maxB) {
    return (minA <= maxB && maxA >= minB) ? 1 : 0;
}

// 6. REFLEXÃO (Ping-pong)
static int32_t Op_Reflexao(int32_t val, int32_t max) {
    if (max <= 0) return 0;
    int32_t cycle = val / max;
    int32_t pos = val % max;
    if (pos < 0) { pos += max; cycle--; }
    return (cycle & 1) ? (max - 1 - pos) : pos;
}

// 7. CRONOS (Delta time scaling)
static int32_t Op_Cronos(int32_t val, int32_t dt, int32_t shift) {
    return (val * dt) >> shift;
}

// ============================================================================
// MECÂNICA CAUSAL & TOPOLOGIA
// ============================================================================

// Característica de Euler: χ = V - E + F
static int32_t Topology_EulerChar(int nVerts, int nEdges, int nFaces) {
    return nVerts - nEdges + nFaces;
}

// Curvatura Gaussiana Discreta
static int32_t Topology_GaussianCurvature(int facesAtVertex, int32_t areaScale) {
    int32_t idealFaces = 4;
    int32_t deficit = (idealFaces - facesAtVertex) * 90;
    if (areaScale == 0) areaScale = 1;
    return (deficit << 8) / areaScale;
}

// Gênero Topológico: g = 1 - χ/2
static int32_t Topology_Genus(int32_t eulerChar) {
    return (2 - eulerChar) >> 1;
}

// ============================================================================
// MECÂNICA ANALÍTICA / LAGRANGIANA (L = T - V)
// ============================================================================

// Energia Potencial: V = m * g * h
static int32_t Lagrangian_PotentialEnergy(int32_t mass, int32_t height, int32_t gravity) {
    return (mass * gravity * height) >> 8;
}

// Lagrangiano: L = T - V
static int32_t Lagrangian_L(int32_t T, int32_t V) { return T - V; }

// Hamiltoniano: H = T + V (Energia Total)
static int32_t Lagrangian_Hamiltonian(int32_t T, int32_t V) { return T + V; }

// Acumulador de Ação: S = ∫ L dt
static int32_t action_accumulator = 0;
static void Lagrangian_AccumulateAction(int32_t L, int32_t dt) {
    action_accumulator += Op_Cronos(L, dt, 8);
}

// Momento: p = m * v
static int32_t Lagrangian_Momentum(int32_t mass, int32_t velocity) {
    return (mass * velocity) >> 4;
}

// Força: F = -∂V/∂q
static int32_t Lagrangian_Force(int32_t V1, int32_t V2, int32_t dist) {
    if (dist == 0) dist = 1;
    return -((V2 - V1) << 8) / dist;
}

// ============================================================================
// ESTRUTURAS 3D/4D
// ============================================================================
struct Vec4 { int16_t x, y, z, w; };
struct Vec3 { int16_t x, y, z; };
struct Vec2 { int16_t x, y; };

struct VDP1Command {
    uint16_t ctrl, link, pmod, colr, srca, size;
    int16_t xa, ya, xb, yb, xc, yc, xd, yd;
    uint16_t grda, reserved;
};

#define MAX_VERTS 16
#define MAX_FACES 24

// ============================================================================
// 5 FORMAS: Tetraedro → Cubo → Octaedro → Dodecaedro → Tesseract
// ============================================================================

// TETRAEDRO (4 vértices, 4 faces triangulares)
static const Vec4 VERTS_TETRA[MAX_VERTS] = {
    {  0, -50,   0,  0}, // Topo
    {-40,  40, -30,  0}, // Base 1
    { 40,  40, -30,  0}, // Base 2
    {  0,  40,  50,  0}, // Base 3
    // Resto zerado
    {0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},
    {0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},
    {0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},
};
static const uint8_t FACES_TETRA[MAX_FACES][4] = {
    {0, 1, 2, 2}, {0, 2, 3, 3}, {0, 3, 1, 1}, {1, 3, 2, 2}, // 4 triângulos
};
#define NFACES_TETRA 4

// CUBO (8 vértices, 6 faces)
static const Vec4 VERTS_CUBE[MAX_VERTS] = {
    {-35,-35,-35, 0}, { 35,-35,-35, 0}, { 35, 35,-35, 0}, {-35, 35,-35, 0},
    {-35,-35, 35, 0}, { 35,-35, 35, 0}, { 35, 35, 35, 0}, {-35, 35, 35, 0},
    {0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},
    {0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},
};
static const uint8_t FACES_CUBE[MAX_FACES][4] = {
    {0,1,2,3}, {5,4,7,6}, {4,0,3,7}, {1,5,6,2}, {4,5,1,0}, {3,2,6,7},
};
#define NFACES_CUBE 6

// OCTAEDRO (6 vértices, 8 faces)
static const Vec4 VERTS_OCTA[MAX_VERTS] = {
    {  0,-50,  0, 0}, {  0, 50,  0, 0}, // Topo/Base
    {-40,  0,-40, 0}, { 40,  0,-40, 0}, { 40,  0, 40, 0}, {-40,  0, 40, 0},
    {0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},
    {0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},
};
static const uint8_t FACES_OCTA[MAX_FACES][4] = {
    {0,2,3,3}, {0,3,4,4}, {0,4,5,5}, {0,5,2,2}, // Topo
    {1,3,2,2}, {1,4,3,3}, {1,5,4,4}, {1,2,5,5}, // Base
};
#define NFACES_OCTA 8

// DODECAEDRO simplificado (12 faces pentagonais → quads)
static const Vec4 VERTS_DODECA[MAX_VERTS] = {
    {-25,-40,-25, 0}, { 25,-40,-25, 0}, { 40,-15,-40, 0}, {  0, 10,-50, 0},
    {-40,-15,-40, 0}, {-25,-40, 25, 0}, { 25,-40, 25, 0}, { 40,-15, 40, 0},
    {  0, 10, 50, 0}, {-40,-15, 40, 0}, {-25, 40,-25, 0}, { 25, 40,-25, 0},
    { 25, 40, 25, 0}, {-25, 40, 25, 0}, {  0, 50,  0, 0}, {  0,-50,  0, 0},
};
static const uint8_t FACES_DODECA[MAX_FACES][4] = {
    {0,1,2,3}, {0,3,4,4}, {0,4,9,5}, {5,6,7,8}, {5,8,9,9}, {1,6,7,2},
    {2,7,11,3}, {3,11,10,4}, {4,10,13,9}, {8,12,13,9}, {10,11,12,13}, {14,10,11,14},
};
#define NFACES_DODECA 12

// ============================================================================
// TESSERACT 4D PRECISO (Hipercubo)
// Vértices: todos os pontos (±1, ±1, ±1, ±1) escalados
// ============================================================================
#define TESS_SCALE 20  // Escala do tesseract (reduzido para widescreen)

// 16 vértices do Tesseract (2⁴ combinações de ±SCALE em 4D)
static const Vec4 VERTS_TESS[MAX_VERTS] = {
    // w = -SCALE (cubo "interno" na 4ª dimensão)
    {-TESS_SCALE, -TESS_SCALE, -TESS_SCALE, -TESS_SCALE},  // 0: ---−
    { TESS_SCALE, -TESS_SCALE, -TESS_SCALE, -TESS_SCALE},  // 1: +--−
    { TESS_SCALE,  TESS_SCALE, -TESS_SCALE, -TESS_SCALE},  // 2: ++-−
    {-TESS_SCALE,  TESS_SCALE, -TESS_SCALE, -TESS_SCALE},  // 3: -+-−
    {-TESS_SCALE, -TESS_SCALE,  TESS_SCALE, -TESS_SCALE},  // 4: --+−
    { TESS_SCALE, -TESS_SCALE,  TESS_SCALE, -TESS_SCALE},  // 5: +-+−
    { TESS_SCALE,  TESS_SCALE,  TESS_SCALE, -TESS_SCALE},  // 6: +++−
    {-TESS_SCALE,  TESS_SCALE,  TESS_SCALE, -TESS_SCALE},  // 7: -++−
    // w = +SCALE (cubo "externo" na 4ª dimensão)
    {-TESS_SCALE, -TESS_SCALE, -TESS_SCALE,  TESS_SCALE},  // 8: ---+
    { TESS_SCALE, -TESS_SCALE, -TESS_SCALE,  TESS_SCALE},  // 9: +--+
    { TESS_SCALE,  TESS_SCALE, -TESS_SCALE,  TESS_SCALE},  // 10: ++-+
    {-TESS_SCALE,  TESS_SCALE, -TESS_SCALE,  TESS_SCALE},  // 11: -+-+
    {-TESS_SCALE, -TESS_SCALE,  TESS_SCALE,  TESS_SCALE},  // 12: --++
    { TESS_SCALE, -TESS_SCALE,  TESS_SCALE,  TESS_SCALE},  // 13: +-++
    { TESS_SCALE,  TESS_SCALE,  TESS_SCALE,  TESS_SCALE},  // 14: ++++
    {-TESS_SCALE,  TESS_SCALE,  TESS_SCALE,  TESS_SCALE},  // 15: -+++
};

// 24 faces do Tesseract (6 por cada par de cubos + 12 conexões)
// Organizadas por célula cúbica para coloração correta
static const uint8_t FACES_TESS[MAX_FACES][4] = {
    // CÉLULA 1: Cubo w=-30 (faces do cubo interno)
    {0, 1, 2, 3},  // Face Z- (frente)
    {4, 7, 6, 5},  // Face Z+ (trás)
    {0, 4, 5, 1},  // Face Y- (baixo)
    {3, 2, 6, 7},  // Face Y+ (cima)
    {0, 3, 7, 4},  // Face X- (esquerda)
    {1, 5, 6, 2},  // Face X+ (direita)
    
    // CÉLULA 2: Cubo w=+30 (faces do cubo externo)
    {8, 11, 10, 9},   // Face Z-
    {12, 13, 14, 15}, // Face Z+
    {8, 9, 13, 12},   // Face Y-
    {11, 15, 14, 10}, // Face Y+
    {8, 12, 15, 11},  // Face X-
    {9, 10, 14, 13},  // Face X+
    
    // CÉLULAS 3-8: Conexões 4D (faces que conectam w- a w+)
    {0, 1, 9, 8},     // Conexão Y-Z-
    {1, 2, 10, 9},    // Conexão X+Z-
    {2, 3, 11, 10},   // Conexão Y+Z-
    {3, 0, 8, 11},    // Conexão X-Z-
    {4, 5, 13, 12},   // Conexão Y-Z+
    {5, 6, 14, 13},   // Conexão X+Z+
    {6, 7, 15, 14},   // Conexão Y+Z+
    {7, 4, 12, 15},   // Conexão X-Z+
    // Faces adicionais das células de conexão
    {0, 4, 12, 8},    // Conexão X-Y-
    {1, 9, 13, 5},    // Conexão X+Y-
    {2, 6, 14, 10},   // Conexão X+Y+
    {3, 7, 15, 11},   // Conexão X-Y+
};
#define NVERTS_TESS 16
#define NFACES_TESS 24
#define NEDGES_TESS 32
#define NCELLS_TESS 8

// Número de faces por forma (para referência)
static const int NFACES[5] = { NFACES_TETRA, NFACES_CUBE, NFACES_OCTA, NFACES_DODECA, NFACES_TESS };

// Cores por forma (gradiente do simples ao complexo)
static const uint16_t SHAPE_COLORS[5] = {
    RGB(31, 8, 8),   // Tetraedro: Vermelho
    RGB(8, 31, 8),   // Cubo: Verde
    RGB(8, 8, 31),   // Octaedro: Azul
    RGB(31, 31, 8),  // Dodecaedro: Amarelo
    RGB(31, 8, 31),  // Tesseract: Magenta
};

// Estado de morfismo
static Vec4 morphedVerts[MAX_VERTS];
static Vec4 prevVerts[MAX_VERTS];
static Vec4 velocity[MAX_VERTS];

// Vértices transformados e projetados
static Vec3 transformed[MAX_VERTS];
static Vec2 projected[MAX_VERTS];

// Estado da forma atual
static int currentNumFaces = NFACES_TETRA;

// Controle de FPS (30, 60, 120, 240)
static const int FPS_MODES[4] = {1, 2, 4, 8};  // SubFrames por VBlank
static int fpsMode = 3;  // Começa em 240 FPS (index 3)
static uint16_t padData = 0;
static uint16_t padPrev = 0;
static uint16_t padTrigger = 0;  // Borda de descida (botão pressionado)

// Estado físico do sistema (Mecânica Lagrangiana 4D)
static int32_t systemKineticEnergy = 0;    // T = Σ(½mv²)
static int32_t systemPotentialEnergy = 0;  // V = Σ(mgh)
static int32_t systemLagrangian = 0;       // L = T - V
static int32_t systemHamiltonian = 0;      // H = T + V (energia total)
static int32_t systemAction = 0;           // S = ∫L dt (acumulador)
static int32_t systemMomentum4D = 0;       // p = Σ|mv| (magnitude 4D)

// Ângulos de rotação 4D (6 planos de rotação)
static uint8_t rotXY = 0, rotXZ = 0, rotXW = 0;
static uint8_t rotYZ = 0, rotYW = 0, rotZW = 0;

// Velocidades angulares (coordenadas generalizadas q̇)
static int16_t omegaXY = 0, omegaXZ = 0, omegaXW = 0;
static int16_t omegaYZ = 0, omegaYW = 0, omegaZW = 0;

// ============================================================================
// HELPERS
// ============================================================================
static inline void delay(volatile int n) { while (n-- > 0) __asm__ volatile("nop"); }
static inline void WaitVBlankIn(void) { while ((VDP2_TVSTAT & 8) == 0); }
static inline void WaitVBlankOut(void) { while ((VDP2_TVSTAT & 8) != 0); }

// Inicializar SMPC para leitura de controle
static void InitSMPC(void) {
    SMPC_DDR1 = 0x60;  // TH/TR como output
    SMPC_DDR2 = 0x60;
    SMPC_IOSEL = 0x03; // Direct mode
    SMPC_EXLE = 0x00;
}

// Ler controle do Player 1 (método direto)
static uint16_t ReadPad(void) {
    uint16_t data = 0xFFFF;
    
    // TH=1, TR=1
    SMPC_PDR1 = 0x60;
    delay(10);
    data = (SMPC_PDR1 & 0x0F) << 12;  // R L D U
    
    // TH=0, TR=1 
    SMPC_PDR1 = 0x20;
    delay(10);
    data |= (SMPC_PDR1 & 0x0F) << 8;  // Start A C B
    
    // TH=0, TR=0
    SMPC_PDR1 = 0x00;
    delay(10);
    data |= (SMPC_PDR1 & 0x0F) << 4;  // R X Y Z
    
    // TH=1, TR=0
    SMPC_PDR1 = 0x40;
    delay(10);
    data |= (SMPC_PDR1 & 0x0F);       // L (novamente)
    
    return ~data;  // Inverter (active high)
}

// Atualizar estado do controle
static void UpdatePad(void) {
    padPrev = padData;
    padData = ReadPad();
    padTrigger = padData & ~padPrev;  // Detectar borda de subida
}

// Interpolação usando Op_Ajuste
static inline int16_t lerp(int16_t a, int16_t b, int16_t t) {
    return (int16_t)Op_Ajuste(a, b, t, 7);
}

// ============================================================================
// CALCULAR ENERGIAS DO SISTEMA (LAGRANGIANO 4D COMPLETO)
// L = T - V onde T = ½Iω² para rotação, V = potencial gravitacional 4D
// ============================================================================
static void CalculateSystemEnergies(int32_t dt) {
    systemKineticEnergy = 0;
    systemPotentialEnergy = 0;
    systemMomentum4D = 0;
    
    const int32_t mass = 16;       // Massa por vértice
    const int32_t gravity = 10;    // Constante gravitacional
    const int32_t inertia = 64;    // Momento de inércia
    
    // ================================================================
    // ENERGIA CINÉTICA ROTACIONAL: T = ½Iω²
    // 6 planos de rotação em 4D
    // ================================================================
    int32_t omegaSq = (omegaXY * omegaXY + omegaXZ * omegaXZ + omegaXW * omegaXW +
                       omegaYZ * omegaYZ + omegaYW * omegaYW + omegaZW * omegaZW);
    int32_t T_rotational = (inertia * omegaSq) >> 9;  // ½Iω²
    
    // ================================================================
    // ENERGIA DOS VÉRTICES
    // ================================================================
    for (int i = 0; i < MAX_VERTS; i++) {
        // Magnitude 4D do vértice usando Op_Hipotenusa
        int32_t r4D = Op_Hipotenusa(morphedVerts[i].x, morphedVerts[i].y, 
                                     morphedVerts[i].z, morphedVerts[i].w);
        
        // Velocidade = ω × r (aproximação para rotação)
        int32_t v = (r4D * (omegaXY + omegaXW)) >> 8;
        
        // T = ½mv² para cada vértice
        int32_t T_vertex = (mass * v * v) >> 9;
        systemKineticEnergy += T_vertex;
        
        // V = mgh (Y como altura, W como "altura 4D")
        // Usar Op_Saturacao para limitar altura
        int32_t height = Op_Saturacao(morphedVerts[i].y + (morphedVerts[i].w >> 1) + 100, 0, 300);
        int32_t V = Lagrangian_PotentialEnergy(mass, height, gravity);
        systemPotentialEnergy += V;
        
        // Momento 4D: p = mv
        systemMomentum4D += Lagrangian_Momentum(mass, v);
    }
    
    systemKineticEnergy += T_rotational;
    
    // ================================================================
    // LAGRANGIANO E HAMILTONIANO
    // ================================================================
    systemLagrangian = Lagrangian_L(systemKineticEnergy, systemPotentialEnergy);
    systemHamiltonian = Lagrangian_Hamiltonian(systemKineticEnergy, systemPotentialEnergy);
    
    // ================================================================
    // AÇÃO: S = ∫L dt (usando Op_Cronos para scaling temporal)
    // ================================================================
    int32_t dAction = Op_Cronos(systemLagrangian, dt, 6);
    systemAction += dAction;
    
    // Manter ação limitada usando Op_Loop
    systemAction = Op_Loop(systemAction, 1000000);
}

// ============================================================================
// ROTAÇÃO 4D COMPLETA + PROJEÇÃO PERSPECTIVA
// 
// Em 4D existem 6 planos de rotação independentes (C(4,2) = 6):
//   1. XY - rotação no plano XY
//   2. XZ - rotação no plano XZ  
//   3. XW - rotação no plano XW (4D!)
//   4. YZ - rotação no plano YZ
//   5. YW - rotação no plano YW (4D!)
//   6. ZW - rotação no plano ZW (4D!)
//
// Cada rotação usa matriz 4x4 com rotação em seu respectivo plano.
// A projeção 4D→3D usa perspectiva com W, depois 3D→2D com Z.
// ============================================================================
static void RotateAndProject4D(void) {
    // Pre-calcular senos e cossenos para os 6 planos
    int16_t sXY = fsin(rotXY), cXY = fcos(rotXY);
    int16_t sXZ = fsin(rotXZ), cXZ = fcos(rotXZ);
    int16_t sXW = fsin(rotXW), cXW = fcos(rotXW);
    int16_t sYZ = fsin(rotYZ), cYZ = fcos(rotYZ);
    int16_t sYW = fsin(rotYW), cYW = fcos(rotYW);
    int16_t sZW = fsin(rotZW), cZW = fcos(rotZW);
    
    for (int i = 0; i < MAX_VERTS; i++) {
        int32_t x = morphedVerts[i].x;
        int32_t y = morphedVerts[i].y;
        int32_t z = morphedVerts[i].z;
        int32_t w = morphedVerts[i].w;
        
        // ============================================================
        // ROTAÇÃO 1: Plano XY (como rotação Z em 3D)
        // ============================================================
        int32_t x1 = (x * cXY - y * sXY) >> 7;
        int32_t y1 = (x * sXY + y * cXY) >> 7;
        int32_t z1 = z;
        int32_t w1 = w;
        
        // ============================================================
        // ROTAÇÃO 2: Plano XZ (como rotação Y em 3D)
        // ============================================================
        int32_t x2 = (x1 * cXZ + z1 * sXZ) >> 7;
        int32_t y2 = y1;
        int32_t z2 = (-x1 * sXZ + z1 * cXZ) >> 7;
        int32_t w2 = w1;
        
        // ============================================================
        // ROTAÇÃO 3: Plano XW (4D - rotação na direção W!)
        // ============================================================
        int32_t x3 = (x2 * cXW - w2 * sXW) >> 7;
        int32_t y3 = y2;
        int32_t z3 = z2;
        int32_t w3 = (x2 * sXW + w2 * cXW) >> 7;
        
        // ============================================================
        // ROTAÇÃO 4: Plano YZ (como rotação X em 3D)
        // ============================================================
        int32_t x4 = x3;
        int32_t y4 = (y3 * cYZ - z3 * sYZ) >> 7;
        int32_t z4 = (y3 * sYZ + z3 * cYZ) >> 7;
        int32_t w4 = w3;
        
        // ============================================================
        // ROTAÇÃO 5: Plano YW (4D - rotação Y↔W!)
        // ============================================================
        int32_t x5 = x4;
        int32_t y5 = (y4 * cYW - w4 * sYW) >> 7;
        int32_t z5 = z4;
        int32_t w5 = (y4 * sYW + w4 * cYW) >> 7;
        
        // ============================================================
        // ROTAÇÃO 6: Plano ZW (4D - rotação Z↔W!)
        // ============================================================
        int32_t x6 = x5;
        int32_t y6 = y5;
        int32_t z6 = (z5 * cZW - w5 * sZW) >> 7;
        int32_t w6 = (z5 * sZW + w5 * cZW) >> 7;
        
        // ============================================================
        // PROJEÇÃO 4D → 3D (projeção perspectiva em W)
        // Similar à projeção 3D→2D, mas usando W como "profundidade 4D"
        // ============================================================
        const int32_t FOCAL_4D = 150;  // Distância focal 4D
        const int32_t W_OFFSET = 100;  // Offset para evitar divisão por zero
        
        int32_t wDist = w6 + W_OFFSET;
        wDist = Op_Saturacao(wDist, 30, 250);  // Clamp para segurança
        
        // Projetar 4D → 3D
        int32_t xProj = (x6 * FOCAL_4D) / wDist;
        int32_t yProj = (y6 * FOCAL_4D) / wDist;
        int32_t zProj = (z6 * FOCAL_4D) / wDist;
        
        // Guardar coordenadas 3D transformadas
        transformed[i].x = (int16_t)xProj;
        transformed[i].y = (int16_t)yProj;
        transformed[i].z = (int16_t)zProj;
        
        // ============================================================
        // PROJEÇÃO 3D → 2D (projeção perspectiva em Z)
        // ============================================================
        const int32_t FOCAL_3D = 200;  // Distância focal 3D
        const int32_t Z_OFFSET = 150;  // Offset Z
        
        int32_t zDist = zProj + Z_OFFSET;
        zDist = Op_Saturacao(zDist, 20, 400);  // Clamp para segurança
        
        // Projetar 3D → 2D para VDP1
        projected[i].x = (int16_t)((xProj * FOCAL_3D) / zDist);
        projected[i].y = (int16_t)((yProj * FOCAL_3D) / zDist);
    }
}

// Normal da face (para backface culling) - usa faces da forma atual
static int16_t FaceNormalZ(const uint8_t (*faces)[4], int face) {
    int i0 = faces[face][0];
    int i1 = faces[face][1];
    int i2 = faces[face][2];
    
    int16_t ax = projected[i1].x - projected[i0].x;
    int16_t ay = projected[i1].y - projected[i0].y;
    int16_t bx = projected[i2].x - projected[i0].x;
    int16_t by = projected[i2].y - projected[i0].y;
    
    return (ax * by - ay * bx);
}

// Profundidade média da face
static int16_t FaceDepth(const uint8_t (*faces)[4], int face) {
    int d = 0;
    for (int i = 0; i < 4; i++) {
        d += transformed[faces[face][i]].z;
    }
    return d >> 2;
}

// ============================================================================
// INICIALIZAÇÃO
// ============================================================================
static void InitHardware(void) {
    // SMPC (Controle)
    InitSMPC();
    
    // VDP2 - WIDESCREEN 352x224 (Hi-Res Horizontal)
    VDP2_TVMD = 0;
    delay(1000);
    VDP2_EXTEN = 0;
    VDP2_BGON = 0;
    VDP2_VRAM[0] = DARK_BLUE;
    VDP2_BKTAU = 0; VDP2_BKTAL = 0;
    VDP2_SPCTL = 0x0020;
    VDP2_PRISA = 0x0707; VDP2_PRISB = 0x0707;
    VDP2_PRISC = 0x0707; VDP2_PRISD = 0x0707;
    VDP2_PRINA = 0; VDP2_PRINB = 0;
    // TVMD: bit 0 = HRESO0 (1=352 pixels), bit 15 = display on
    VDP2_TVMD = 0x8001;  // 352x224 widescreen mode
    
    // VDP1 - Configurado para 352x224 widescreen
    VDP1_PTMR = 0; VDP1_ENDR = 0;
    delay(10000);
    VDP1_TVMR = 0; VDP1_FBCR = 0;
    VDP1_EWDR = DARK_BLUE;
    VDP1_EWLR = 0;
    // Erase Window: X=(43<<9)=352/8-1, Y=223
    VDP1_EWRR = (43 << 9) | 223;  // 352x224 erase area
    
    for (int i = 0; i < 512; i++) VDP1_VRAM[i] = 0;
    
    // Inicializar vértices com Tesseract 4D
    for (int i = 0; i < MAX_VERTS; i++) {
        morphedVerts[i] = VERTS_TESS[i];
        prevVerts[i] = VERTS_TESS[i];
        velocity[i] = {0, 0, 0, 0};
    }
    currentNumFaces = NFACES_TESS;
}

static void BuildCommands(void) {
    volatile VDP1Command* cmd = (volatile VDP1Command*)VDP1_VRAM;
    
    // System Clip (352x224 widescreen)
    cmd[0].ctrl = VDP1_CMD_SYSTEM_CLIP;
    cmd[0].xc = 351; cmd[0].yc = 223;  // 352-1=351
    
    // Local Coordinates (centro da tela widescreen 352x224)
    cmd[1].ctrl = VDP1_CMD_LOCAL_COORD;
    cmd[1].xa = 176; cmd[1].ya = 112;  // Centro: 352/2=176, 224/2=112
    
    // MAX_FACES comandos de polígono (reservar espaço)
    for (int i = 0; i < MAX_FACES; i++) {
        cmd[2 + i].ctrl = VDP1_CMD_POLYGON;
        cmd[2 + i].pmod = 0x00C0;
        cmd[2 + i].colr = SHAPE_COLORS[0];
    }
    
    // End
    cmd[2 + MAX_FACES].ctrl = VDP1_CMD_END;
}

// ============================================================================
// MAIN - TESSERACT 4D @ 240 FPS
// Hipercubo com rotação em 4 dimensões (XY, XZ, XW, YZ, YW, ZW)
// ============================================================================
int main(void) {
    InitHardware();
    BuildCommands();
    
    volatile VDP1Command* cmd = (volatile VDP1Command*)VDP1_VRAM;
    uint32_t frame = 0;
    
    while (1) {
        WaitVBlankIn();
        VDP1_PTMR = 0x0001;
        WaitVBlankOut();
        
        // ================================================================
        // LER CONTROLE
        // ================================================================
        UpdatePad();
        
        // START: Alternar modo FPS (30 → 60 → 120 → 240 → 30...)
        if (padTrigger & PAD_START) {
            fpsMode = (fpsMode + 1) & 3;
        }
        
        // A/B/C/X: Selecionar FPS diretamente
        if (padTrigger & PAD_A) fpsMode = 0;  // 30 FPS
        if (padTrigger & PAD_B) fpsMode = 1;  // 60 FPS  
        if (padTrigger & PAD_C) fpsMode = 2;  // 120 FPS
        if (padTrigger & PAD_X) fpsMode = 3;  // 240 FPS
        
        // Número de sub-frames baseado no modo
        int numSubFrames = FPS_MODES[fpsMode];
        
        // ================================================================
        // UPDATES DE LÓGICA (30/60/120/240 FPS)
        // ================================================================
        for (int subFrame = 0; subFrame < numSubFrames; subFrame++) {
            // Calcular velocidade para física (baseado em rotação)
            for (int i = 0; i < MAX_VERTS; i++) {
                prevVerts[i] = morphedVerts[i];
                // Tesseract permanece estático (rotação é aplicada na projeção)
                velocity[i].x = 0;
                velocity[i].y = 0;
                velocity[i].z = 0;
                velocity[i].w = 0;
            }
        }
        
        // ================================================================
        // FÍSICA LAGRANGIANA 4D (L = T - V)
        // 
        // Coordenadas generalizadas: 6 ângulos de rotação 4D
        // q = (θXY, θXZ, θXW, θYZ, θYW, θZW)
        // Velocidades generalizadas: ω = dq/dt
        // T = ½Iω² (energia cinética rotacional)
        // ================================================================
        
        // Calcular energias com delta time baseado no FPS
        int32_t dt = 16 / numSubFrames;  // ~16ms / subframes
        CalculateSystemEnergies(dt);
        
        // ================================================================
        // ATUALIZAR VELOCIDADES ANGULARES (ω = dθ/dt)
        // Usando o princípio de mínima ação: equações de Euler-Lagrange
        // d/dt(∂L/∂ω) - ∂L/∂θ = 0
        //
        // Para sistema rotacional livre sem potencial angular:
        // ω permanece constante (momento angular conservado)
        // ================================================================
        
        // Velocidades angulares base (radianos/frame em fixed point)
        int32_t omegaBase = 2 + (systemKineticEnergy >> 10);
        omegaBase = Op_Saturacao(omegaBase, 1, 8);
        
        // Modulação por energia do sistema (acoplamento físico)
        int32_t energyMod = Op_Reflexao(systemHamiltonian >> 8, 16);
        
        // Diferentes velocidades para cada plano (padrões interessantes)
        omegaXY = omegaBase + (energyMod >> 2);      // Rotação principal
        omegaXZ = (omegaBase * 2) / 3;               // Rotação secundária
        omegaXW = omegaBase / 2 + (energyMod >> 3);  // Rotação 4D lenta
        omegaYZ = (omegaBase * 3) / 4;               // Rotação terciária
        omegaYW = omegaBase / 3;                     // Rotação 4D média
        omegaZW = omegaBase / 4;                     // Rotação 4D lenta
        
        // ================================================================
        // ATUALIZAR ÂNGULOS DE ROTAÇÃO (θ += ω·dt)
        // Integração de Euler para simplificar
        // ================================================================
        rotXY = Op_Loop(rotXY + omegaXY, 256);
        rotXZ = Op_Loop(rotXZ + omegaXZ, 256);
        rotXW = Op_Loop(rotXW + omegaXW, 256);
        rotYZ = Op_Loop(rotYZ + omegaYZ, 256);
        rotYW = Op_Loop(rotYW + omegaYW, 256);
        rotZW = Op_Loop(rotZW + omegaZW, 256);
        
        // ================================================================
        // TOPOLOGIA 4D DO TESSERACT
        // V=16, E=32, F=24, C=8 → χ = 16-32+24-8 = 0
        // ================================================================
        int32_t numVerts = NVERTS_TESS;
        int32_t numEdges = NEDGES_TESS;
        int32_t numCells = NCELLS_TESS;
        
        // Euler-Poincaré 4D: χ = V - E + F - C + H (H=hiperfaces)
        int32_t eulerChi4D = Topology_EulerChar(numVerts, numEdges, currentNumFaces);
        eulerChi4D -= numCells;  // Ajuste 4D: subtrair células
        
        // Genus e curvatura para modulação de cor
        int32_t genus = Topology_Genus(eulerChi4D + 2);  // Ajustar para 3D equiv
        int32_t curvature = Topology_GaussianCurvature(NFACES_TESS >> 1, 64);
        
        // Acumular Ação (Princípio de Mínima Ação: δS = 0)
        Lagrangian_AccumulateAction(systemLagrangian, dt);
        
        // Força Generalizada (para modular velocidades futuras)
        int32_t force4D = Lagrangian_Force(
            systemPotentialEnergy,
            systemPotentialEnergy + (morphedVerts[0].y >> 2),
            Op_Hipotenusa(morphedVerts[0].x, morphedVerts[0].y, morphedVerts[0].z, morphedVerts[0].w) + 1
        );
        
        // Usar força para modular velocidade angular
        omegaXY += Op_Saturacao(force4D >> 12, -2, 2);
        omegaXY = Op_Saturacao(omegaXY, 1, 16);
        
        // ================================================================
        // PROJEÇÃO 4D → 2D COM ROTAÇÃO EM 6 PLANOS
        // ================================================================
        RotateAndProject4D();
        
        // ================================================================
        // TESSERACT: 24 FACES
        // ================================================================
        const uint8_t (*currentFaces)[4] = FACES_TESS;
        int numFacesToRender = NFACES_TESS;
        
        // ================================================================
        // ORDENAR FACES POR PROFUNDIDADE
        // ================================================================
        int8_t order[MAX_FACES];
        int16_t depths[MAX_FACES];
        
        for (int i = 0; i < numFacesToRender; i++) {
            order[i] = i;
            depths[i] = FaceDepth(currentFaces, i);
        }
        
        // Bubble sort (ordenar do mais longe ao mais perto)
        for (int i = 0; i < numFacesToRender - 1; i++) {
            for (int j = i + 1; j < numFacesToRender; j++) {
                if (depths[i] > depths[j]) {
                    int16_t td = depths[i]; depths[i] = depths[j]; depths[j] = td;
                    int8_t to = order[i]; order[i] = order[j]; order[j] = to;
                }
            }
        }
        
        // ================================================================
        // RENDERIZAR FACES
        // ================================================================
        int cmdIdx = 2;
        int16_t screenMinX = -176, screenMaxX = 176;  // Widescreen 352/2=176
        
        // Cor base do Tesseract (Magenta 4D)
        uint16_t baseColor = SHAPE_COLORS[4];  // Tesseract = Magenta
        
        for (int i = 0; i < numFacesToRender; i++) {
            int face = order[i];
            int16_t nz = FaceNormalZ(currentFaces, face);
            
            // Backface culling
            if (nz >= 0) continue;
            
            int i0 = currentFaces[face][0];
            int i1 = currentFaces[face][1];
            int i2 = currentFaces[face][2];
            int i3 = currentFaces[face][3];
            
            // Verificar visibilidade na tela
            int16_t faceMinX = projected[i0].x;
            int16_t faceMaxX = projected[i0].x;
            for (int v = 1; v < 4; v++) {
                int16_t px = projected[currentFaces[face][v]].x;
                if (px < faceMinX) faceMinX = px;
                if (px > faceMaxX) faceMaxX = px;
            }
            
            if (!Op_Intersecao(faceMinX, faceMaxX, screenMinX, screenMaxX)) continue;
            
            // Cor baseada na profundidade da face (gradiente 4D)
            int16_t depthShade = Op_Saturacao((depths[i] + 100) >> 3, 0, 15);
            uint8_t r = (baseColor & 0x1F);
            uint8_t g = (baseColor >> 5) & 0x1F;
            uint8_t b = (baseColor >> 10) & 0x1F;
            
            // Variar cor por face para distinguir cubos interno/externo/conexões
            // Usar genus e curvatura para modulação topológica
            int16_t genusShade = Op_Saturacao(genus, 0, 8);
            int16_t curveShade = Op_Saturacao(curvature >> 8, 0, 8);
            
            if (face < 6) {           // Cubo interno (w=-25)
                r = 31; g = 8 + depthShade + genusShade; b = 8 + curveShade;
            } else if (face < 12) {   // Cubo externo (w=+25)
                r = 8 + curveShade; g = 8 + depthShade; b = 31;
            } else {                  // Conexões 4D
                r = 31; g = 31 - genusShade; b = 8 + depthShade;
            }
            
            // Shading baseado em física
            int16_t energyShade = Op_Saturacao((systemHamiltonian >> 8) + 8, 0, 15);
            r = Op_Saturacao(r + (energyShade >> 2), 8, 31);
            g = Op_Saturacao(g + (energyShade >> 2), 8, 31);
            b = Op_Saturacao(b + (energyShade >> 2), 8, 31);
            
            cmd[cmdIdx].ctrl = VDP1_CMD_POLYGON;
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
        
        // Terminar lista
        cmd[cmdIdx].ctrl = VDP1_CMD_END;
        
        // Usar Op_Loop para ciclar frame
        frame = Op_Loop(frame + 1, 65536);
    }
    
    return 0;
}
