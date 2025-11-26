/**
 * @file test_polygon.cxx
 * @brief MORPHING 3D/4D com Mecânica Lagrangiana (L = T - V)
 * 
 * Cubo 3D rotaciona e se deforma em Pirâmide (e vice-versa)
 * Com física Lagrangiana real afetando a dinâmica
 * 
 * 7 Operadores do Protocolo de Dinâmica Computacional
 * + Mecânica Analítica/Lagrangiana (L = T - V)
 * + Topologia (Euler, Curvatura Gaussiana, Gênero)
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

// Energia Cinética: T = (1/2) * m * v²
static int32_t Lagrangian_KineticEnergy(int32_t mass, int32_t vx, int32_t vy, int32_t vz) {
    int32_t vSquared = (vx*vx + vy*vy + vz*vz) >> 4;
    return (mass * vSquared) >> 1;
}

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
// ESTRUTURAS 3D
// ============================================================================
struct Vec3 { int16_t x, y, z; };
struct Vec2 { int16_t x, y; };

struct VDP1Command {
    uint16_t ctrl, link, pmod, colr, srca, size;
    int16_t xa, ya, xb, yb, xc, yc, xd, yd;
    uint16_t grda, reserved;
};

// ============================================================================
// CUBO BASE (8 vértices)
// ============================================================================
static const Vec3 cubeBase[8] = {
    {-40, -40, -40}, { 40, -40, -40}, { 40,  40, -40}, {-40,  40, -40},
    {-40, -40,  40}, { 40, -40,  40}, { 40,  40,  40}, {-40,  40,  40},
};

// PIRÂMIDE TARGET (vértices superiores colapsam no topo)
static const Vec3 pyramidTarget[8] = {
    {  0, -70,   0}, {  0, -70,   0}, { 50,  40, -50}, {-50,  40, -50},
    {  0, -70,   0}, {  0, -70,   0}, { 50,  40,  50}, {-50,  40,  50},
};

// Vértices animados
static Vec3 morphedVerts[8];
static Vec3 prevVerts[8];  // Para calcular velocidade
static Vec3 velocity[8];   // Velocidade de cada vértice

// Vértices transformados e projetados
static Vec3 transformed[8];
static Vec2 projected[8];

// 6 faces do cubo
static const uint8_t cubeFaces[6][4] = {
    {0, 1, 2, 3}, {5, 4, 7, 6}, {4, 0, 3, 7},
    {1, 5, 6, 2}, {4, 5, 1, 0}, {3, 2, 6, 7},
};

// Cores das faces
static const uint16_t faceColors[6] = {
    RGB(31, 8, 8),   RGB(8, 31, 8),   RGB(8, 8, 31),
    RGB(31, 31, 8),  RGB(31, 8, 31),  RGB(8, 31, 31),
};

// Estado físico do sistema
static int32_t systemKineticEnergy = 0;
static int32_t systemPotentialEnergy = 0;
static int32_t systemLagrangian = 0;
static int32_t systemHamiltonian = 0;

// ============================================================================
// HELPERS
// ============================================================================
static inline void delay(volatile int n) { while (n-- > 0) __asm__ volatile("nop"); }
static inline void WaitVBlankIn(void) { while ((VDP2_TVSTAT & 8) == 0); }
static inline void WaitVBlankOut(void) { while ((VDP2_TVSTAT & 8) != 0); }

// Interpolação usando Op_Ajuste
static inline int16_t lerp(int16_t a, int16_t b, int16_t t) {
    return (int16_t)Op_Ajuste(a, b, t, 7);
}

// ============================================================================
// MORPHING: Cubo ↔ Pirâmide
// ============================================================================
static void MorphShape(int16_t t) {
    for (int i = 0; i < 8; i++) {
        // Salvar posição anterior para calcular velocidade
        prevVerts[i] = morphedVerts[i];
        
        // Interpolar entre cubo e pirâmide
        morphedVerts[i].x = lerp(cubeBase[i].x, pyramidTarget[i].x, t);
        morphedVerts[i].y = lerp(cubeBase[i].y, pyramidTarget[i].y, t);
        morphedVerts[i].z = lerp(cubeBase[i].z, pyramidTarget[i].z, t);
        
        // Calcular velocidade (para energia cinética)
        velocity[i].x = morphedVerts[i].x - prevVerts[i].x;
        velocity[i].y = morphedVerts[i].y - prevVerts[i].y;
        velocity[i].z = morphedVerts[i].z - prevVerts[i].z;
    }
}

// ============================================================================
// CALCULAR ENERGIAS DO SISTEMA (LAGRANGIANO)
// ============================================================================
static void CalculateSystemEnergies(int32_t gravity) {
    systemKineticEnergy = 0;
    systemPotentialEnergy = 0;
    
    const int32_t mass = 16;
    
    for (int i = 0; i < 8; i++) {
        // T = (1/2) * m * v²
        int32_t T = Lagrangian_KineticEnergy(mass, velocity[i].x, velocity[i].y, velocity[i].z);
        
        // V = m * g * h (Y como altura)
        int32_t V = Lagrangian_PotentialEnergy(mass, morphedVerts[i].y + 100, gravity);
        
        systemKineticEnergy += T;
        systemPotentialEnergy += V;
    }
    
    // L = T - V, H = T + V
    systemLagrangian = Lagrangian_L(systemKineticEnergy, systemPotentialEnergy);
    systemHamiltonian = Lagrangian_Hamiltonian(systemKineticEnergy, systemPotentialEnergy);
}

// ============================================================================
// ROTAÇÃO 3D + PROJEÇÃO PERSPECTIVA
// ============================================================================
static void RotateAndProject(uint8_t angleX, uint8_t angleY, uint8_t angleZ) {
    int16_t sx = fsin(angleX), cx = fcos(angleX);
    int16_t sy = fsin(angleY), cy = fcos(angleY);
    int16_t sz = fsin(angleZ), cz = fcos(angleZ);
    
    for (int i = 0; i < 8; i++) {
        int16_t x = morphedVerts[i].x;
        int16_t y = morphedVerts[i].y;
        int16_t z = morphedVerts[i].z;
        
        // Rotação Z
        int16_t x1 = (x * cz - y * sz) >> 7;
        int16_t y1 = (x * sz + y * cz) >> 7;
        
        // Rotação Y
        int16_t x2 = (x1 * cy + z * sy) >> 7;
        int16_t z2 = (-x1 * sy + z * cy) >> 7;
        
        // Rotação X
        int16_t y3 = (y1 * cx - z2 * sx) >> 7;
        int16_t z3 = (y1 * sx + z2 * cx) >> 7;
        
        transformed[i] = {x2, y3, z3};
        
        // Projeção perspectiva
        int16_t zOff = z3 + 150;
        zOff = Op_Saturacao(zOff, 10, 300);
        
        const int16_t FOCAL = 200;
        projected[i].x = (x2 * FOCAL) / zOff;
        projected[i].y = (y3 * FOCAL) / zOff;
    }
}

// Normal da face (para backface culling)
static int16_t FaceNormalZ(int face) {
    int i0 = cubeFaces[face][0];
    int i1 = cubeFaces[face][1];
    int i2 = cubeFaces[face][2];
    
    int16_t ax = projected[i1].x - projected[i0].x;
    int16_t ay = projected[i1].y - projected[i0].y;
    int16_t bx = projected[i2].x - projected[i0].x;
    int16_t by = projected[i2].y - projected[i0].y;
    
    return (ax * by - ay * bx);
}

// Profundidade média da face
static int16_t FaceDepth(int face) {
    int d = 0;
    for (int i = 0; i < 4; i++) {
        d += transformed[cubeFaces[face][i]].z;
    }
    return d >> 2;
}

// ============================================================================
// INICIALIZAÇÃO
// ============================================================================
static void InitHardware(void) {
    // VDP2
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
    VDP2_TVMD = 0x8000;
    
    // VDP1
    VDP1_PTMR = 0; VDP1_ENDR = 0;
    delay(10000);
    VDP1_TVMR = 0; VDP1_FBCR = 0;
    VDP1_EWDR = DARK_BLUE;
    VDP1_EWLR = 0;
    VDP1_EWRR = (39 << 9) | 223;
    
    for (int i = 0; i < 512; i++) VDP1_VRAM[i] = 0;
    
    // Inicializar vértices
    for (int i = 0; i < 8; i++) {
        morphedVerts[i] = cubeBase[i];
        prevVerts[i] = cubeBase[i];
        velocity[i] = {0, 0, 0};
    }
}

static void BuildCommands(void) {
    volatile VDP1Command* cmd = (volatile VDP1Command*)VDP1_VRAM;
    
    // System Clip
    cmd[0].ctrl = VDP1_CMD_SYSTEM_CLIP;
    cmd[0].xc = 319; cmd[0].yc = 223;
    
    // Local Coordinates (centro da tela)
    cmd[1].ctrl = VDP1_CMD_LOCAL_COORD;
    cmd[1].xa = 160; cmd[1].ya = 112;
    
    // 6 faces (inicialmente vazias)
    for (int i = 0; i < 6; i++) {
        cmd[2 + i].ctrl = VDP1_CMD_POLYGON;
        cmd[2 + i].pmod = 0x00C0;
        cmd[2 + i].colr = faceColors[i];
    }
    
    // End
    cmd[8].ctrl = VDP1_CMD_END;
}

// ============================================================================
// MAIN - MORPHING + ROTAÇÃO + FÍSICA LAGRANGIANA
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
        // MORFISMO: Cubo ↔ Pirâmide (usando seno para suavidade)
        // ================================================================
        uint8_t morphPhase = frame;
        int16_t morphT = (fsin(morphPhase) + 127) >> 1;  // 0 a 127
        morphT = Op_Saturacao(morphT, 0, 127);
        
        MorphShape(morphT);
        
        // ================================================================
        // FÍSICA LAGRANGIANA (L = T - V)
        // ================================================================
        int32_t gravity = 8 + (morphT >> 4);  // Gravidade aumenta com deformação
        CalculateSystemEnergies(gravity);
        
        // Acumular Ação (Princípio de Mínima Ação: δS = 0)
        Lagrangian_AccumulateAction(systemLagrangian, 4);
        
        // Calcular Momento Total
        int32_t totalMomentum = 0;
        for (int i = 0; i < 8; i++) {
            int32_t vMag = Op_Hipotenusa(velocity[i].x, velocity[i].y, velocity[i].z, 0);
            totalMomentum += Lagrangian_Momentum(16, vMag);
        }
        
        // Força Generalizada
        int32_t force = Lagrangian_Force(
            systemPotentialEnergy,
            systemPotentialEnergy + morphedVerts[0].y,
            Op_Hipotenusa(morphedVerts[0].x, morphedVerts[0].y, morphedVerts[0].z, 0) + 1
        );
        
        // ================================================================
        // TOPOLOGIA
        // ================================================================
        int32_t nFaces = 6 - (morphT >> 5);  // 6→4 faces durante morph
        nFaces = Op_Saturacao(nFaces, 4, 6);
        int32_t eulerChi = Topology_EulerChar(8, 12, nFaces);
        int32_t genus = Topology_Genus(eulerChi);
        int32_t curvature = Topology_GaussianCurvature(nFaces >> 1, 64);
        
        // ================================================================
        // ROTAÇÃO 3D (modulada por física)
        // ================================================================
        // Energia cinética afeta velocidade de rotação
        int32_t rotBoost = Op_Saturacao(systemKineticEnergy >> 6, 0, 32);
        rotBoost += (totalMomentum >> 8);
        rotBoost += Op_Reflexao(force >> 8, 16);  // Usar Op_Reflexao
        
        uint8_t angleX = frame + (rotBoost >> 2);
        uint8_t angleY = (frame * 2) / 3 + (genus << 2);
        uint8_t angleZ = frame / 2 + (curvature >> 6);
        
        RotateAndProject(angleX, angleY, angleZ);
        
        // ================================================================
        // ORDENAR FACES POR PROFUNDIDADE
        // ================================================================
        int8_t order[6] = {0, 1, 2, 3, 4, 5};
        int16_t depths[6];
        
        for (int i = 0; i < 6; i++) {
            depths[i] = FaceDepth(i);
        }
        
        // Bubble sort (ordenar do mais longe ao mais perto)
        for (int i = 0; i < 5; i++) {
            for (int j = i + 1; j < 6; j++) {
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
        
        // Usar Op_Intersecao para verificar visibilidade
        int16_t screenMinX = -160, screenMaxX = 160;
        
        for (int i = 0; i < 6; i++) {
            int face = order[i];
            int16_t nz = FaceNormalZ(face);
            
            // Backface culling (face visível se normal < 0)
            if (nz >= 0) continue;
            
            int i0 = cubeFaces[face][0];
            int i1 = cubeFaces[face][1];
            int i2 = cubeFaces[face][2];
            int i3 = cubeFaces[face][3];
            
            // Verificar se face está na tela usando Op_Intersecao
            int16_t faceMinX = projected[i0].x;
            int16_t faceMaxX = projected[i0].x;
            for (int v = 1; v < 4; v++) {
                int16_t px = projected[cubeFaces[face][v]].x;
                if (px < faceMinX) faceMinX = px;
                if (px > faceMaxX) faceMaxX = px;
            }
            
            if (!Op_Intersecao(faceMinX, faceMaxX, screenMinX, screenMaxX)) continue;
            
            // Cor base
            uint16_t baseColor = faceColors[face];
            uint8_t r = (baseColor & 0x1F);
            uint8_t g = (baseColor >> 5) & 0x1F;
            uint8_t b = (baseColor >> 10) & 0x1F;
            
            // Shading baseado em Hamiltoniano (energia total)
            int16_t energyShade = Op_Saturacao((systemHamiltonian >> 8) + 8, 0, 15);
            
            // Boost de cor durante morph (Lagrangiano)
            int16_t morphBoost = morphT >> 3;
            
            r = Op_Saturacao(r + morphBoost + (energyShade >> 2), 8, 31);
            g = Op_Saturacao(g + morphBoost + (energyShade >> 2), 8, 31);
            b = Op_Saturacao(b + morphBoost + (energyShade >> 2), 8, 31);
            
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
