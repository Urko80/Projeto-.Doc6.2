#pragma once

/**
 * DOC INÍCIO — controlador de movimento do Projeto .DOC
 *
 * Camada acima do PCA9685 (web_servo_api.cc). Usada por duas entradas:
 *   - comandos de voz (ferramentas MCP registradas em doc_ai_s3cam.cc)
 *   - painel web (endpoints /api/mode, /api/move, /api/tune em web_servo_api.cc)
 *
 * Modos:
 *   kWheels: as rodas (MG90S contínuos, CH1/CH2) andam/giram.
 *   kLegs:   as pernas (SG90, CH0/CH3) inclinam o corpo de um lado para o outro
 *            e a roda do pé apoiado dá um leve impulso (passo arrastado).
 *
 * Segurança:
 *   - todo movimento tem duração máxima e para sozinho;
 *   - o watchdog das rodas em web_servo_api.cc continua ativo;
 *   - limites de pulso são validados no firmware.
 *
 * NÃO compilado nem validado na placa: testar com as rodas suspensas.
 * DOC FIM
 */

#include <cstdint>
#include <functional>

#include <esp_err.h>

namespace doc_motion {

enum class Mode : uint8_t { kWheels = 0, kLegs = 1 };
enum class Dir : uint8_t { kStop = 0, kForward, kBackward, kLeft, kRight, kSettle /* interno: só muda de postura */ };

struct WheelTune {
    uint16_t stop_us;    // pulso de parada (neutro)
    uint16_t cw_us;      // pulso horário
    uint16_t ccw_us;     // pulso anti-horário
    bool forward_is_cw;  // true: "frente" desta roda = horário
    uint8_t trim_pct;    // 50..150: ajuste fino de velocidade desta roda
};

struct Tuning {
    WheelTune left;
    WheelTune right;
    uint8_t speed_pct;             // 10..100
    uint16_t leg_center_left_us;   // centro da perna esquerda (CH0)
    uint16_t leg_center_right_us;  // centro da perna direita (CH3)
    uint16_t leg_amplitude_us;     // 20..250 (centro +/- amplitude deve caber em 1250..1750)
    bool leg_left_inverted;
    bool leg_right_inverted;
    uint16_t step_hold_ms;         // 150..1500: tempo em cada lado
    uint16_t push_us;              // 10..120: deslocamento da roda no impulso
    uint16_t push_ms;              // 50..600: duração do impulso
    uint16_t obstacle_mm;          // 0 (desligado) ou 30..800
    uint16_t leg_sit_left_us;      // perna esquerda na postura SENTADO (Modo Rodas)
    uint16_t leg_sit_right_us;     // perna direita na postura SENTADO (Modo Rodas)
    uint16_t posture_ms;           // 300..5000: duração da transição em pé <-> sentado
};

using DistanceFn = std::function<esp_err_t(uint16_t* distance_mm)>;

/** Carrega ajustes (NVS), cria a task de movimento. Idempotente. */
esp_err_t Start();
/** Para o movimento e encerra a task. */
void Shutdown();

esp_err_t SetMode(Mode mode);
Mode GetMode();

/** Move na direção pelo tempo pedido (limitado). Repetir a mesma direção estende o prazo. */
esp_err_t Move(Dir dir, uint32_t duration_ms);
/** Para tudo imediatamente (rodas neutras; pernas voltam ao centro e seguram). */
void Stop();
bool IsBusy();
bool IsReady();

Tuning DefaultTuning();
Tuning GetTuning();
/** Valida, aplica e salva em NVS. ESP_ERR_INVALID_ARG se algum valor sair dos limites. */
esp_err_t SetTuning(const Tuning& tuning);
esp_err_t ResetTuning();

/** Leitor de distância (VL53L0X) usado para parar as rodas indo para frente. */
void SetDistanceProvider(DistanceFn fn);

/** "stand", "sit", "moving" ou "unknown" (servos das pernas soltos / recém-ligado). */
const char* PostureName();
/** Avisa que as pernas foram movidas/soltas por fora (endpoints brutos): posição passa a "desconhecida". */
void InvalidateLegPosition();

const char* ModeName(Mode mode);
const char* DirName(Dir dir);
bool ParseMode(const char* text, Mode* mode);
bool ParseDir(const char* text, Dir* dir);

constexpr uint32_t kMaxWheelMoveMs = 5000;
constexpr uint32_t kMaxLegMoveMs = 10000;

}  // namespace doc_motion
