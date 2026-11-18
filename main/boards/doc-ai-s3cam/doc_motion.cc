/**
 * DOC INÍCIO — controlador de movimento (ver doc_motion.h).
 *
 * Uma task a ~50 Hz executa o comando atual. Quem pede movimento (voz ou web)
 * só grava um comando (direção + prazo); a task faz a escrita no PCA9685 pelas
 * funções DocServo*() de web_servo_api.cc, que usam o mesmo mutex e mantêm o
 * watchdog das rodas atualizado.
 *
 * Marcha das pernas (Modo Pernas), por direção:
 *   - A cada "meio passo" (step_hold_ms) o corpo inclina para um lado
 *     (lado apoiado), alternando direita/esquerda.
 *   - Depois de assentar a inclinação, a roda do pé apoiado recebe um leve
 *     impulso (push_us por push_ms):
 *       frente/trás: roda apoiada gira no sentido da marcha;
 *       esquerda:    apoio direito -> roda direita para frente;
 *                    apoio esquerdo -> roda esquerda para trás;
 *       direita:     apoio esquerdo -> roda esquerda para frente;
 *                    apoio direito -> roda direita para trás.
 *   Esta sequência é uma PRIMEIRA VERSÃO baseada na descrição do projeto
 *   (rodas dão um leve impulso para girar o corpo). Amplitude, tempos e
 *   inversões são ajustáveis no painel e precisam ser calibrados no robô.
 * DOC FIM
 */

#include "doc_motion.h"

#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <cstring>

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "config.h"
#include "settings.h"
#include "web_servo_api.h"

namespace doc_motion {
namespace {

constexpr const char* TAG = "doc_motion";
constexpr uint32_t kTickMs = 20;
constexpr uint32_t kRefreshMs = 200;
constexpr uint32_t kLeadInMs = 150;
constexpr uint32_t kObstaclePollMs = 150;
constexpr int kWheelPulseMin = 1300;
constexpr int kWheelPulseMax = 1900;

struct Command {
    Dir dir = Dir::kStop;
    uint32_t seq = 0;
    uint32_t refresh_ms = 0;   // instante do último pedido
    uint32_t duration_ms = 0;  // duração pedida (o movimento acaba em refresh_ms + duration_ms)
};

SemaphoreHandle_t s_lock = nullptr;  // criado uma vez, nunca destruído
Command s_cmd;
Mode s_mode = Mode::kWheels;
Tuning s_tune = {};
DistanceFn s_distance_fn;
std::atomic<bool> s_busy{false};
std::atomic<bool> s_quit{false};
std::atomic<bool> s_task_running{false};
// Último pulso escrito nas pernas (-1 = desconhecido). Usado para a transição suave de postura.
std::atomic<int> s_leg_cur_l{-1};
std::atomic<int> s_leg_cur_r{-1};

uint32_t NowMs() {
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

class Guard {
public:
    Guard() : ok_(s_lock != nullptr && xSemaphoreTake(s_lock, pdMS_TO_TICKS(200)) == pdTRUE) {}
    ~Guard() {
        if (ok_) xSemaphoreGive(s_lock);
    }
    Guard(const Guard&) = delete;
    Guard& operator=(const Guard&) = delete;
    bool ok() const { return ok_; }

private:
    bool ok_;
};

bool InRange(int value, int lo, int hi) {
    return value >= lo && value <= hi;
}

bool ValidWheel(const WheelTune& w) {
    return InRange(w.stop_us, kWheelPulseMin, kWheelPulseMax) &&
           InRange(w.cw_us, kWheelPulseMin, kWheelPulseMax) &&
           InRange(w.ccw_us, kWheelPulseMin, kWheelPulseMax) &&
           InRange(w.trim_pct, 50, 150);
}

bool Validate(const Tuning& t) {
    if (!ValidWheel(t.left) || !ValidWheel(t.right)) return false;
    if (!InRange(t.speed_pct, 10, 100)) return false;
    if (!InRange(t.leg_amplitude_us, 20, 250)) return false;
    const int amp = t.leg_amplitude_us;
    const int cl = t.leg_center_left_us;
    const int cr = t.leg_center_right_us;
    if (cl - amp < DOC_LEG_ABS_MIN_US || cl + amp > DOC_LEG_ABS_MAX_US) return false;
    if (cr - amp < DOC_LEG_ABS_MIN_US || cr + amp > DOC_LEG_ABS_MAX_US) return false;
    if (!InRange(t.leg_sit_left_us, DOC_LEG_ABS_MIN_US, DOC_LEG_ABS_MAX_US)) return false;
    if (!InRange(t.leg_sit_right_us, DOC_LEG_ABS_MIN_US, DOC_LEG_ABS_MAX_US)) return false;
    if (!InRange(t.posture_ms, 300, 5000)) return false;
    if (!InRange(t.step_hold_ms, 150, 1500)) return false;
    if (!InRange(t.push_us, 10, 120)) return false;
    if (!InRange(t.push_ms, 50, 600)) return false;
    if (t.obstacle_mm != 0 && !InRange(t.obstacle_mm, 30, 800)) return false;
    return true;
}

// ---- persistência (NVS, namespace "doc_motion") ------------------------------------------

void SaveTuning(const Tuning& t) {
    Settings s("doc_motion", true);
    s.SetInt("spd", t.speed_pct);
    s.SetInt("wl_stop", t.left.stop_us);
    s.SetInt("wl_cw", t.left.cw_us);
    s.SetInt("wl_ccw", t.left.ccw_us);
    s.SetBool("wl_fcw", t.left.forward_is_cw);
    s.SetInt("wl_trim", t.left.trim_pct);
    s.SetInt("wr_stop", t.right.stop_us);
    s.SetInt("wr_cw", t.right.cw_us);
    s.SetInt("wr_ccw", t.right.ccw_us);
    s.SetBool("wr_fcw", t.right.forward_is_cw);
    s.SetInt("wr_trim", t.right.trim_pct);
    s.SetInt("lc_l", t.leg_center_left_us);
    s.SetInt("lc_r", t.leg_center_right_us);
    s.SetInt("l_amp", t.leg_amplitude_us);
    s.SetBool("l_inv_l", t.leg_left_inverted);
    s.SetBool("l_inv_r", t.leg_right_inverted);
    s.SetInt("hold", t.step_hold_ms);
    s.SetInt("push_us", t.push_us);
    s.SetInt("push_ms", t.push_ms);
    s.SetInt("obst", t.obstacle_mm);
    s.SetInt("sit_l", t.leg_sit_left_us);
    s.SetInt("sit_r", t.leg_sit_right_us);
    s.SetInt("post_ms", t.posture_ms);
}

Tuning LoadTuning() {
    Tuning t = DefaultTuning();
    Settings s("doc_motion", false);
    t.speed_pct = static_cast<uint8_t>(s.GetInt("spd", t.speed_pct));
    t.left.stop_us = static_cast<uint16_t>(s.GetInt("wl_stop", t.left.stop_us));
    t.left.cw_us = static_cast<uint16_t>(s.GetInt("wl_cw", t.left.cw_us));
    t.left.ccw_us = static_cast<uint16_t>(s.GetInt("wl_ccw", t.left.ccw_us));
    t.left.forward_is_cw = s.GetBool("wl_fcw", t.left.forward_is_cw);
    t.left.trim_pct = static_cast<uint8_t>(s.GetInt("wl_trim", t.left.trim_pct));
    t.right.stop_us = static_cast<uint16_t>(s.GetInt("wr_stop", t.right.stop_us));
    t.right.cw_us = static_cast<uint16_t>(s.GetInt("wr_cw", t.right.cw_us));
    t.right.ccw_us = static_cast<uint16_t>(s.GetInt("wr_ccw", t.right.ccw_us));
    t.right.forward_is_cw = s.GetBool("wr_fcw", t.right.forward_is_cw);
    t.right.trim_pct = static_cast<uint8_t>(s.GetInt("wr_trim", t.right.trim_pct));
    t.leg_center_left_us = static_cast<uint16_t>(s.GetInt("lc_l", t.leg_center_left_us));
    t.leg_center_right_us = static_cast<uint16_t>(s.GetInt("lc_r", t.leg_center_right_us));
    t.leg_amplitude_us = static_cast<uint16_t>(s.GetInt("l_amp", t.leg_amplitude_us));
    t.leg_left_inverted = s.GetBool("l_inv_l", t.leg_left_inverted);
    t.leg_right_inverted = s.GetBool("l_inv_r", t.leg_right_inverted);
    t.step_hold_ms = static_cast<uint16_t>(s.GetInt("hold", t.step_hold_ms));
    t.push_us = static_cast<uint16_t>(s.GetInt("push_us", t.push_us));
    t.push_ms = static_cast<uint16_t>(s.GetInt("push_ms", t.push_ms));
    t.obstacle_mm = static_cast<uint16_t>(s.GetInt("obst", t.obstacle_mm));
    t.leg_sit_left_us = static_cast<uint16_t>(s.GetInt("sit_l", t.leg_sit_left_us));
    t.leg_sit_right_us = static_cast<uint16_t>(s.GetInt("sit_r", t.leg_sit_right_us));
    t.posture_ms = static_cast<uint16_t>(s.GetInt("post_ms", t.posture_ms));
    if (!Validate(t)) {
        ESP_LOGW(TAG, "Ajustes salvos inválidos; usando valores padrão");
        return DefaultTuning();
    }
    return t;
}

// ---- cálculo de pulsos --------------------------------------------------------------------

// sense: +1 = "frente" desta roda, -1 = "trás", 0 = parada.
uint16_t WheelPulse(const WheelTune& w, int sense, int speed_pct) {
    if (sense == 0) return w.stop_us;
    const bool cw = (sense > 0) ? w.forward_is_cw : !w.forward_is_cw;
    const int target = cw ? w.cw_us : w.ccw_us;
    int delta = target - static_cast<int>(w.stop_us);
    delta = delta * speed_pct * static_cast<int>(w.trim_pct) / 10000;
    return static_cast<uint16_t>(
        std::clamp(static_cast<int>(w.stop_us) + delta, kWheelPulseMin, kWheelPulseMax));
}

// Impulso curto: desloca push_us a partir do neutro, no lado do sentido pedido.
uint16_t PushPulse(const WheelTune& w, int sense, int push_us) {
    if (sense == 0) return w.stop_us;
    const bool cw = (sense > 0) ? w.forward_is_cw : !w.forward_is_cw;
    const int target = cw ? w.cw_us : w.ccw_us;
    const int diff = target - static_cast<int>(w.stop_us);
    const int sign = (diff > 0) - (diff < 0);
    if (sign == 0) return w.stop_us;
    return static_cast<uint16_t>(
        std::clamp(static_cast<int>(w.stop_us) + sign * push_us, kWheelPulseMin, kWheelPulseMax));
}

// ---- estado de execução (somente a task usa) ----------------------------------------------

enum class Phase : uint8_t { kPosture, kMotion };

struct Run {
    bool active = false;
    Dir dir = Dir::kStop;
    Mode mode = Mode::kWheels;
    Phase phase = Phase::kMotion;
    uint32_t cmd_duration_ms = 0;
    uint32_t min_end_ms = 0;        // fim mínimo do movimento (conta só depois da postura)
    uint32_t posture_start_ms = 0;
    uint32_t posture_ms = 0;
    int from_l = 0, from_r = 0, to_l = 0, to_r = 0;
    bool gait_started = false;      // true quando a marcha das pernas começou a escrever
    uint32_t start_ms = 0;
    uint32_t last_wheel_write_ms = 0;
    uint32_t last_obstacle_ms = 0;
    bool obstacle_polled = false;
    int last_wl = -1, last_wr = -1;
    int last_ll = -1, last_lr = -1;
    bool legs_touched = false;
};

bool WriteWheels(Run& r, int left_us, int right_us, uint32_t now) {
    if (left_us == r.last_wl && right_us == r.last_wr && (now - r.last_wheel_write_ms) < kRefreshMs) {
        return true;
    }
    const esp_err_t err = DocServoWheelsWrite(static_cast<uint16_t>(left_us), static_cast<uint16_t>(right_us));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao escrever rodas: %s", esp_err_to_name(err));
        return false;
    }
    r.last_wl = left_us;
    r.last_wr = right_us;
    r.last_wheel_write_ms = now;
    return true;
}

bool WriteLegs(Run& r, int left_us, int right_us) {
    if (left_us == r.last_ll && right_us == r.last_lr) return true;
    const esp_err_t err = DocServoLegsWrite(static_cast<uint16_t>(left_us), static_cast<uint16_t>(right_us));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao escrever pernas: %s", esp_err_to_name(err));
        return false;
    }
    r.last_ll = left_us;
    r.last_lr = right_us;
    r.legs_touched = true;
    s_leg_cur_l.store(left_us);
    s_leg_cur_r.store(right_us);
    return true;
}

// Rodas neutras; se as pernas foram usadas, volta ao centro e SEGURA (não solta o torque).
void EndMotion(Run& r, const Tuning& t) {
    DocServoWheelsWrite(t.left.stop_us, t.right.stop_us);
    r.last_wl = r.last_wr = -1;
    // Só a marcha volta ao centro. No Modo Rodas (sentado) ou numa troca de postura as pernas ficam onde estão.
    if (r.gait_started && r.legs_touched) {
        if (DocServoLegsWrite(t.leg_center_left_us, t.leg_center_right_us) == ESP_OK) {
            s_leg_cur_l.store(t.leg_center_left_us);
            s_leg_cur_r.store(t.leg_center_right_us);
        }
        r.last_ll = r.last_lr = -1;
    }
    r.active = false;
}

DistanceFn GetDistanceFn() {
    Guard g;
    return g.ok() ? s_distance_fn : DistanceFn();
}

// Retorna false para encerrar o movimento (obstáculo ou falha de escrita).
bool WheelsTick(Run& r, const Tuning& t, uint32_t now) {
    int sl = 0, sr = 0;
    switch (r.dir) {
        case Dir::kForward:  sl = +1; sr = +1; break;
        case Dir::kBackward: sl = -1; sr = -1; break;
        case Dir::kLeft:     sl = -1; sr = +1; break;
        case Dir::kRight:    sl = +1; sr = -1; break;
        default: break;
    }

    if (r.dir == Dir::kForward && t.obstacle_mm > 0 &&
        (!r.obstacle_polled || (now - r.last_obstacle_ms) >= kObstaclePollMs)) {
        r.obstacle_polled = true;
        r.last_obstacle_ms = now;
        DistanceFn fn = GetDistanceFn();
        uint16_t mm = 0;
        // mm == 0 é tratado como leitura inválida (evita parada falsa).
        if (fn && fn(&mm) == ESP_OK && mm != 0 && mm < t.obstacle_mm) {
            ESP_LOGW(TAG, "Obstáculo a %u mm (< %u): parando", mm, t.obstacle_mm);
            return false;
        }
    }

    return WriteWheels(r, WheelPulse(t.left, sl, t.speed_pct), WheelPulse(t.right, sr, t.speed_pct), now);
}

bool LegsTick(Run& r, const Tuning& t, uint32_t now) {
    r.gait_started = true;
    const uint32_t el = now - r.start_ms;
    int tau = 0;  // +1: corpo inclinado para a direita (apoio direito); -1: esquerda
    int sense_l = 0, sense_r = 0;

    if (el >= kLeadInMs) {
        const uint32_t u = el - kLeadInMs;
        const uint32_t hold = t.step_hold_ms;
        const uint32_t half = u / hold;
        const uint32_t in_half = u % hold;
        const bool right_loaded = (half % 2) == 0;
        tau = right_loaded ? +1 : -1;

        const uint32_t push_start = hold * 35 / 100;
        const uint32_t push_len = std::min<uint32_t>(t.push_ms, hold * 60 / 100);
        if (in_half >= push_start && in_half < push_start + push_len) {
            switch (r.dir) {
                case Dir::kForward:
                    if (right_loaded) sense_r = +1; else sense_l = +1;
                    break;
                case Dir::kBackward:
                    if (right_loaded) sense_r = -1; else sense_l = -1;
                    break;
                case Dir::kLeft:
                    if (right_loaded) sense_r = +1; else sense_l = -1;
                    break;
                case Dir::kRight:
                    if (right_loaded) sense_r = -1; else sense_l = +1;
                    break;
                default: break;
            }
        }
    }

    const int sign_l = t.leg_left_inverted ? -1 : +1;
    const int sign_r = t.leg_right_inverted ? -1 : +1;
    const int leg_l = static_cast<int>(t.leg_center_left_us) + sign_l * tau * static_cast<int>(t.leg_amplitude_us);
    const int leg_r = static_cast<int>(t.leg_center_right_us) + sign_r * tau * static_cast<int>(t.leg_amplitude_us);
    if (!WriteLegs(r, leg_l, leg_r)) return false;

    return WriteWheels(r, PushPulse(t.left, sense_l, t.push_us), PushPulse(t.right, sense_r, t.push_us), now);
}

// ---- postura: em pé (Modo Pernas) <-> sentado (Modo Rodas) ---------------------------------

void PostureTarget(Mode mode, const Tuning& t, int* l, int* r) {
    if (mode == Mode::kWheels) {
        *l = t.leg_sit_left_us;
        *r = t.leg_sit_right_us;
    } else {
        *l = t.leg_center_left_us;
        *r = t.leg_center_right_us;
    }
}

// Decide se o comando começa pela transição de postura ou direto pelo movimento.
void BeginRun(Run& r, const Tuning& t, uint32_t now) {
    int to_l = 0, to_r = 0;
    PostureTarget(r.mode, t, &to_l, &to_r);
    int cur_l = s_leg_cur_l.load();
    int cur_r = s_leg_cur_r.load();
    const bool unknown = cur_l < 0 || cur_r < 0;
    if (unknown) {
        // Recém-ligado ou servos soltos: assume o robô EM PÉ (ligue-o sempre em pé).
        if (r.mode == Mode::kLegs) {
            cur_l = to_l;
            cur_r = to_r;
        } else {
            cur_l = t.leg_center_left_us;
            cur_r = t.leg_center_right_us;
        }
    }
    r.to_l = to_l;
    r.to_r = to_r;
    r.from_l = cur_l;
    r.from_r = cur_r;
    if (!unknown && cur_l == to_l && cur_r == to_r) {
        r.phase = Phase::kMotion;
        r.start_ms = now;
        r.min_end_ms = now + r.cmd_duration_ms;
        return;
    }
    r.phase = Phase::kPosture;
    r.posture_start_ms = now;
    r.posture_ms = (cur_l == to_l && cur_r == to_r) ? 0 : t.posture_ms;
    ESP_LOGI(TAG, "Trocando de postura (%s): %d/%d -> %d/%d em %u ms",
             r.mode == Mode::kWheels ? "sentar" : "levantar", cur_l, cur_r, to_l, to_r,
             static_cast<unsigned>(r.posture_ms));
}

// Retorna false quando a troca de postura de um comando "só postura" termina (ou em falha).
bool PostureTick(Run& r, const Tuning& t, uint32_t now) {
    const uint32_t el = now - r.posture_start_ms;
    float p = (r.posture_ms == 0) ? 1.0f : static_cast<float>(el) / static_cast<float>(r.posture_ms);
    if (p > 1.0f) p = 1.0f;
    const float e = p * p * (3.0f - 2.0f * p);  // partida e chegada suaves
    const int l = r.from_l + static_cast<int>(static_cast<float>(r.to_l - r.from_l) * e);
    const int rr = r.from_r + static_cast<int>(static_cast<float>(r.to_r - r.from_r) * e);
    if (!WriteLegs(r, p >= 1.0f ? r.to_l : l, p >= 1.0f ? r.to_r : rr)) return false;
    // Rodas sempre paradas enquanto o corpo muda de postura.
    if (!WriteWheels(r, t.left.stop_us, t.right.stop_us, now)) return false;
    if (p < 1.0f) return true;
    if (r.dir == Dir::kSettle) return false;
    r.phase = Phase::kMotion;
    r.start_ms = now;
    r.min_end_ms = now + r.cmd_duration_ms;
    return true;
}

bool Snapshot(Command* cmd, Mode* mode, Tuning* tune) {
    Guard g;
    if (!g.ok()) return false;
    *cmd = s_cmd;
    *mode = s_mode;
    *tune = s_tune;
    return true;
}

void MotionTask(void*) {
    Run run;
    uint32_t seen_seq = 0;
    {
        Command c;
        Mode m;
        Tuning t;
        if (Snapshot(&c, &m, &t)) seen_seq = c.seq;
    }
    while (!s_quit.load()) {
        Command cmd;
        Mode mode;
        Tuning tune;
        if (Snapshot(&cmd, &mode, &tune)) {
            const uint32_t now = NowMs();

            if (cmd.seq != seen_seq) {
                seen_seq = cmd.seq;
                if (cmd.dir == Dir::kStop) {
                    if (run.active || run.legs_touched) EndMotion(run, tune);
                    run.active = false;
                    s_busy.store(false);
                } else {
                    run = Run();
                    run.active = true;
                    run.dir = cmd.dir;
                    run.mode = mode;
                    run.cmd_duration_ms = cmd.duration_ms;
                    BeginRun(run, tune, now);
                    ESP_LOGI(TAG, "Comando: modo=%s direção=%s", ModeName(mode), DirName(cmd.dir));
                    if (run.dir == Dir::kSettle && run.phase == Phase::kMotion) {
                        // Já está na postura certa: nada a fazer.
                        EndMotion(run, tune);
                        s_busy.store(false);
                    }
                }
            }

            if (run.active) {
                bool keep_going = true;
                if (run.phase == Phase::kPosture) {
                    keep_going = PostureTick(run, tune, now);
                } else {
                    // Fim = o maior entre (último pedido + duração) e (início do movimento + duração).
                    const uint32_t end_a = cmd.refresh_ms + cmd.duration_ms;
                    const uint32_t end_b = run.min_end_ms;
                    const uint32_t end = (static_cast<int32_t>(end_a - end_b) >= 0) ? end_a : end_b;
                    keep_going = static_cast<int32_t>(now - end) < 0;
                    if (keep_going) {
                        keep_going = (run.mode == Mode::kLegs) ? LegsTick(run, tune, now) : WheelsTick(run, tune, now);
                    }
                }
                if (!keep_going) {
                    EndMotion(run, tune);
                    s_busy.store(false);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(kTickMs));
    }

    s_task_running.store(false);
    vTaskDelete(nullptr);
}

}  // namespace

// ---- API pública ---------------------------------------------------------------------------

Tuning DefaultTuning() {
    Tuning t = {};
    t.left = {DOC_WHEEL_LEFT_STOP_US, DOC_WHEEL_LEFT_CW_US, DOC_WHEEL_LEFT_CCW_US,
              DOC_WHEEL_LEFT_FORWARD_IS_CW, 100};
    t.right = {DOC_WHEEL_RIGHT_STOP_US, DOC_WHEEL_RIGHT_CW_US, DOC_WHEEL_RIGHT_CCW_US,
               DOC_WHEEL_RIGHT_FORWARD_IS_CW, 100};
    t.speed_pct = DOC_MOTION_SPEED_PCT;
    t.leg_center_left_us = DOC_LEG_CENTER_LEFT_US;
    t.leg_center_right_us = DOC_LEG_CENTER_RIGHT_US;
    t.leg_amplitude_us = DOC_LEG_TILT_AMPLITUDE_US;
    t.leg_left_inverted = DOC_LEG_LEFT_INVERTED;
    t.leg_right_inverted = DOC_LEG_RIGHT_INVERTED;
    t.step_hold_ms = DOC_STEP_HOLD_MS;
    t.push_us = DOC_STEP_PUSH_US;
    t.push_ms = DOC_STEP_PUSH_MS;
    t.obstacle_mm = DOC_OBSTACLE_STOP_MM;
    t.leg_sit_left_us = DOC_LEG_SIT_LEFT_US;
    t.leg_sit_right_us = DOC_LEG_SIT_RIGHT_US;
    t.posture_ms = DOC_POSTURE_MS;
    return t;
}

esp_err_t Start() {
    if (s_task_running.load()) return ESP_OK;
    if (s_lock == nullptr) {
        s_lock = xSemaphoreCreateMutex();
        if (s_lock == nullptr) return ESP_ERR_NO_MEM;
    }

    const Tuning loaded = LoadTuning();
    {
        Guard g;
        if (!g.ok()) return ESP_ERR_TIMEOUT;
        s_tune = loaded;
        s_cmd.dir = Dir::kStop;
        s_cmd.seq++;
        s_cmd.refresh_ms = 0;
        s_cmd.duration_ms = 0;
    }
    s_busy.store(false);
    s_leg_cur_l.store(-1);  // posição das pernas desconhecida até o primeiro comando
    s_leg_cur_r.store(-1);
    DocServoSetWheelStop(loaded.left.stop_us, loaded.right.stop_us);

    s_quit.store(false);
    s_task_running.store(true);
    if (xTaskCreate(MotionTask, "doc_motion", 4096, nullptr, 5, nullptr) != pdPASS) {
        s_task_running.store(false);
        ESP_LOGE(TAG, "Falha ao criar a task de movimento");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Controlador de movimento iniciado");
    return ESP_OK;
}

void Shutdown() {
    if (!s_task_running.load()) return;
    Stop();
    s_quit.store(true);
    for (int i = 0; i < 50 && s_task_running.load(); ++i) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

bool IsReady() {
    return s_task_running.load() && DocServoIsReady();
}

bool IsBusy() {
    return s_busy.load();
}

Mode GetMode() {
    Guard g;
    return g.ok() ? s_mode : Mode::kWheels;
}

esp_err_t SetMode(Mode mode) {
    if (s_lock == nullptr) return ESP_ERR_INVALID_STATE;
    Stop();
    Guard g;
    if (!g.ok()) return ESP_ERR_TIMEOUT;
    s_mode = mode;
    if (IsReady()) {
        // Pede à task que mude a postura: Modo Rodas = senta; Modo Pernas = levanta.
        s_cmd.dir = Dir::kSettle;
        s_cmd.seq++;
        s_cmd.refresh_ms = NowMs();
        s_cmd.duration_ms = 0;
        s_busy.store(true);
    }
    return ESP_OK;
}

esp_err_t Move(Dir dir, uint32_t duration_ms) {
    if (dir == Dir::kStop) {
        Stop();
        return ESP_OK;
    }
    if (!IsReady()) return ESP_ERR_INVALID_STATE;

    Guard g;
    if (!g.ok()) return ESP_ERR_TIMEOUT;
    const uint32_t limit = (s_mode == Mode::kLegs) ? kMaxLegMoveMs : kMaxWheelMoveMs;
    duration_ms = std::clamp<uint32_t>(duration_ms, 100, limit);
    const uint32_t now = NowMs();
    if (s_busy.load() && s_cmd.dir == dir) {
        // Mesma direção em andamento: só estende o prazo (usado pelo "segurar para mover").
        s_cmd.refresh_ms = now;
        s_cmd.duration_ms = duration_ms;
    } else {
        s_cmd.dir = dir;
        s_cmd.seq++;
        s_cmd.refresh_ms = now;
        s_cmd.duration_ms = duration_ms;
        s_busy.store(true);
    }
    return ESP_OK;
}

void Stop() {
    if (s_lock == nullptr) return;
    Tuning t;
    {
        Guard g;
        if (!g.ok()) return;
        s_cmd.dir = Dir::kStop;
        s_cmd.seq++;
        t = s_tune;
    }
    // Parada imediata das rodas, sem esperar o próximo ciclo da task.
    if (DocServoIsReady()) {
        DocServoWheelsWrite(t.left.stop_us, t.right.stop_us);
    }
}

Tuning GetTuning() {
    Guard g;
    return g.ok() ? s_tune : DefaultTuning();
}

esp_err_t SetTuning(const Tuning& tuning) {
    if (!Validate(tuning)) return ESP_ERR_INVALID_ARG;
    if (s_lock == nullptr) return ESP_ERR_INVALID_STATE;
    {
        Guard g;
        if (!g.ok()) return ESP_ERR_TIMEOUT;
        s_tune = tuning;
    }
    SaveTuning(tuning);
    DocServoSetWheelStop(tuning.left.stop_us, tuning.right.stop_us);
    // Parado: reaplica o neutro novo para que o ajuste fino de "stop" seja visível na hora.
    if (!s_busy.load() && DocServoIsReady()) {
        DocServoWheelsWrite(tuning.left.stop_us, tuning.right.stop_us);
    }
    return ESP_OK;
}

esp_err_t ResetTuning() {
    return SetTuning(DefaultTuning());
}

void SetDistanceProvider(DistanceFn fn) {
    if (s_lock == nullptr) {
        s_lock = xSemaphoreCreateMutex();
    }
    Guard g;
    if (g.ok()) s_distance_fn = std::move(fn);
}

const char* PostureName() {
    const Tuning t = GetTuning();
    const int l = s_leg_cur_l.load();
    const int r = s_leg_cur_r.load();
    if (l < 0 || r < 0) return "unknown";
    if (l == t.leg_center_left_us && r == t.leg_center_right_us) return "stand";
    if (l == t.leg_sit_left_us && r == t.leg_sit_right_us) return "sit";
    return "moving";
}

void InvalidateLegPosition() {
    s_leg_cur_l.store(-1);
    s_leg_cur_r.store(-1);
}

const char* ModeName(Mode mode) {
    return mode == Mode::kLegs ? "legs" : "wheels";
}

const char* DirName(Dir dir) {
    switch (dir) {
        case Dir::kForward:  return "forward";
        case Dir::kBackward: return "backward";
        case Dir::kLeft:     return "left";
        case Dir::kRight:    return "right";
        case Dir::kSettle:   return "settle";
        default:             return "stop";
    }
}

bool ParseMode(const char* text, Mode* mode) {
    if (text == nullptr || mode == nullptr) return false;
    if (strcmp(text, "wheels") == 0) { *mode = Mode::kWheels; return true; }
    if (strcmp(text, "legs") == 0) { *mode = Mode::kLegs; return true; }
    return false;
}

bool ParseDir(const char* text, Dir* dir) {
    if (text == nullptr || dir == nullptr) return false;
    if (strcmp(text, "forward") == 0) { *dir = Dir::kForward; return true; }
    if (strcmp(text, "backward") == 0 || strcmp(text, "back") == 0) { *dir = Dir::kBackward; return true; }
    if (strcmp(text, "left") == 0) { *dir = Dir::kLeft; return true; }
    if (strcmp(text, "right") == 0) { *dir = Dir::kRight; return true; }
    if (strcmp(text, "stop") == 0) { *dir = Dir::kStop; return true; }
    return false;
}

}  // namespace doc_motion
