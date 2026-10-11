#include "play_state.h"

namespace {
SemaphoreHandle_t s_mtx = nullptr;
PlayShot s_shot;
PlaySong s_song;
volatile uint32_t s_rev = 1;
}  // namespace

namespace play {

void begin() { s_mtx = xSemaphoreCreateRecursiveMutex(); }
void lock() { xSemaphoreTakeRecursive(s_mtx, portMAX_DELAY); }
void unlock() { xSemaphoreGiveRecursive(s_mtx); }
PlayShot& shot() { return s_shot; }
PlaySong& song() { return s_song; }
void touch() { s_rev++; }
uint32_t revision() { return s_rev; }

}  // namespace play
