// =============================================================================
// TOUCH INPUT HANDLER - OPTIMIZED FOR INSTANT WAKE
// =============================================================================
// Key optimizations:
// 1. Reduced wake debounce to 15ms for ~100ms total wake response
// 2. Edge-triggered immediate wake (no waiting for debounce)
// 3. Touch release tracking for proper tap detection
// 4. Interrupt-driven wake from light sleep
// =============================================================================

#include "touch.h"

#include <Arduino.h>
#include "../hardware_config.h"  // For TOUCH_INT_PIN
#include "../ui/ui_common.h"
#include "../ui/ui_answer.h"
#include "../system/state.h"
#include "../system/sleep.h"
#include "../ble/ble_core.h"
#include "../power/power_manager.h"

// -----------------------------------------------------------------------------
// Debounce Configuration
// -----------------------------------------------------------------------------
// Normal UI debounce - prevents accidental double-taps
constexpr uint32_t TOUCH_DEBOUNCE_MS = 30;  // Reduced from 35ms for faster response

// Wake debounce - minimal for instant wake response
// Target: Touch detected -> screen on in <100ms total
constexpr uint32_t WAKE_DEBOUNCE_MS = 15;   // Reduced from 20ms

// Minimum touch duration for UI actions (prevents noise)
constexpr uint32_t MIN_TOUCH_DURATION_MS = 10;

// Maximum time between touch down and up for a "tap" (vs. hold)
constexpr uint32_t TAP_MAX_DURATION_MS = 500;

// -----------------------------------------------------------------------------
// Touch State
// -----------------------------------------------------------------------------
static bool s_wasTouched = false;
static uint32_t s_touchDownMs = 0;
static bool s_pendingTouch = false;
static bool s_touchProcessed = false;  // Prevents multiple triggers per touch

void handleTouch() {
    // -------------------------------------------------------------------------
    // POWER: During sleep, check GPIO directly instead of I2C polling!
    // Touch INT pin (GPIO16) goes LOW when touched - no I2C needed for wake detect
    // -------------------------------------------------------------------------
    if (g_sleeping) {
        // Check touch interrupt pin directly (LOW = touched)
        if (digitalRead(TOUCH_INT_PIN) == LOW) {
            // Touch detected during sleep - trigger wake!
            powerMarkActivity();  // This sets g_wokeFromSleep = true
            g_ignoreTap = true;   // Consume this wake tap
        }
        return;  // POWER: Don't poll I2C during sleep - saves ~2-5mA!
    }

    const uint32_t now = millis();
    lgfx::touch_point_t tp;
    bool touched = gfx.getTouch(&tp);

    // -------------------------------------------------------------------------
    // EDGE DETECTION: Touch down
    // -------------------------------------------------------------------------
    if (touched && !s_wasTouched) {
        s_touchDownMs = now;
        s_pendingTouch = true;
        s_touchProcessed = false;

        // INSTANT WAKE: Don't wait for debounce when waking
        // The wake tap will be consumed (not forwarded to UI)
        if (g_sleeping) {
            powerMarkActivity();  // This sets g_wokeFromSleep = true
            g_ignoreTap = true;   // Consume this wake tap
            s_touchProcessed = true;
        }
        else if (g_dimmed) {
            // Immediate undim on touch edge
            markActivity();
            g_ignoreTap = true;
            s_touchProcessed = true;
        }
    }

    // -------------------------------------------------------------------------
    // EDGE DETECTION: Touch up (release)
    // -------------------------------------------------------------------------
    bool justReleased = !touched && s_wasTouched;  // Save before updating s_wasTouched
    if (justReleased) {
        s_pendingTouch = false;
        // Reset ignore flag on release so next touch works
        if (g_ignoreTap && !g_sleeping && !g_dimmed) {
            g_ignoreTap = false;
        }
    }

    // Update state
    s_wasTouched = touched;

    // -------------------------------------------------------------------------
    // DIMMED HANDLING: Exit early (touch edge already handled above)
    // -------------------------------------------------------------------------
    if (g_dimmed) {
        // Already handled immediate undim above
        return;
    }

    // -------------------------------------------------------------------------
    // DEBOUNCE: Validate touch duration
    // -------------------------------------------------------------------------
    uint32_t touchDuration = now - s_touchDownMs;
    bool debounceOk = s_pendingTouch && touched && (touchDuration >= TOUCH_DEBOUNCE_MS);

    // Mark activity for any valid touch
    if (debounceOk && !s_touchProcessed) {
        markActivity();
    }

    // -------------------------------------------------------------------------
    // IGNORE TAP: Skip UI processing for consumed taps
    // -------------------------------------------------------------------------
    if (g_ignoreTap) {
        if (!touched) {
            g_ignoreTap = false;
        }
        return;
    }

    // -------------------------------------------------------------------------
    // WAITING STATES: No touch input during wait animations
    // -------------------------------------------------------------------------
    if (currentState == WAITING_ANSWER || currentState == WAITING_TIME) {
        return;
    }

    // -------------------------------------------------------------------------
    // ANSWER STATE: Scroll handling
    // -------------------------------------------------------------------------
    if (currentState == ANSWER) {
        if (!touched) {
            // Touch release - check if it was a tap (short touch, no movement)
            if (g_lastTouchY >= 0 && !g_touchMoved) {
                uint32_t tapDuration = now - s_touchDownMs;
                if (tapDuration < TAP_MAX_DURATION_MS) {
                    currentState = IDLE;
                }
            }
            g_lastTouchY = -1;
            g_touchMoved = false;
            return;
        }

        // Touch start - record initial position
        if (g_lastTouchY < 0) {
            g_touchStartX = tp.x;
            g_touchStartY = tp.y;
            g_lastTouchY = tp.y;
            g_touchMoved = false;
            return;
        }

        // Touch drag - handle scrolling
        int dy = tp.y - g_lastTouchY;
        g_lastTouchY = tp.y;

        // Check for movement (distinguishes tap from scroll)
        if (abs(tp.x - g_touchStartX) > 3 || abs(tp.y - g_touchStartY) > 3) {
            g_touchMoved = true;
        }

        // Apply scroll
        if (dy != 0 && g_touchMoved) {
            int prevScroll = g_scrollY;
            g_scrollY += dy;

            // DEBUG: Print scroll values before clamping
            LOG("[TOUCH] dy=%d scrollY=%d->%d maxScroll=%d\n",
                          dy, prevScroll, g_scrollY, g_maxScroll);

            if (g_scrollY < 0) g_scrollY = 0;
            if (g_scrollY > g_maxScroll) g_scrollY = g_maxScroll;

            if (g_scrollY != prevScroll) {
                drawFullAnswerScreen();
            }
        }
        return;
    }

    // -------------------------------------------------------------------------
    // IDLE/RECORDING: Toggle recording on tap RELEASE
    // -------------------------------------------------------------------------
    // Detect tap on release (like ANSWER state does) for reliable detection
    if (currentState == IDLE || currentState == RECORDING) {
        if (justReleased && !s_touchProcessed) {
            // Touch just released - check if it was a valid tap
            uint32_t tapDuration = now - s_touchDownMs;
            if (tapDuration >= MIN_TOUCH_DURATION_MS && tapDuration < TAP_MAX_DURATION_MS) {
                s_touchProcessed = true;  // Prevent multiple triggers

                if (g_recordingInProgress) {
                    stopRecording();
                } else if (canSendControlMessages()) {
                    startRecording();
                }
            }
        }
    }
}
