#pragma once

/// Two-stage hood latch model (no Chrono / Irrlicht — pure, unit-testable).
///
/// A real hood has a primary latch (released by the interior cable lever under
/// the dash) and a secondary "safety catch" (released by hand under the leading
/// edge).  Only the primary latch carries an ajar switch, so the harness sees
/// only latched-vs-not — it cannot distinguish POPPED from OPEN.  ev1sim tracks
/// the full mechanical state; the published sensor (signal 6962) is derived from
/// it via ajar_sensor().
///
///   CLOSED   primary latch engaged, hood flush
///   POPPED   primary released; hood resting on the secondary safety catch
///   OPEN     safety catch released and hood raised (on prop / struts)
///
/// Transitions are one-way per call and no-op from the wrong state, so the UI
/// can wire one control per action without guarding.  Raising deliberately
/// requires POPPED first — you cannot jump CLOSED -> OPEN, mirroring the two
/// physical releases.
class Hood {
public:
    // Underlying values are explicit and stable: FormatHoodStateLabel() and the
    // SimApp status row map State by integer (0=CLOSED, 1=POPPED, 2=OPEN) via
    // static_cast<int>, so do not renumber these.
    enum class State { CLOSED = 0, POPPED = 1, OPEN = 2 };

    /// Interior cable release: CLOSED -> POPPED (pops onto the safety catch).
    void interior_release() { if (m_state == State::CLOSED) m_state = State::POPPED; }

    /// Trip the safety catch and lift: POPPED -> OPEN (must be popped first).
    void raise() { if (m_state == State::POPPED) m_state = State::OPEN; }

    /// The closing control: step one stage toward CLOSED
    /// (OPEN -> POPPED -> CLOSED), mirroring the two motions of closing a hood
    /// (lower onto the safety catch, then press past it to re-latch the primary).
    void lower_latch() {
        if (m_state == State::OPEN)        m_state = State::POPPED;
        else if (m_state == State::POPPED) m_state = State::CLOSED;
    }

    /// Quick keyboard shortcut (F): pop when closed, otherwise close one stage.
    /// Never raises — lifting requires the deliberate safety-catch action.
    void quick_toggle() {
        if (m_state == State::CLOSED) interior_release();
        else                          lower_latch();
    }

    State state() const { return m_state; }

    /// Primary-latch ajar switch — the only hood sensor.  Asserted whenever the
    /// hood is not fully closed; POPPED and OPEN read identically on the bus.
    bool ajar_sensor() const { return m_state != State::CLOSED; }

private:
    State m_state = State::CLOSED;
};
