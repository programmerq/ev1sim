#pragma once

// macOS-specific platform fixes for Irrlicht.
// All functions are no-ops on non-Apple platforms.

#ifdef __APPLE__

// Real implementations live in MacOSPlatform.mm (Objective-C++).

/// Promote the process to a foreground application so the window
/// can receive focus.  Call after the Irrlicht device is created.
void macos_activate_app();

/// One-time setup: disable HiDPI backing on views and apply CGL
/// surface override.  Call after the GL context is created.
void macos_fix_retina_viewport();

/// Per-frame viewport fix: queries the actual framebuffer pixel
/// dimensions and calls glViewport accordingly.  Handles Retina
/// scaling, window resize, and fullscreen transitions.
/// Call once per frame after BeginScene().
void macos_apply_viewport();

/// Create a standard macOS menu bar with:
///   - App menu:    Cmd+Q (Quit), Cmd+H (Hide)
///   - Window menu: Cmd+W (Close -> exits sim), Cmd+M (Minimize),
///                  Cmd+F (Toggle Fullscreen)
void macos_setup_menu_bar();

/// Enable the green titlebar fullscreen button on the Irrlicht window.
void macos_enable_fullscreen();

#else

// Non-Apple platforms: inline no-op stubs so callers link without the .mm.
inline void macos_activate_app() {}
inline void macos_fix_retina_viewport() {}
inline void macos_apply_viewport() {}
inline void macos_setup_menu_bar() {}
inline void macos_enable_fullscreen() {}

#endif  // __APPLE__
