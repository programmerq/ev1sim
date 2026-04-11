#pragma once

// macOS-specific platform fixes for Irrlicht.
// All functions are no-ops on non-Apple platforms.

/// Promote the process to a foreground application so the window
/// can receive focus.  Call after the Irrlicht device is created.
void macos_activate_app();

/// Disable Retina high-DPI backing on the OpenGL surface so the
/// Irrlicht viewport matches the window size.  Also installs a
/// window-resize observer that keeps the fix in sync when the
/// window is resized or enters/exits fullscreen.
/// Call before any rendering occurs but after the GL context exists.
void macos_fix_retina_viewport();

/// Create a standard macOS menu bar with:
///   - App menu:    Cmd+Q (Quit), Cmd+H (Hide)
///   - Window menu: Cmd+W (Close -> exits sim), Cmd+M (Minimize),
///                  Cmd+F (Toggle Fullscreen)
void macos_setup_menu_bar();

/// Enable the green titlebar fullscreen button on the Irrlicht window.
void macos_enable_fullscreen();
