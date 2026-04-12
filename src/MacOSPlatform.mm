#include "MacOSPlatform.h"

#ifdef __APPLE__

// Irrlicht uses OpenGL, which Apple deprecated in 10.14.  Silence warnings.
#define GL_SILENCE_DEPRECATION
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"

#import <Cocoa/Cocoa.h>
#import <OpenGL/OpenGL.h>
#import <OpenGL/gl.h>

// ── Helpers ────────────────────────────────────────────────────────────────

/// Robust window finder — tries mainWindow, keyWindow, then any visible window.
static NSWindow* find_irrlicht_window() {
    NSWindow* win = [NSApp mainWindow];
    if (!win) win = [NSApp keyWindow];
    if (!win) {
        for (NSWindow* w in [NSApp windows]) {
            if ([w isVisible]) { win = w; break; }
        }
    }
    return win;
}

/// Walk the entire view tree, setting wantsBestResolutionOpenGLSurface:NO.
static void disable_hidpi_recursive(NSView* view) {
    view.wantsBestResolutionOpenGLSurface = NO;
    for (NSView* sub in [view subviews])
        disable_hidpi_recursive(sub);
}

/// Apply the CGL backing-size fix for a given content-view size.
/// Locks the GL framebuffer to the logical window dimensions so
/// Irrlicht's glViewport matches the actual pixel count.
static void apply_cgl_backing(CGLContextObj cgl, int w, int h) {
    if (!cgl) return;
    GLint dim[2] = { (GLint)w, (GLint)h };
    CGLSetParameter(cgl, kCGLCPSurfaceBackingSize, dim);
    CGLEnable(cgl, kCGLCESurfaceBackingSize);
}

// ── Resize observer ────────────────────────────────────────────────────────

/// Stored CGL context for use in the notification handler.
/// Captured once during macos_fix_retina_viewport() while the GL
/// context is guaranteed to be current.
static CGLContextObj s_cgl_context = NULL;

@interface EV1WindowObserver : NSObject
- (void)windowResized:(NSNotification*)note;
@end

@implementation EV1WindowObserver
- (void)windowResized:(NSNotification*)note {
    if (!s_cgl_context) return;
    NSWindow* win = (NSWindow*)[note object];
    NSView* content = [win contentView];
    if (!content) return;

    NSRect rect = [content frame];
    apply_cgl_backing(s_cgl_context, (int)rect.size.width, (int)rect.size.height);

    // Also update the NSOpenGLContext if one is current.
    NSOpenGLContext* nsCtx = [NSOpenGLContext currentContext];
    if (nsCtx) [nsCtx update];
}
@end

static EV1WindowObserver* s_observer = nil;

// ── Public API ─────────────────────────────────────────────────────────────

void macos_activate_app() {
    // A process launched from the terminal is a "background" app by default.
    // Promote it to a regular GUI app so the window can be focused and the
    // menu bar appears.
    [NSApp setActivationPolicy:NSApplicationActivationPolicyRegular];
    [NSApp activateIgnoringOtherApps:YES];

    NSWindow* win = find_irrlicht_window();
    if (win)
        [win makeKeyAndOrderFront:nil];
}

void macos_fix_retina_viewport() {
    NSWindow* win = find_irrlicht_window();
    if (!win) return;

    NSView* content = [win contentView];
    if (!content) return;

    // ── Strategy 1: NSView property ──
    // Tell every view in the hierarchy not to use HiDPI backing.
    disable_hidpi_recursive(content);

    // ── Strategy 2: CGL surface backing override ──
    // Force the CGL context to use a fixed surface size equal to
    // the logical window dimensions (not 2x Retina).
    s_cgl_context = CGLGetCurrentContext();
    NSRect rect = [content frame];
    apply_cgl_backing(s_cgl_context, (int)rect.size.width, (int)rect.size.height);

    // ── Strategy 3: NSOpenGLContext update ──
    NSOpenGLContext* nsCtx = [NSOpenGLContext currentContext];
    if (nsCtx) [nsCtx update];

    // ── Force Irrlicht to re-layout ──
    // Toggle the window frame by 1 pixel.  This triggers Irrlicht's resize
    // handler which re-queries the framebuffer dimensions.
    NSRect frame = [win frame];
    frame.size.height += 1;
    [win setFrame:frame display:YES animate:NO];
    frame.size.height -= 1;
    [win setFrame:frame display:YES animate:NO];

    // ── Install resize observer ──
    // Re-applies the CGL fix whenever the window is resized or goes
    // fullscreen so the framebuffer always matches the logical size.
    if (!s_observer) {
        s_observer = [[EV1WindowObserver alloc] init];
        NSNotificationCenter* nc = [NSNotificationCenter defaultCenter];
        [nc addObserver:s_observer
               selector:@selector(windowResized:)
                   name:NSWindowDidResizeNotification
                 object:win];
        [nc addObserver:s_observer
               selector:@selector(windowResized:)
                   name:NSWindowDidEnterFullScreenNotification
                 object:win];
        [nc addObserver:s_observer
               selector:@selector(windowResized:)
                   name:NSWindowDidExitFullScreenNotification
                 object:win];
    }
}

void macos_setup_menu_bar() {
    NSMenu* mainMenu = [[NSMenu alloc] init];

    // ── App menu (bold "EV1 Simulator" in the menu bar) ──
    NSMenuItem* appMenuItem = [[NSMenuItem alloc] init];
    NSMenu* appMenu = [[NSMenu alloc] initWithTitle:@"EV1 Simulator"];

    [appMenu addItemWithTitle:@"About EV1 Simulator"
                       action:@selector(orderFrontStandardAboutPanel:)
                keyEquivalent:@""];
    [appMenu addItem:[NSMenuItem separatorItem]];
    [appMenu addItemWithTitle:@"Hide EV1 Simulator"
                       action:@selector(hide:)
                keyEquivalent:@"h"];
    NSMenuItem* hideOthers =
        [appMenu addItemWithTitle:@"Hide Others"
                           action:@selector(hideOtherApplications:)
                    keyEquivalent:@"h"];
    [hideOthers setKeyEquivalentModifierMask:
        NSEventModifierFlagCommand | NSEventModifierFlagOption];
    [appMenu addItemWithTitle:@"Show All"
                       action:@selector(unhideAllApplications:)
                keyEquivalent:@""];
    [appMenu addItem:[NSMenuItem separatorItem]];
    [appMenu addItemWithTitle:@"Quit EV1 Simulator"
                       action:@selector(terminate:)
                keyEquivalent:@"q"];

    [appMenuItem setSubmenu:appMenu];
    [mainMenu addItem:appMenuItem];

    // ── Window menu ──
    NSMenuItem* windowMenuItem = [[NSMenuItem alloc] init];
    NSMenu* windowMenu = [[NSMenu alloc] initWithTitle:@"Window"];

    [windowMenu addItemWithTitle:@"Close"
                          action:@selector(performClose:)
                   keyEquivalent:@"w"];
    [windowMenu addItemWithTitle:@"Minimize"
                          action:@selector(performMiniaturize:)
                   keyEquivalent:@"m"];
    [windowMenu addItemWithTitle:@"Zoom"
                          action:@selector(performZoom:)
                   keyEquivalent:@""];
    [windowMenu addItem:[NSMenuItem separatorItem]];
    [windowMenu addItemWithTitle:@"Toggle Full Screen"
                          action:@selector(toggleFullScreen:)
                   keyEquivalent:@"f"];

    [windowMenuItem setSubmenu:windowMenu];
    [mainMenu addItem:windowMenuItem];

    [NSApp setMainMenu:mainMenu];
    [NSApp setWindowsMenu:windowMenu];
}

void macos_enable_fullscreen() {
    NSWindow* win = find_irrlicht_window();
    if (!win) return;

    // Enable the green fullscreen button in the traffic-light title bar
    // and allow the window to be freely resized.
    win.collectionBehavior |= NSWindowCollectionBehaviorFullScreenPrimary;
    win.styleMask |= NSWindowStyleMaskResizable;
}

void macos_apply_viewport() {
    // Query the actual framebuffer pixel dimensions each frame.
    // On Retina this is 2× the logical size; after resize/fullscreen
    // it reflects the new window geometry.  Override Irrlicht's
    // glViewport so rendering fills the entire framebuffer.
    NSWindow* win = find_irrlicht_window();
    if (!win) return;
    NSView* content = [win contentView];
    if (!content) return;

    NSRect backing = [content convertRectToBacking:[content bounds]];
    glViewport(0, 0,
               (GLsizei)backing.size.width,
               (GLsizei)backing.size.height);
}

#pragma clang diagnostic pop

#else  // !__APPLE__

// No-ops on non-Apple platforms.
void macos_activate_app()          {}
void macos_fix_retina_viewport()   {}
void macos_setup_menu_bar()        {}
void macos_enable_fullscreen()     {}
void macos_apply_viewport()        {}

#endif
