#!/usr/bin/env python3
"""Resize an X11 window to fill the screen when no window manager is running."""
import ctypes
import ctypes.util
import sys


def load_x11():
    name = ctypes.util.find_library("X11") or "libX11.so.6"
    x11 = ctypes.CDLL(name)
    x11.XOpenDisplay.restype = ctypes.c_void_p
    x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
    x11.XDefaultRootWindow.restype = ctypes.c_ulong
    x11.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
    x11.XQueryTree.argtypes = [
        ctypes.c_void_p,
        ctypes.c_ulong,
        ctypes.POINTER(ctypes.c_ulong),
        ctypes.POINTER(ctypes.c_ulong),
        ctypes.POINTER(ctypes.POINTER(ctypes.c_ulong)),
        ctypes.POINTER(ctypes.c_uint),
    ]
    x11.XFetchName.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.POINTER(ctypes.c_char_p)]
    x11.XMoveResizeWindow.argtypes = [
        ctypes.c_void_p,
        ctypes.c_ulong,
        ctypes.c_int,
        ctypes.c_int,
        ctypes.c_uint,
        ctypes.c_uint,
    ]
    x11.XFlush.argtypes = [ctypes.c_void_p]
    return x11


def children(x11, display, window):
    root = ctypes.c_ulong()
    parent = ctypes.c_ulong()
    kids = ctypes.POINTER(ctypes.c_ulong)()
    count = ctypes.c_uint()
    if x11.XQueryTree(display, window, ctypes.byref(root), ctypes.byref(parent),
                      ctypes.byref(kids), ctypes.byref(count)) == 0:
        return []
    return [kids[i] for i in range(count.value)]


def title(x11, display, window):
    name = ctypes.c_char_p()
    if x11.XFetchName(display, window, ctypes.byref(name)) and name.value:
        return name.value.decode("utf-8", "replace")
    return ""


def main() -> int:
    width = int(sys.argv[1]) if len(sys.argv) > 1 else 1600
    height = int(sys.argv[2]) if len(sys.argv) > 2 else 900
    needle = sys.argv[3].lower() if len(sys.argv) > 3 else "rviz"
    x11 = load_x11()
    display = x11.XOpenDisplay(None)
    if not display:
        print("cannot open display", file=sys.stderr)
        return 1
    root = x11.XDefaultRootWindow(display)
    stack = [root]
    resized = 0
    while stack:
        window = stack.pop()
        stack.extend(children(x11, display, window))
        name = title(x11, display, window).lower()
        if name and needle in name:
            x11.XMoveResizeWindow(display, window, 0, 0, width, height)
            print(f"resized {name!r} to {width}x{height}")
            resized += 1
    x11.XFlush(display)
    return 0 if resized else 2


if __name__ == "__main__":
    raise SystemExit(main())
