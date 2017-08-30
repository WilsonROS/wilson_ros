#include "KeyHandler.hpp"

KeyHandler::KeyHandler(int width, int height) {
	// Small sanity check because too small dimensions might cause the window to be invisible or being spawned outsize of vision
	if(width<=50) width = 400;
	if(height<=50) height = 400;
	initialize(width, height);
}

KeyHandler::KeyHandler()
{
	initialize(400, 400);
}

KeyHandler::~KeyHandler() {
	XFreeGC(display, gc);
	XFreeGC(display, gc_eraser);
	XDestroyWindow(display, window);
	XCloseDisplay(display);
}

void KeyHandler::initialize(int width, int height)
{
	// First connect to the display server
	display = XOpenDisplay(NULL);
	if (!display) {
		std::cerr << "Failed to connect to x-display. Exiting" << std::endl;
		exit(1);
	}

	// Create the window
	window = XCreateSimpleWindow(display, XRootWindow(display, 0), 0, 0, width, height, 5,
			XBlackPixel(display, 0), XWhitePixel(display, 0));

	// Create&Define the Graphical context
	gc = XCreateGC(display, window, 0, 0);
	XSetBackground(display, gc, XWhitePixel(display, 0));
	XSetForeground(display, gc, XBlackPixel(display, 0));
	gc_eraser = XCreateGC(display, window, 0, 0);
	XSetBackground(display, gc_eraser, XWhitePixel(display, 0));
	XSetForeground(display, gc_eraser, XWhitePixel(display, 0));

	// Set a telling Window-Titlebar name
	XStoreName(display, window, "KeyHandler-Window");

	// tell the display server what kind of events we would like to see
	XSelectInput(display, window,
	StructureNotifyMask | KeyPressMask | KeyReleaseMask | KeymapStateMask | ExposureMask);

	// We want to catch close-button clicks
	Atom WM_DELETE_WINDOW = XInternAtom(display, "WM_DELETE_WINDOW", False);
	XSetWMProtocols(display, window, &WM_DELETE_WINDOW, 1);
	clickedClose = false;

	// Clear & Show the window
	XClearWindow(display, window);
	XMapWindow(display, window);
}

/**
 * @details Stores the string in the map<int,string> structure for the purpose of constant redraws
 */
void KeyHandler::drawString(int line, std::string& str) {
	// A small sanity check to not store unnecessary values
	if (line < 0 || line > 100)
		return;

	// Do not save and draw again when the new string matches the existing one
	if (str.compare(strMap[line]) == 0)
		return;

	// God bless Cpp, cannot image how work intensive this line would have been without maps lvalue[] and string-copying
	strMap[line] = str;

	XFillRectangle(display, window, gc_eraser, 5, 2 + line * 12, win_attr.width - 5, 12);

	XDrawString(display, window, gc, 5, (1 + line) * 12, str.c_str(), str.length());
}

/**
 * @details A c-style wrapper for drawString. It processes a c-formatted string just to create a std::string of it for a drawString() call
 */
void KeyHandler::printf(int line, const char* format, ...) {
	char buf[255];

	va_list args;
	va_start(args, format);
	vsnprintf(buf, 255, format, args);
	va_end(args);

	std::string tmp = std::string(buf);
	drawString(line, tmp);
	return;
}

void KeyHandler::setWindowTitle(const char* title) {
	XStoreName(display, window, title);
}

bool KeyHandler::pressedCloseButton() {
	return clickedClose;
}

void KeyHandler::redrawStrings() {
	// Clear the window first
	XClearWindow(display, window);

	for (std::map<int, std::string>::iterator it = strMap.begin(); it != strMap.end(); ++it) {
		XDrawString(display, window, gc, 5, (1 + it->first) * 12, it->second.c_str(),
				it->second.length());
	}
}

bool KeyHandler::isPressed(int c) {
	// c has to be in pressedKeys and isNewpressed must be set
	return (pressedKeys.find(c) != pressedKeys.end()) && pressedKeys[c].isNewpressed;
}

bool KeyHandler::isHeld(int c) {
	// c has to be in pressedKeys and isHeld must be set
	// * Alternatively isNewpressed should also indicate the Held-State. This is a workaround for extremely short keypresses which happen durch sleeps
	return (pressedKeys.find(c) != pressedKeys.end()) && (pressedKeys[c].isHeld || pressedKeys[c].isNewpressed);
}

void KeyHandler::process() {
	// First, set all previously pressed keys to not being newpressed anymore
	for (std::map<int, _keyElement>::iterator it = pressedKeys.begin(); it != pressedKeys.end();
			++it)
		it->second.isNewpressed = false;

	XEvent ev;
	int key;
	while (XPending(display)) {
		XNextEvent(display, &ev);
		switch (ev.type) {

		case Expose: // The window has been changed/moved/resized/refocused. Everything has to be redrawn
			XGetWindowAttributes(display, window, &win_attr);
			redrawStrings();
			break;

		case ClientMessage:	// The window was closed (actually it could be more, but thats the only message we requested)
			clickedClose = true;
			break;

		case KeymapNotify:	// Not sure what exactly this does, but it is included in most example code I found
			XRefreshKeyboardMapping(&ev.xmapping);
			break;

		case KeyPress:
			key = XLookupKeysym(&ev.xkey, 0);
			pressedKeys[key].isHeld = true;
			pressedKeys[key].isNewpressed = true;
			break;

		case KeyRelease:
			key = XLookupKeysym(&ev.xkey, 0);

			// Held keys are constantly re-pressed
			// Check if the key is actually being released or if the next event instantly retriggers the key
			unsigned short is_retriggered = 0;
			if (XEventsQueued(display, QueuedAfterReading)) {
				XEvent next_ev;
				XPeekEvent(display, &next_ev);	// Peeks at the next event without taking it from the queue

				if (next_ev.type == KeyPress && next_ev.xkey.time == ev.xkey.time
						&& next_ev.xkey.keycode == ev.xkey.keycode) {
					// The next event actually is triggering the key which was just notified as released
					// Read the next event (the one we just peeked at) without processing it (e.g. skip it)
					XNextEvent(display, &ev);
					is_retriggered = 1;
				}
			}

			if (!is_retriggered) {
				/*
				 * In theory, the element could be erased from the map
				 * But it is likely that the key will be pressed again sooner or later.
				 * So instead of always deleting&recreating it, the struct is just kept in memory
				 */
				pressedKeys[key].isHeld = false;
			}
			break;
		}
	}
}

