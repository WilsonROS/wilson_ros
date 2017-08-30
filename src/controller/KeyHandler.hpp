#ifndef KEYHANDLER_H_
#define KEYHANDLER_H_

#include <iostream>
#include <map>
#include <string>

#include <cstdarg>
#include <cstdio>
#include <cstdlib>

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xresource.h>

// Some redefinitions of X11 Special-Symbols as a quick reference
// If you need some other special characters, see /usr/include/X11/keysymdef.h

#define KEY_Tab		XK_Tab
#define KEY_Return	XK_Return
#define KEY_Esc		XK_Escape
#define KEY_Del		XK_Delete

#define KEY_Up		XK_Up
#define KEY_Down	XK_Down
#define KEY_Right	XK_Right
#define KEY_Left	XK_Left

typedef struct {
	bool isNewpressed;
	bool isHeld;
} _keyElement;

/**
 * @brief Manages an X11 window for the purpose of reading exact Keyboard Input states
 */
class KeyHandler {
public:
	/**
	 * Initializes a Window of passed width and height
	 */
	KeyHandler(int width, int height);

	/**
	 * Initializes a Window of 400x400 Pixel in size
	 */
	KeyHandler();

	~KeyHandler();

	/**
	 * @brief Processes window-events like keypresses and redraw-requests. Not calling this will also cause isPressed() and isHeld() to be stuck at the same state
	 */
	void process();

	/**
	 * @brief Checks if a key is newly pressed down
	 * @param c The key or key-code to check for
	 * @return New-Press status of the key
	 */
	bool isPressed(int c);

	/**
	 * @brief Checks if a key is held down
	 * @param c The key or key-code to check for
	 * @return The held status of the key
	 */
	bool isHeld(int c);
	/**
	 * @brief Any kind of close request by the window-system will be processed by this function
	 * @return Wheither or not closing the Window was requested
	 */
	bool pressedCloseButton();

	/**
	 * @brief Sets the title to show in the Taskbar and the Window-header
	 * @param title Window name
	 */
	void setWindowTitle(const char* title);

	/**
	 * @brief C-Style printf() function to print something to a specific line of the Window. Does not support linebreaks!
	 * @note The old text at the specified line will be deleted
	 * @param line A linenumber starting from 0 to a maximum of 100
	 * @param format A c-string supporting format specifiers like %d which will be replaced by the following arguments
	 */
	void printf(int line, const char* format, ...);

	/**
	 * @brief Writes a string to a specific line of the Window. Does not support linebreaks!
	 * @note The old text at the specified line will be deleted
	 * @param line A linenumber starting from 0 to a maximum of 100
	 * @param str The string to display
	 */
	void drawString(int line, std::string& str);

private:
	std::map<int, _keyElement> pressedKeys;	// A map for all pressed keys

	// Members about managing the X-Window
	void initialize(int width, int height);
	Display *display;
	Window window;
	XWindowAttributes win_attr;
	GC gc, gc_eraser;
	std::map<int, std::string> strMap;

	void redrawStrings();
	bool clickedClose;
};

#endif /* KEYHANDLER_H_ */
