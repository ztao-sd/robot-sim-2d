#ifndef BASIC_WINDOW_H
#define BASIC_WINDOW_H

#include <Windows.h>

/**
* Abstract class representing a window.
**/
template <class DerivedWindow>
class BasicWindow
{
protected:
	WNDCLASS m_wc;	// Window class
	HWND m_hwnd;	// Window handle
	virtual PCWSTR ClassName() const = 0; // Class name getter
	virtual LRESULT HandleMessage(UINT uMsg, WPARAM wParam, LPARAM lParam) = 0; // Window message handler

public:
	BasicWindow() : m_hwnd{ NULL }, m_wc{} {};
	BOOL Create(
		PCWSTR lpWindowName,		// Window text
		DWORD dwStyle,				// Window style	
		DWORD dwExStyle = 0,		// Extended Window style
		int x = CW_USEDEFAULT,		// X coordinate of window (upper-left corner)
		int y = CW_USEDEFAULT,		// Y coordinate of window (upper-left corner)
		int nWidth = CW_USEDEFAULT,	// Width (device units)
		int nHeight = CW_USEDEFAULT,// Height (device units)
		HWND hWndParent = 0,		// Handle to parent window
		HMENU hMenu = 0				// Handle to menu
	);
	HWND Window() const { return m_hwnd; } // Window handle getter
	static LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
	{
		DerivedWindow* pThis = NULL;

		if (uMsg == WM_NCCREATE)
		{
			CREATESTRUCT* pCreate = (CREATESTRUCT*)lParam;
			pThis = reinterpret_cast<DerivedWindow*>(pCreate->lpCreateParams);
			SetWindowLongPtr(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
			pThis->m_hwnd = hwnd;
		}
		else
		{
			pThis = reinterpret_cast<DerivedWindow*>(GetWindowLongPtr(hwnd, GWLP_USERDATA));
		}
		if (pThis)
		{
			return pThis->HandleMessage(uMsg, wParam, lParam);
		}
		else
		{
			return DefWindowProc(hwnd, uMsg, wParam, lParam);
		}
	}
};

// Safe memory deallocation of COM object.
template <class T> void SafeRelease(T** ppT);


#endif // !BASIC_CLASS_H



