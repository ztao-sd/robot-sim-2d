#include "basic_window.h"

template <class DerivedWindow>
BOOL BasicWindow<DerivedWindow>::Create(
	PCWSTR lpWindowName, // 
	DWORD dwStyle,
	DWORD dwExStyle,
	int x,
	int y,
	int nWidth,
	int nHeight,
	HWND hWndParent,
	HMENU hMenu
)
{
	m_wc = {};
	m_wc.lpfnWndProc = DerivedWindow::WindowProc;	// Window message handler
	m_wc.hInstance = GetModuleHandle(NULL);			// Application instance handle
	m_wc.lpszClassName = ClassName();				// Class name

	RegisterClass(&m_wc);	// Register Window class

	m_hwnd = CreateWindowEx(	// Create new instance of a window
		dwExStyle,		// Extended Window style
		ClassName(),	// Class name (optional)
		lpWindowName,	// Window text
		dwStyle,		// Window style
		x,				// X coordinate
		y,				// Y coordinate
		nWidth,			// Width
		nHeight,		// Height
		hWndParent,		// Parent Window
		hMenu,			// Menu Window
		GetModuleHandle(NULL),	// Instance handle
		this			// Additional data
	);

	return (m_hwnd ? TRUE : FALSE);
}

template <class T> void SafeRelease(T** ppT)
{
	if (*ppT)
	{
		(*ppT)->Release();
		*ppT = NULL;
	}
}