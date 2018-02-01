/*****************************************************************
 *
 * HwDLLMain.cpp
 *
 * Shanghai Share-E tech. Co., LTD.
 *
 ****************************************************************/
    
#include "rtx_inc.h"

BOOL RTAPI DllMain(HINSTANCE hinstDLL, DWORD fdwReason, LPVOID lpvReserved)
{
   switch (fdwReason)
   {
		case DLL_PROCESS_ATTACH:
			//INIT_HW_TASK(&g_Task);
			// Indicates that the DLL is being loaded into the virtual address
			// space of the current process as a result of a call to
			// LoadLibrary (either explicitly or implicitly).
			break;

		case DLL_PROCESS_DETACH:
			//RESET_HW_TASK(&g_Task);
			// Indicates that the DLL is being unloaded from the virtual address
			// space of the calling process as a result of either a process exit
			// or a call to FreeLibrary (either explicitly or implicitly).
			break;
    }    

	return TRUE;
}
