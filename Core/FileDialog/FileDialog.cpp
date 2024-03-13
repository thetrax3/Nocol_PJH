#include "FileDialog.h"
#include <windows.h>
#include <commdlg.h>

std::string FileDialog::openFileDialog()
{
    // ���� ��ȭ ���� ����
    OPENFILENAME ofn;
    CHAR szFile[260] = {0};

    ZeroMemory(&ofn, sizeof(ofn));
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = NULL;
    ofn.lpstrFile = szFile;
    ofn.lpstrFile[0] = '\0';
    ofn.nMaxFile = sizeof(szFile);
    ofn.lpstrFilter = "All\0*.*\0";
    ofn.nFilterIndex = 1;
    ofn.lpstrInitialDir = NULL;
    ofn.lpstrTitle = "Select a file";
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR;

    // ���� ��ȭ ���� ����
    if (GetOpenFileNameA(&ofn) == TRUE)
    {
        return ofn.lpstrFile; // ������ ���� ��� ��ȯ
    }
    else
    {
        return ""; // ����ڰ� ����� ��� �� ���ڿ� ��ȯ
    }
}
