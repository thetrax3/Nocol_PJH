#include "FileDialog.h"
#include <windows.h>
#include <commdlg.h>

std::string FileDialog::openFileDialog()
{
    // 파일 대화 상자 생성
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

    // 파일 대화 상자 열기
    if (GetOpenFileNameA(&ofn) == TRUE)
    {
        return ofn.lpstrFile; // 선택한 파일 경로 반환
    }
    else
    {
        return ""; // 사용자가 취소한 경우 빈 문자열 반환
    }
}
