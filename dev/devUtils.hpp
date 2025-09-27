#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <format>

using namespace Eigen;
using Eigen::MatrixXd;
using namespace std;

string objToBin(void *obj, size_t obj_size)
{
    unsigned char *pnt = reinterpret_cast<unsigned char *>(obj);
    ostringstream out;
    for (size_t i = 0; i < obj_size; i++)
    {
        for (size_t j = 0; j < 8; j++)
            out << !!((pnt[i] << j) & 0x80);
        out << " ";
    }
    return out.str();
}

template <typename... Args>
string dyna_print(string_view rt_fmt_str, Args &&...args)
{
    return vformat(rt_fmt_str, make_format_args(args...));
}
void drawSectionLine(string sectionTitle = "")
{
    const char *sectionLine = "─";
    const size_t sectionLineLength = 161;
    if (sectionTitle != "")
        sectionTitle = "« " + sectionTitle + " »";
    const size_t sectionTitleSize = sectionTitle.size();

    cout << endl;
    for (int i = 0; i < sectionLineLength; ++i)
    {
        if (i == (floor(sectionLineLength / 2) - floor(sectionTitleSize / 2)))
        {
            cout << sectionTitle;
            i += sectionTitleSize;
        }
        else
            cout << sectionLine;
    }
    cout << endl
         << endl;
}

template <typename Derived>
void print_mat(const MatrixBase<Derived> &mat, bool newline = true, string formatstr = "  % 16.10G│")
{
    if (newline)
        cout << endl;
    for (size_t x = 0; x < mat.rows(); ++x)
    {
        for (size_t y = 0; y < mat.cols(); ++y)
            printf(formatstr.c_str(), mat(x, y));
        if (newline)
            cout << endl;
    }
    if (newline)
        cout << endl;
}

template <typename Derived>
void printf_mat(const MatrixBase<Derived> &mat, string &out, bool newline = true, string formatstr = " {: 16.10G}│")
{
    if (newline)
        out += "\r\n";
    for (size_t x = 0; x < mat.rows(); ++x)
    {
        for (size_t y = 0; y < mat.cols(); ++y)
            out += dyna_print(formatstr, mat(x, y));
        if (newline)
            out += "\r\n";
    }
    if (newline)
        out += "\r\n";
}

static void drawProgressBar(int iteration, int totalIterations, int progressWidth = 60)
{
    totalIterations = totalIterations - 1;
    float percent = (iteration * 100.0f) / totalIterations;
    int filledWidth = (iteration * progressWidth) / totalIterations;

    cout << "\r│";
    for (int i = 0; i < filledWidth; ++i)
        cout << "▓";

    cout << string(progressWidth - filledWidth, ' ')
         << "│ "
         << setw(3) << (int)percent << "% ";

    cout.flush();
}
