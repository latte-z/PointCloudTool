//
// Created by Ryan Shen on 2018/11/8.
//

#include <QtWidgets/QApplication>
#include "main_window.h"

int main(int argc, char **argv)
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}