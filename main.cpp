//
// Created by Ryan Shen on 2018/11/8.
//

//  ____       _       _    ____ _                 _ _____           _
// |  _ \ ___ (_)_ __ | |_ / ___| | ___  _   _  __| |_   _|__   ___ | |
// | |_) / _ \| | '_ \| __| |   | |/ _ \| | | |/ _` | | |/ _ \ / _ \| |
// |  __/ (_) | | | | | |_| |___| | (_) | |_| | (_| | | | (_) | (_) | |
// |_|   \___/|_|_| |_|\__|\____|_|\___/ \__,_|\__,_| |_|\___/ \___/|_|

#include <QtWidgets/QApplication>
#include "main_window.h"

int main(int argc, char **argv)
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}