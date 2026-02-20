#include <QApplication>
#include <iostream>
#include <QStringList>
#include "../include/ebimu/main_window.hpp"

int main(int argc, char **argv)
{
    QApplication app(argc, argv);

    QString geometryArg;

    for (int i = 0; i < argc; ++i)
    {
        QString arg(argv[i]);
        if (arg.startsWith("--pm-geometry="))
        {
            geometryArg = arg.section("=", 1);
        }
    }

    e2box_imu::MainWindow w;

    if (!geometryArg.isEmpty())
    {
        QStringList parts = geometryArg.split("+");
        QString sizePart = parts[0];

        int x = parts.size() > 1 ? parts[1].toInt() : 0;
        int y = parts.size() > 2 ? parts[2].toInt() : 0;

        QStringList sizeSplit = sizePart.split("x");
        int width = sizeSplit[0].toInt();
        int height = sizeSplit[1].toInt();

        w.setGeometry(x, y, width, height);
    }

    w.show();
    return app.exec();
}