#include <QApplication>
#include "ui/MainWindow.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QApplication::setApplicationName("StepperHost");
    QApplication::setApplicationVersion("0.1.0");
    MainWindow window;
    window.show();
    return app.exec();
}
