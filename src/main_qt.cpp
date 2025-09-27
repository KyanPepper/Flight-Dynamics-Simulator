#include <QApplication>
#include <QLabel>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QLabel hello("Hello from Qt 6!");
    hello.resize(320, 60);
    hello.show();
    return app.exec();
}
