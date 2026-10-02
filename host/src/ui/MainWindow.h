#ifndef STEPPER_MAINWINDOW_H
#define STEPPER_MAINWINDOW_H
#include <QMainWindow>
#include <QThread>
class DeviceService;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;
signals:
    void connectRequested(QString endpoint);
    void disconnectRequested();
    void stopRequested();
private:
    QThread worker;
    DeviceService *service;
};
#endif
