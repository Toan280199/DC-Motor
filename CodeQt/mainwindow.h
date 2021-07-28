#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include<QTimer>
#include <qcustomplot.h>


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void setcustomplot(QCustomPlot *customPlot);
    void setcustomplot2(QCustomPlot *customPlot);

protected:
    void closeEvent(QCloseEvent *event);

private slots:
    void SerialReceived();

    void on_DisconnectBt_clicked();

    void on_ConnectBt_clicked();

    void on_Transmit_clicked();

    void on_SeeAll_clicked();

    void on_SeePart_clicked();

    void on_Stopbt_clicked();

    void on_btn_Save_Graph_clicked();

    void on_btn_Save_PID_clicked();

    void on_PID_Read_clicked();

    void on_ModeSelectCBB_editTextChanged(const QString mytext);

    void on_ClearWarning_clicked();

private:
    Ui::MainWindow *ui;
    QTimer dataTimer;
};

#endif // MAINWINDOW_H
