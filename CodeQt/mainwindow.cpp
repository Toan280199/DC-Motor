#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "string.h"
#include "QSerialPort"
#include "QDebug"
#include "QString"
#include "QFile"
#include "QTextStream"
#include "QMessageBox"
#include "qcustomplot.h"
#include "QThread"
#include "QTimer"
#include "QWidget"
#include "QCloseEvent"
#include "QPixmap"
#include "stdio.h"
#include "conio.h"
#include "QFile"
#include "QTableWidgetItem"
//Ket qua hieu chinh toc do: 0.05 0.03 0
//Ket qua hieu chinh vi tri: 0.107 0 0.002028

QSerialPort *serial;
QTimer *timer;
int j = 0, i=0, k=0;
int32_t row = 1;
QByteArray senddata, datasendfile;
QString textBuffer;
union float32bit
{
    float send;
    char sendbyte[4];  //tach so 32 bit thanh cac char 8 bit de truyen di
}gui, nhan;

float SetPoint=0, mydata = 0, y_max = 0, y_min = 0;
float Kp, Ki, Kd, pre_key=0, key=0, error=0, duty = 0, set_duty = 0;
QByteArray receiveData, tmpData;
int cnt1 = 0, cnt2 = 0;
char mode, pre_mode;

static const uint8_t CRC_8_TABLE[256] =
{
      0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
    157,195, 33,127,252,162, 64, 30, 95,  1,227,189, 62, 96,130,220,
     35,125,159,193, 66, 28,254,160,225,191, 93,  3,128,222, 60, 98,
    190,224,  2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
     70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89,  7,
    219,133,103, 57,186,228,  6, 88, 25, 71,165,251,120, 38,196,154,
    101, 59,217,135,  4, 90,184,230,167,249, 27, 69,198,152,122, 36,
    248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91,  5,231,185,
    140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
     17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
    175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
     50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
    202,148,118, 40,171,245, 23, 73,  8, 86,180,234,105, 55,213,139,
     87,  9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
    233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
    116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53
};
char Calc_CRC_8(const char *DataArray, const uint16_t Length);

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    setWindowFlags(Qt::FramelessWindowHint);
    setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
    this->setStyleSheet("background-color: white;");
    ui->setupUi(this);

    timer = new QTimer(this);
    timer->start();
    ui->SetDutyLE->setEnabled(true);
    QFont font;
    font.setPointSize(21);
    font.setItalic(true);
    font.setBold(true);
    setcustomplot(ui->customPlot);
    setcustomplot2(ui->customPlot_2);

    ui->DataTableWidget->setHorizontalHeaderLabels(QStringList() << "Time" << "Mode" << "Data" << "SetPoint");
    ui->DataTableWidget->verticalHeader()->setVisible(false);
    ui->DataTableWidget->setShowGrid(true);
    ui->DataTableWidget->setStyleSheet("QTableView {selection-background-color: red;}");

    connect(ui->ExitBt,SIGNAL(clicked()),this,SLOT(close()));
}

MainWindow::~MainWindow()
{
    delete ui;
    serial->close();
}

void MainWindow::closeEvent(QCloseEvent *event) //xac nhan truoc khi thoat
{
    if(QMessageBox::question(this,"Confirm","Are you want to exit?")==QMessageBox::No)
    {
        event->ignore();
    }
}


void MainWindow::on_Stopbt_clicked()    //dung dong co
{
   ui->SPvl->setText("0");
}

void MainWindow::on_DisconnectBt_clicked()
{
    serial->close();
    ui->TrangThai->setText("Disconected");
    QMessageBox::information(this,"","Disconected");
    ui->ConnectBt->setEnabled(true);
    ui->DisconnectBt->setEnabled(false);
    ui->Warning->setText("No Warning");
}

void MainWindow::on_ConnectBt_clicked()
{
    serial = new QSerialPort(this);
    serial->setPortName(ui->comboBox->currentText());
    serial->setBaudRate(QSerialPort::Baud115200);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);
    serial->open(QIODevice::ReadWrite);
    if (serial->NotOpen)
    {
        QMessageBox::warning(0,"Could not open COM",
                                        QObject::tr( "\n Could not open this COM"));
    }
    else
    {
        connect(serial,SIGNAL(readyRead()),this,SLOT(SerialReceived()));
        ui->TrangThai->setText("Connected");
        QMessageBox::information(this,"","Connected");
        ui->ConnectBt->setEnabled(false);
        ui->DisconnectBt->setEnabled(true);
    }
}

void MainWindow::on_Transmit_clicked()
{
    if(ui->TrangThai->text()=="Connected")
    {
        Kp = ui->Kpvl->text().toFloat();
        Ki = ui->Kivl->text().toFloat(); //chuyen cac noi dung trong o Kp, Ki, Kd sang kieu float
        Kd = ui->Kdvl->text().toFloat();
        set_duty = ui->SetDutyLE->text().toFloat();
        if (set_duty>99)
        {
            QMessageBox::warning(this,"","The duty range is -99 to 99");
            set_duty = 99;
            ui->SetDutyLE->setText("99");
        }
        if (set_duty<-99)
        {
            QMessageBox::warning(this,"","The duty range is from -99 to 99");
            set_duty = -99;
            ui->SetDutyLE->setText("-99");
        }
        SetPoint = ui->SPvl->text().toFloat();
        senddata = "A";
        if (ui->ModeSelectCBB->currentText()=="Velocity Control") mode = '1';
                else if(ui->ModeSelectCBB->currentText()=="Position Control") mode = '3';
                else mode = '2';
        senddata.append(mode);
        gui.send=Kp;
        for(int z=0; z<4;z++)
            {
               senddata.append(gui.sendbyte[z]);
            }
        gui.send=Ki;
        for(int z=0; z<4;z++)
            {
                senddata.append(gui.sendbyte[z]);
            }
        gui.send=Kd;
        for(int z=0; z<4;z++)
            {
                senddata.append(gui.sendbyte[z]);
            }
        gui.send=SetPoint;
        for(int z=0; z<4;z++)
            {
                senddata.append(gui.sendbyte[z]);
            }
        gui.send=set_duty;
        for(int z=0; z<4;z++)
            {
                senddata.append(gui.sendbyte[z]);
            }
        senddata.append("B");
        serial->write(senddata);
    }
    else
        {
           QMessageBox::warning(this,"","No COM was found");
        }
}

void MainWindow::setcustomplot(QCustomPlot *customPlot)
{
    customPlot->addGraph()->setName("Đồ thị dữ liệu theo thời gian");    // khởi tạo graph vẽ
    customPlot->graph(0)->setPen(QPen(QColor(40, 110, 255)));
    customPlot->addGraph()->setName("Data"); // khởi tạo graph vẽ
    customPlot->graph(1)->setPen(QPen(QColor(255, 110, 40)));

    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%s");
    customPlot->xAxis->setTicker(timeTicker);
    customPlot->axisRect()->setupFullAxesBox();
    customPlot->yAxis->setRange(y_min, y_max);
    ui->customPlot->xAxis->setLabel("Time (s)");
    ui->customPlot->yAxis->setLabel("Data");

    ui->customPlot->xAxis2->setVisible(true);
    ui->customPlot->yAxis->setVisible(true);
    ui->customPlot->xAxis2->setTicks(false);
    ui->customPlot->yAxis2->setTicks(false);

    connect(customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->xAxis2, SLOT(setRange(QCPRange)));
    connect(customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->yAxis2, SLOT(setRange(QCPRange)));
}

void MainWindow::setcustomplot2(QCustomPlot *customPlot)
{
    customPlot->addGraph()->setName("Đồ thị xung theo thời gian");    // khởi tạo graph vẽ
    customPlot->graph(0)->setPen(QPen(QColor(200, 110, 255)));

    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%s");
    customPlot->xAxis->setTicker(timeTicker);
    customPlot->axisRect()->setupFullAxesBox();
    customPlot->yAxis->setRange(-100, 100);
    ui->customPlot_2->xAxis->setLabel("Time (s)");
    ui->customPlot_2->yAxis->setLabel("PWM (%)");

    ui->customPlot_2->xAxis2->setVisible(true);
    ui->customPlot_2->yAxis->setVisible(true);
    ui->customPlot_2->xAxis2->setTicks(false);
    ui->customPlot_2->yAxis2->setTicks(false);

    connect(customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->xAxis2, SLOT(setRange(QCPRange)));
    connect(customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->yAxis2, SLOT(setRange(QCPRange)));
}

void MainWindow::on_SeeAll_clicked()
{
     pre_key=0;    //clear graph
}

void MainWindow::on_SeePart_clicked()
{
    pre_key=key;
    y_max = 0;
    y_min = 0;
}

void MainWindow::on_btn_Save_Graph_clicked()
{
    //save data
    QString outputDir = "E:/";
    ui->customPlot->saveJpg(outputDir+"/"+"mydata.jpg",1080,720,1,-1,96);
    ui->customPlot_2->saveJpg(outputDir+"/"+"myPWM.jpg",1080,720,1,-1,96);
}


void MainWindow::on_btn_Save_PID_clicked()
{
    QString outputDir = "E:/";
    QString fileName = "solieu.txt" ;
    QFile file(outputDir+"/"+fileName);
    QTextStream TextStream(&file);
    file.resize(0);
             //QString tenfile=outputDir+"/"+fileName;
    if (!file.open(QIODevice::WriteOnly|QIODevice::Truncate))
        {

             QMessageBox::warning(0,"Could not create Project File",
                                                 QObject::tr( "\n Could not create Project File on disk"));
        }
     else
        {
             datasendfile.append("Kp = ");
             datasendfile.append(ui->Kpvl->text());
             datasendfile.append("\n");

             datasendfile.append("Ki = ");
             datasendfile.append(ui->Kivl->text());
             datasendfile.append("\n");

             datasendfile.append("Kd = ");
             datasendfile.append(ui->Kdvl->text());
             datasendfile.append("\n");

             TextStream << datasendfile << endl;    //write data to file
             file.flush();
             file.close();
             datasendfile.clear();
         }
}
void MainWindow::SerialReceived()   //cu phap truyen "Y/N + error + data + duty + CRC + \n"
{
    receiveData = serial->readAll();
    for(cnt1 = 0; cnt1 < receiveData.length(); cnt1++)
    {
        tmpData[cnt2] = receiveData[cnt1];
        if(cnt2==27)
        {
            ui->Warning->setText("No Warning");
            qDebug()<<cnt2;
            cnt2 = 0;
            for(int k = 0;k<4;k++)
                nhan.sendbyte[k] = tmpData[k+1];
            if (nhan.send!=SetPoint) ui->Warning->setText("Frame error");
            for(k = 0; k<4; k++)
                nhan.sendbyte[k] = tmpData[k+5];
            mydata = nhan.send;
            for(k = 0; k<4; k++)
                nhan.sendbyte[k] = tmpData[k+9];
            duty = nhan.send;

            for(k = 0; k<4; k++)
                nhan.sendbyte[k] = tmpData[k+13];
            if (nhan.send!=Kp) ui->Warning->setText("Frame error");

            for(k = 0; k<4; k++)
                nhan.sendbyte[k] = tmpData[k+17];

            if (nhan.send!=Ki) ui->Warning->setText("Frame error");

            for(k = 0; k<4; k++)
                nhan.sendbyte[k] = tmpData[k+21];
            if (nhan.send!=Kd) ui->Warning->setText("Frame error");
            if (tmpData[25]!=mode) ui->Warning->setText("Frame error");

                if(ui->Warning->text()=="No Warning")
                {
                ui->SaiSoHienThi->display(SetPoint-mydata);
                ui->Value->display(mydata);
                static QTime time(QTime::currentTime());
                key = time.elapsed()/1000.0;

                if (mode!=pre_mode)
                {
                    pre_key = key;
                    y_max = 0;
                    y_min = 0;
                }
                //customplot1
                ui->customPlot->graph(0)->addData(key,SetPoint);    //graph 0 la SetPoint
                ui->customPlot->graph(1)->addData(key,mydata);    //graph 1 la Real Speed
                ui->customPlot->graph(0)->rescaleValueAxis();
                ui->customPlot->graph(1)->rescaleValueAxis();
                if (SetPoint>y_max) y_max = SetPoint;
                if (mydata>y_max) y_max = mydata;
                if (SetPoint<y_min) y_min = SetPoint;
                if (mydata<y_min) y_min = mydata;
                ui->customPlot->yAxis->setRange(y_min-100,y_max+100);
                ui->customPlot->xAxis->setRange(pre_key,key);      //scale truc x
                ui->customPlot->replot();

                //customplot2
                ui->customPlot_2->graph(0)->addData(key,duty);    //graph 0 la SetPoint
                ui->customPlot_2->graph(0)->rescaleValueAxis();
                ui->customPlot_2->yAxis->setRange(-110,110);    //scale truc y
                ui->customPlot_2->xAxis->setRange(pre_key,key);      //scale truc x
                ui->customPlot_2->replot();
                pre_mode = mode;

                //table widget
                ui->DataTableWidget->verticalScrollBar()->setValue(row-7);
                ui->DataTableWidget->setItem(row,0,new QTableWidgetItem(QString::number(key,'f',3)));
                ui->DataTableWidget->setItem(row,1,new QTableWidgetItem(QString(mode)));
                ui->DataTableWidget->setItem(row,2,new QTableWidgetItem(QString::number(mydata,'f',3)));
                ui->DataTableWidget->setItem(row,3,new QTableWidgetItem(QString::number(SetPoint,'f',3)));

                row++;
                if(row==25000)
                {
                    row = 1;
                    ui->DataTableWidget->clear();
                    ui->DataTableWidget->setHorizontalHeaderLabels(QStringList() << "Time" << "Mode" << "Data" << "SetPoint");
                }
                }
        }
        else cnt2++;
    }
}

void MainWindow::on_ModeSelectCBB_editTextChanged(const QString mytext)
{
    if (mytext == "Direct") ui->SetDutyLE->setEnabled(true);
    else ui->SetDutyLE->setEnabled(false);
}

char Calc_CRC_8(const char *DataArray, const uint16_t Length)
{
    uint8_t my_crc = 0;
    for (int i=0; i<Length; i++)
        my_crc = CRC_8_TABLE[my_crc ^ DataArray[i]];

    return my_crc;
}
