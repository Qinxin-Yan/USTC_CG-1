#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <qmessagebox.h>
#include <QToolBar>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    CreateButtons();
}

 
MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::CreateButtons()
{
    // ������������ǩΪHello world
    //& �����ã����ô˶����Ŀ�ݼ�Ϊ��ǩ�ַ���������ĸ������Ϊ 'H'
    hello_world_action_ = new QAction(tr("&Hello world"), this);

    // ʹ�� connect �������ź������������
   // connect(�źŷ��䷽���źţ��źŽ��շ����ۺ���);
     connect(hello_world_action_, &QAction::triggered, this, &MainWindow::HelloWorld);

    // �����˵�����ǩΪMain
    main_menu_ = menuBar()->addMenu(tr("&Main"));
    main_menu_2 = menuBar()->addMenu(tr("&Main2"));

    // ������ Hello world ���ص� Main �˵���
    main_menu_->addAction(hello_world_action_);
    main_menu_2->addAction(hello_world_action_);

    // ��������������ǩΪ Main
    main_toolbar_ = addToolBar(tr("&Tool"));

    // ������ Hello world ���ص� Main ��������
    main_toolbar_->addAction(hello_world_action_);
}

void MainWindow::HelloWorld() {
    QMessageBox::about(this, tr("Hello world"),
        tr("Well done! Go ahead."));
}