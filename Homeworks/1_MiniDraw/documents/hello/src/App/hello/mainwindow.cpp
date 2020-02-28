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
    // 创建动作，标签为Hello world
    //& 的作用：设置此动作的快捷键为标签字符串的首字母，此例为 'H'
    hello_world_action_ = new QAction(tr("&Hello world"), this);

    // 使用 connect 函数将信号与槽连接起来
   // connect(信号发射方，信号，信号接收方，槽函数);
     connect(hello_world_action_, &QAction::triggered, this, &MainWindow::HelloWorld);

    // 创建菜单，标签为Main
    main_menu_ = menuBar()->addMenu(tr("&Main"));
    main_menu_2 = menuBar()->addMenu(tr("&Main2"));

    // 将动作 Hello world 加载到 Main 菜单中
    main_menu_->addAction(hello_world_action_);
    main_menu_2->addAction(hello_world_action_);

    // 创建工具栏，标签为 Main
    main_toolbar_ = addToolBar(tr("&Tool"));

    // 将动作 Hello world 加载到 Main 工具栏中
    main_toolbar_->addAction(hello_world_action_);
}

void MainWindow::HelloWorld() {
    QMessageBox::about(this, tr("Hello world"),
        tr("Well done! Go ahead."));
}