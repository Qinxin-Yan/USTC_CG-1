#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QToolBar>

//QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
//QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots: void HelloWorld();

private:
    Ui::MainWindow *ui;
    QAction* hello_world_action_; // ��������
    QMenu* main_menu_; // �����˵�
    QMenu* main_menu_2;
    QToolBar* main_toolbar_; // ����������
    void CreateButtons(); // ��������
};
#endif // MAINWINDOW_H
