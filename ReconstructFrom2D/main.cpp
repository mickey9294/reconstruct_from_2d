#include "ReconstructFrom2D.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	ReconstructFrom2D w;
	w.show();
	return a.exec();
}
