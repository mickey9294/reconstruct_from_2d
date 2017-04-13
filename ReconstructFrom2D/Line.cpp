#include "Line.h"



Line::Line()
{
	p1_ = 0;
	p2_ = 0;
}

Line::Line(int _p1, int _p2)
{
	p1_ = _p1;
	p2_ = _p2;
}

Line::Line(const Line & another)
{
	p1_ = another.p1();
	p2_ = another.p2();
}


Line::~Line()
{
}

int Line::p1() const
{
	return p1_;
}

int Line::p2() const
{
	return p2_;
}

void Line::setP1(int p1)
{
	p1_ = p1;
}

void Line::setP2(int p2)
{
	p2_ = p2;
}

int & Line::operator[](int index)
{
	assert(index >= 0 && index < 2);
	if (index == 0)
		return p1_;
	else
		return p2_;
}
