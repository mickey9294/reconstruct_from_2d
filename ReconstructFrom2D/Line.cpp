#include "Line.h"



Line::Line()
{
	p1_ = 0;
	p2_ = 0;
}

Line::Line(int _p1, int _p2)
{
	if (_p1 < _p2)
	{
		p1_ = _p1;
		p2_ = _p2;
	}
	else
	{
		p1_ = _p2;
		p2_ = _p1;
	}
}

Line::Line(const Line & another)
{
	p1_ = another.p1();
	p2_ = another.p2();
}


Line::~Line()
{
}

Line & Line::operator=(const Line & another)
{
	p1_ = another.p1();
	p2_ = another.p2();

	return *this;
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

bool Line::operator==(const Line & another) const
{
	if (p1_ == another.p1() && p2_ == another.p2() 
		|| p1_ == another.p2() && p2_ == another.p1())
		return true;
	else
		return false;
}

bool Line::same_line(int p1, int p2)
{
	if (p1_ == p1 && p2_ == p2 || p1_ == p2 && p2_ == p1)
		return true;
	else
		return false;
}
