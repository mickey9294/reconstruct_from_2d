#pragma once

#include <assert.h>

class Line
{
public:
	Line();
	Line(int _p1, int _p2);
	Line(const Line &another);
	~Line();

	int p1() const;
	int p2() const;
	void setP1(int p1);
	void setP2(int p2);
	int & operator[](int index);

private:
	int p1_;
	int p2_;
};

