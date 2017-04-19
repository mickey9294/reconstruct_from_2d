#pragma once

#include <assert.h>

class Line
{
public:
	Line();
	Line(int _p1, int _p2);
	Line(const Line &another);
	~Line();

	Line &operator=(const Line &another);

	int p1() const;
	int p2() const;
	void setP1(int p1);
	void setP2(int p2);
	int & operator[](int index);
	bool operator==(const Line &another) const;
	bool same_line(int p1, int p2);

private:
	int p1_;
	int p2_;
};

