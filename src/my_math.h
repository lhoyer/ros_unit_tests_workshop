#pragma once

class MyMath {
public:
	MyMath() {}
	int knobel(int x, int y);

protected:
	int mLastSquare = 0;
	int nextSquare();
};