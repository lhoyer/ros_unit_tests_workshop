#include "my_math.h"
#include <stdexcept>

int MyMath::knobel(int x, int y) {
	if (x < 0 || y < 0) throw std::invalid_argument("received negative value");
	if (x == 0)
		return y;
	while (y != 0) {
		if (x > y) x = x -y;
		else y = y - x;
	}
	return x;
}

int MyMath::nextSquare() {
	int a=0, b=0, c=0;
	mLastSquare++;
	while (a <= mLastSquare) {
		a = a+1;
		b = a+a-1;
		c = b+c;
	}
	return c;
}