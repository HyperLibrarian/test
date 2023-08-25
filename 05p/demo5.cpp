#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <stdio.h>

using namespace std;

extern "C"
{

// 主函数
int IntCal(int a, int b)
{
	int c;
	
	c = a + b;

	string s;
	s = to_string(a) + " is an int";

	cout << s << endl;


	return c;
}

// 主函数
double DoubleCal(double a, double b)
{
	double c;
	c = a + b;


	return c;
}


}

